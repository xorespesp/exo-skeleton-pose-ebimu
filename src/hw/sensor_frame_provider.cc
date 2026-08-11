#include "sensor_frame_provider.hh"

#include "backends/k4a_frame_source.hh"
#ifdef EXO_HAS_VZ_BACKEND
#include "backends/vz_frame_source.hh"
#endif

#include "io/mcap_record_player.hh"

#include <spdlog/spdlog.h>

#include <exception>
#include <format>
#include <utility>
#include <vector>

namespace hw
{
    namespace
    {
        constexpr auto kIdlePollSleep = std::chrono::milliseconds{ 5 };
        constexpr float kFpsEmaAlpha = 0.1f; // weight of the newest sample

        template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
    }

    sensor_frame_provider::~sensor_frame_provider()
    {
        this->close();
    }

    void sensor_frame_provider::add_observer(std::shared_ptr<sensor_frame_observer> observer)
    {
        if (!observer) { return; }
        std::scoped_lock lk{ _observers_mtx };
        _observers.push_back(std::move(observer));
    }

    void sensor_frame_provider::remove_observer(const std::shared_ptr<sensor_frame_observer>& observer)
    {
        std::scoped_lock lk{ _observers_mtx };
        std::erase(_observers, observer);
    }

    bool sensor_frame_provider::is_opened() const
    {
        std::scoped_lock lk{ _source_mtx };
        return static_cast<bool>(_source);
    }

    bool sensor_frame_provider::open(const source_config_t& config) noexcept try
    {
        this->close();

        std::unique_ptr<sensor_frame_source> source = std::visit(overloaded{
            [](const k4a_device_config_t& c) -> std::unique_ptr<sensor_frame_source> {
                auto s = std::make_unique<k4a_device_capturer>();
                if (!s->open(c.device_index, { c.exposure_us, c.gain }, c.color_format)) {
                    return nullptr;
                }
                return s;
            },
            [](const vz_device_config_t& c) -> std::unique_ptr<sensor_frame_source> {
#ifdef EXO_HAS_VZ_BACKEND
                auto s = std::make_unique<vz_frame_source>();
                if (!s->open(c)) { return nullptr; }
                return s;
#else
                spdlog::error("provider: this build carries no VZ camera backend");
                return nullptr;
#endif
            },
            [](const recording_config_t& c) -> std::unique_ptr<sensor_frame_source> {
                auto s = std::make_unique<io::mcap_record_player>();
                if (!s->open(c.file)) { return nullptr; }
                return s;
            },
        }, config);

        if (!source) { return false; }

        // Every config kind carries a `color_roi`, so this stays generic.
        const std::optional<roi_t> requested_roi = std::visit(
            [](const auto& c) { return c.color_roi; }, 
            config
        );

        this->_install_source(
            std::move(source),
            hw::get_source_backend(config),
            describe(config),
            requested_roi
        );
        spdlog::info("provider opened: {} ({}, {})"
            , _source_name
            , frame_format_to_str(_color_format)
            , _color_roi.has_value()
                ? std::format("roi {}x{}+{}+{}", _color_roi->width, _color_roi->height, _color_roi->x, _color_roi->y)
                : std::string{ "whole frame" }
        );
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("provider: failed to open {}: {}", describe(config), e.what());
        return false;
    }

    void sensor_frame_provider::_install_source(
        std::unique_ptr<sensor_frame_source> source,
        const source_backend_t source_backend,
        std::string source_name,
        const std::optional<roi_t>& requested_roi)
    {
        // Cache metadata once; it stays constant while streaming.
        _calib = source->get_calibration();
        _color_format = source->get_color_format();
        _color_roi.reset();

        if (requested_roi.has_value())
        {
            if (const std::optional<roi_t> granted = source->try_set_color_roi(*requested_roi);
                granted.has_value())
            {
                _color_roi = granted;
                apply_roi(_calib, *granted); // the delivered images are what the calibration describes
            }
            else
            {
                spdlog::warn("provider: ROI {}x{}+{}+{} was not applied to {}'s {}x{} frame"
                    , requested_roi->width, requested_roi->height, requested_roi->x, requested_roi->y
                    , source_name
                    , _calib.frame_resolution.x(), _calib.frame_resolution.y()
                );
            }
        }

        // Copied from `_calib`, which the ROI above already adjusted, so it cannot disagree with it.
        _color_frame_resolution = _calib.frame_resolution;
        _source_backend = source_backend;
        _source_name = std::move(source_name);

        _frame_seq.store(0);
        _update_rate.store(0.0f);
        _paused.store(false);
        _need_repace.store(true);
        {
            // A seek posted for the source being replaced has nowhere to land.
            std::scoped_lock lk{ _wake_cv_mtx };
            _pending_seek_req.reset();
        }

        {
            std::scoped_lock lk{ _source_mtx };
            // Non-null only for a recording: seeking is what `record_player_source` adds.
            _player = dynamic_cast<record_player_source*>(source.get());
            _source = std::move(source);
        }

        _notify_sensor_stream_reset();
        _start_thread();
    }

    void sensor_frame_provider::close()
    {
        _stop_thread();

        bool had_source = false;
        {
            std::scoped_lock lk{ _source_mtx };
            had_source = static_cast<bool>(_source);
            if (_source) { _source->close(); }
            _source.reset();
            _player = nullptr;
        }

        if (had_source)
        {
            _notify_sensor_stream_end();
        }
    }

    void sensor_frame_provider::_start_thread()
    {
        _running.store(true);
        _thread = std::thread{ &sensor_frame_provider::_polling_thread_proc, this };
    }

    void sensor_frame_provider::_stop_thread()
    {
        { std::scoped_lock lk{ _wake_cv_mtx }; _running.store(false); }
        _wake_cv.notify_all(); // out of whichever sleep it is in, rather than waiting the sleep out
        if (_thread.joinable())
        {
            _thread.join();
        }
    }

    void sensor_frame_provider::_polling_thread_proc()
    {
        // Carries out a posted seek, if one is waiting. True when the position moved.
        const auto apply_pending_seek = [this] {
            std::optional<seek_request_t> request;
            {
                std::scoped_lock lk{ _wake_cv_mtx };
                request = std::exchange(_pending_seek_req, std::nullopt);
            }
            if (!request.has_value()) { return false; }

            std::scoped_lock lk{ _source_mtx };
            if (!_player) { return false; } // a live camera has nowhere to seek to

            switch (request->kind) {
            case seek_request_t::kind_t::begin:    _player->seek_begin(); break;
            case seek_request_t::kind_t::end:      _player->seek_end(); break;
            case seek_request_t::kind_t::timeline: _player->seek_timestamp(request->at); break;
            }
            return true;
        };

        // Sleeps until `until`, waking early on a stop, a seek, or playback moving away from
        // `paused_now`. Each is written under `_wake_cv_mtx`, so none of them is left to time out.
        const auto sleep_until = [this](
            const std::chrono::steady_clock::time_point until,
            const bool paused_now)
        {
            std::unique_lock lk{ _wake_cv_mtx };
            _wake_cv.wait_until(lk, until, [&] {
                return !_running.load() || _pending_seek_req.has_value() || _paused.load() != paused_now;
            });
        };

        // Playback pacing anchor (recording playback only)
        bool anchor_set = false;
        std::chrono::steady_clock::time_point anchor_wall{};
        timestamp_t anchor_ts{};

        // Wall-clock reference for the fps EMA.
        bool have_last_wall = false;
        std::chrono::steady_clock::time_point last_wall{};

        // The stream has been sent back to its start and has yet to yield a frame.
        bool restarted = false;

        while (_running.load())
        {
            // Only this thread moves the source, so the fetch below can only return the position
            // seeked to here. The jump is announced ahead of that frame.
            const bool seeked = apply_pending_seek();
            if (seeked)
            {
                _need_repace.store(true);
                this->_notify_sensor_stream_reset();
            }

            // Paused with nothing asking for a frame. A seek does ask, which is what shows where
            // it landed; one frame later the loop idles here again.
            if (_paused.load() && !seeked)
            {
                _need_repace.store(true);
                sleep_until(std::chrono::steady_clock::now() + kIdlePollSleep, true);
                continue;
            }

            std::optional<sensor_frameset> fs;
            {
                std::scoped_lock lk{ _source_mtx };
                if (!_source) { break; }
                fs = _source->fetch_next_sensor_frameset();
            }

            if (!_running.load()) { break; }

            if (!fs.has_value())
            {
                if (!_player)
                {
                    spdlog::warn("provider: no frame from device (timeout), retrying");
                    continue;
                }

                // Going around is a seek like any other, so the next pass carries it out. A restart
                // that yields nothing says the recording holds nothing, so it ends instead.
                if (_auto_repeat.load() && !restarted)
                {
                    restarted = true;
                    spdlog::debug("provider: recording reached its end, starting over");
                    this->_post_seek_request({ .kind = seek_request_t::kind_t::begin });
                    continue;
                }

                spdlog::info("provider: end of recording stream");
                this->_notify_sensor_stream_end();
                this->pause(); // idle until close / seek / play
                continue;
            }

            const std::shared_ptr<sensor_frame>& new_frame = fs->color_frame();
            if (!new_frame)
            {
                spdlog::warn("provider: capture has no color image, dropping");
                continue;
            }

            restarted = false; // the stream is delivering, so the next end is one to go around
            _frame_seq.fetch_add(1);

            // EMA fps (wall-clock based)
            const auto now = std::chrono::steady_clock::now();
            if (have_last_wall)
            {
                const double dt_sec = std::chrono::duration<double>{ now - last_wall }.count();
                if (dt_sec > 1e-9)
                {
                    const float inst = static_cast<float>(1.0 / dt_sec);
                    const float prev = _update_rate.load();
                    _update_rate.store(prev <= 0.0f ? inst : (kFpsEmaAlpha * inst + (1.0f - kFpsEmaAlpha) * prev));
                }
            }
            last_wall = now;
            have_last_wall = true;

            this->_notify_sensor_frame_update(new_frame);

            // Pace playback to real-time * speed from the timestamps the frames carry.
            if (_player)
            {
                const float speed = (_speed.load() > 0.0f) ? _speed.load() : 1.0f;

                if (!anchor_set || _need_repace.exchange(false))
                {
                    anchor_set = true;
                    anchor_wall = std::chrono::steady_clock::now();
                    anchor_ts = new_frame->timestamp();
                }
                else
                {
                    const double rel_ns = static_cast<double>((new_frame->timestamp() - anchor_ts).count()) / speed;
                    const auto target = anchor_wall + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double, std::nano>{ rel_ns });
                    sleep_until(target, false);
                }
            }
        }
    }

    std::vector<std::shared_ptr<sensor_frame_observer>> sensor_frame_provider::_snapshot_observers() const
    {
        std::scoped_lock lk{ _observers_mtx };
        return _observers; // snapshot copy so callbacks run without holding the lock
    }

    void sensor_frame_provider::_notify_sensor_frame_update(const std::shared_ptr<sensor_frame>& frame)
    {
        for (const auto& obs : _snapshot_observers()) { obs->on_sensor_frame_update(frame); }
    }

    void sensor_frame_provider::_notify_sensor_stream_reset()
    {
        for (const auto& obs : _snapshot_observers()) { obs->on_sensor_stream_reset(); }
    }

    void sensor_frame_provider::_notify_sensor_stream_end()
    {
        for (const auto& obs : _snapshot_observers()) { obs->on_sensor_stream_end(); }
    }

    void sensor_frame_provider::play()
    {
        _need_repace.store(true);
        { std::scoped_lock lk{ _wake_cv_mtx }; _paused.store(false); }
        _wake_cv.notify_all();
    }

    void sensor_frame_provider::pause()
    {
        { std::scoped_lock lk{ _wake_cv_mtx }; _paused.store(true); }
        _wake_cv.notify_all();
    }

    void sensor_frame_provider::seek_recording_to_begin()
    {
        this->_post_seek_request({ .kind = seek_request_t::kind_t::begin });
    }

    void sensor_frame_provider::seek_recording_to_end()
    {
        this->_post_seek_request({ .kind = seek_request_t::kind_t::end });
    }

    void sensor_frame_provider::seek_recording_timeline(const timestamp_t timestamp)
    {
        this->_post_seek_request({ .kind = seek_request_t::kind_t::timeline, .at = timestamp });
    }

    void sensor_frame_provider::_post_seek_request(const seek_request_t& request)
    {
        {
            // Overwritten rather than queued: a drag posts one of these per mouse move and only
            // the position it ends on is worth reading frames from.
            std::scoped_lock lk{ _wake_cv_mtx };
            _pending_seek_req = request;
        }
        _wake_cv.notify_all();
    }

    std::chrono::nanoseconds sensor_frame_provider::get_recording_length() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _player ? _player->get_recording_length() : std::chrono::nanoseconds{ 0 };
    }

    timestamp_t sensor_frame_provider::get_first_record_timestamp() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _player ? _player->get_first_record_timestamp() : timestamp_t{};
    }

    timestamp_t sensor_frame_provider::get_last_record_timestamp() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _player ? _player->get_last_record_timestamp() : timestamp_t{};
    }

    void sensor_frame_provider::set_update_speed(float factor)
    {
        _speed.store(factor > 0.0f ? factor : 1.0f);
        _need_repace.store(true);
    }

} // namespace hw
