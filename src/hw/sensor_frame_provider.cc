#include "sensor_frame_provider.hh"

#include "backends/k4a_frame_source.hh"

#include "io/mcap_record_player.hh"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <exception>

namespace hw
{
    namespace
    {
        constexpr auto kIdlePollSleep = std::chrono::milliseconds{ 5 };
        constexpr float kFpsEmaAlpha = 0.1f; // weight of the newest sample
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

    bool sensor_frame_provider::open_device(
        const uint32_t device_index,
        const std::optional<int32_t> exposure_us, 
        const std::optional<int32_t> gain) noexcept try
    {
        this->close();

        auto source = std::make_unique<k4a_device_capturer>();
        if (!source->open(device_index, { exposure_us, gain })) { 
            return false;
        }

        _install_source(std::move(source), nullptr, "device #" + std::to_string(device_index));
        spdlog::info("provider opened: device #{}", device_index);
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("provider: failed to open device #{}: {}", device_index, e.what());
        return false;
    }

    bool sensor_frame_provider::open_recording(const std::filesystem::path& recording_file) noexcept try
    {
        this->close();

        auto recording = std::make_unique<io::mcap_record_player>();
        if (!recording->open(recording_file)) { return false; }
        recording->enable_auto_repeat(true);

        record_player_source* player = recording.get();
        _install_source(std::move(recording), player, recording_file.filename().string());
        spdlog::info("provider opened: recording '{}'", recording_file.string());
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("provider: failed to open recording '{}': {}", recording_file.string(), e.what());
        return false;
    }

    void sensor_frame_provider::_install_source(
        std::unique_ptr<sensor_frame_source> source,
        record_player_source* player,
        std::string source_name)
    {
        // Cache metadata once; it stays constant while streaming.
        _calib = source->get_calibration();
        _color_resolution = source->get_color_camera_resolution();
        _color_fov = source->get_color_camera_fov();
        _source_name = std::move(source_name);

        _frame_id.store(0);
        _update_rate.store(0.0f);
        _paused.store(false);
        _need_repace.store(true);

        {
            std::scoped_lock lk{ _source_mtx };
            _source = std::move(source);
            _player = player;
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
        _running.store(false);
        if (_thread.joinable())
        {
            _thread.join();
        }
    }

    void sensor_frame_provider::_polling_thread_proc()
    {
        // Playback pacing anchor (recording playback only)
        bool anchor_set = false;
        std::chrono::steady_clock::time_point anchor_wall{};
        std::chrono::microseconds anchor_ts{ 0 };

        // Wall-clock reference for the fps EMA.
        bool have_last_wall = false;
        std::chrono::steady_clock::time_point last_wall{};

        while (_running.load())
        {
            if (_paused.load())
            {
                _need_repace.store(true);
                std::this_thread::sleep_for(kIdlePollSleep);
                continue;
            }

            std::unique_ptr<sensor_frameset> fs;
            {
                std::scoped_lock lk{ _source_mtx };
                if (!_source) { break; }
                fs = _source->fetch_next_sensor_frameset();
            }

            if (!_running.load()) { break; }

            if (!fs)
            {
                if (_player)
                {
                    // nullptr from a recording means EOF (auto-repeat off).
                    spdlog::info("provider: end of recording stream");
                    _notify_sensor_stream_end();
                    _paused.store(true); // idle until close / seek / play
                }
                else
                {
                    spdlog::warn("provider: no frame from device (timeout), retrying");
                }
                continue;
            }

            if (!fs->has_color_image())
            {
                spdlog::warn("provider: frame has no color image, dropping");
                continue;
            }

            const uint32_t id = _frame_id.fetch_add(1) + 1;
            auto new_frame = std::make_shared<sensor_frame>(id);
            new_frame->timestamp = fs->get_color_timestamp();
            new_frame->color_image = fs->get_color_image().clone(); // deep-copy: the view outlives the frameset

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

            _notify_sensor_frame_update(new_frame);

            // Pace playback to real-time * speed using device timestamps.
            if (_player)
            {
                const float speed = (_speed.load() > 0.0f) ? _speed.load() : 1.0f;

                if (!anchor_set || _need_repace.exchange(false))
                {
                    anchor_set = true;
                    anchor_wall = std::chrono::steady_clock::now();
                    anchor_ts = new_frame->timestamp;
                }
                else
                {
                    const double rel_us = static_cast<double>((new_frame->timestamp - anchor_ts).count()) / speed;
                    const auto target = anchor_wall + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double, std::micro>{ rel_us });
                    std::this_thread::sleep_until(target);
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
        _paused.store(false);
    }

    void sensor_frame_provider::pause()
    {
        _paused.store(true);
    }

    void sensor_frame_provider::seek_recording_to_begin()
    {
        std::scoped_lock lk{ _source_mtx };
        if (_player) { _player->seek_begin(); _need_repace.store(true); }
    }

    void sensor_frame_provider::seek_recording_to_end()
    {
        std::scoped_lock lk{ _source_mtx };
        if (_player) { _player->seek_end(); _need_repace.store(true); }
    }

    void sensor_frame_provider::seek_recording_timeline(std::chrono::microseconds timestamp)
    {
        std::scoped_lock lk{ _source_mtx };
        if (_player) { _player->seek_timestamp(timestamp); _need_repace.store(true); }
    }

    std::chrono::microseconds sensor_frame_provider::get_recording_length() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _player ? _player->get_recording_length() : std::chrono::microseconds{ 0 };
    }

    std::chrono::microseconds sensor_frame_provider::get_first_record_timestamp() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _player ? _player->get_first_record_timestamp() : std::chrono::microseconds{ 0 };
    }

    std::chrono::microseconds sensor_frame_provider::get_last_record_timestamp() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _player ? _player->get_last_record_timestamp() : std::chrono::microseconds{ 0 };
    }

    void sensor_frame_provider::set_update_speed(float factor)
    {
        _speed.store(factor > 0.0f ? factor : 1.0f);
        _need_repace.store(true);
    }

    bool sensor_frame_provider::is_auto_repeat_enabled() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _player ? _player->auto_repeat_enabled() : false;
    }

    void sensor_frame_provider::set_auto_repeat(bool enable)
    {
        std::scoped_lock lk{ _source_mtx };
        if (_player) { _player->enable_auto_repeat(enable); }
    }

} // namespace hw
