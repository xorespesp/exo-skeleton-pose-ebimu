#include "exo_pose_pipeline.hh"

#include "hw/sensor_frame_observer.hh"
#include "pose/tag_detector.hh"

#include <spdlog/spdlog.h>

#include <atomic>
#include <format>
#include <mutex>
#include <string>
#include <utility>

namespace net
{
    namespace
    {
        // How often poll() summarizes throughput while a source streams.
        constexpr auto kStatsInterval = std::chrono::seconds{ 5 };

        // Tag ids beyond this are still detected and estimated; they just fall out of the
        // appeared/disappeared bookkeeping, which tracks visibility in a 64-bit mask.
        constexpr int kMaxLoggedTagId = 63;

        // "3(r_ankle)": a log line names the joint a tag stands for, not just the raw id.
        std::string tag_label(int tag_id)
        {
            const auto j = pose::tag_to_joint(tag_id);
            return j.has_value()
                ? std::format("{}({})", tag_id, pose::joint_info(j.value()).name)
                : std::format("{}(unmapped)", tag_id);
        }

        std::string tag_list(uint64_t mask)
        {
            std::string s;
            for (int t = 0; t <= kMaxLoggedTagId; ++t)
            {
                if ((mask & (1ull << t)) == 0) { continue; }
                if (!s.empty()) { s += ", "; }
                s += tag_label(t);
            }
            return s.empty() ? std::string{ "none" } : s;
        }
    } // namespace

    // --- observer (worker thread) ------------------------------------------------
    // Worker-thread tag detection; latches detections & annotated frame for the loop thread to pull.
    class pose_frame_observer final : public hw::sensor_frame_observer
    {
    public:
        pose_frame_observer(const hw::sensor_frame_provider& provider, double tag_size_m, bool annotate)
            : _provider{ provider }, _tag_size_m{ tag_size_m }, _annotate{ annotate }
        { }

        // Returns false if nothing new since `last_seq`, else copies out + advances it.
        bool try_get(
            std::vector<pose::tag_detection_t>& out_dets,
            std::chrono::microseconds& out_timestamp,
            uint64_t& last_seq)
        {
            std::scoped_lock lk{ _mtx };
            if (_seq == last_seq) { return false; }
            out_dets = _detections;
            out_timestamp = _timestamp;
            last_seq = _seq;
            return true;
        }

        // Like try_get, plus the annotated frame image.
        // Empty image if annotation is off.
        bool try_get_frame(
            cv::Mat& out_img,
            std::vector<pose::tag_detection_t>& out_dets,
            std::chrono::microseconds& out_timestamp,
            uint64_t& last_seq)
        {
            std::scoped_lock lk{ _mtx };
            if (_seq == last_seq) { return false; }
            out_img = _annotated;
            out_dets = _detections;
            out_timestamp = _timestamp;
            last_seq = _seq;
            return true;
        }

        // True once per stream-end signal (consumes the latched flag).
        bool consume_stream_ended_signal() noexcept {
            return _stream_ended.exchange(false);
        }

    public:
        void on_sensor_frame_update(const std::shared_ptr<hw::sensor_frame>& frame) override
        {
            if (!_detector.has_value()) // built once intrinsics are known (after open)
            {
                pose::tag_detector::options_t opt;
                opt.intrinsics = _provider.get_calibration().color_intr;
                opt.tag_size_m = _tag_size_m;
                _detector.emplace(opt);
                spdlog::debug("pipeline: tag detector built on first frame (tag size {:.3f} m, annotate {})",
                    _tag_size_m, _annotate);
            }

            cv::Mat annotated;
            std::vector<pose::tag_detection_t> detections;
            if (_annotate) {
                annotated = frame->color_image.clone();
                detections = _detector.value().detect(annotated);
                pose::draw_tag_detections(annotated, detections);
            } else {
                detections = _detector.value().detect(frame->color_image);
            }

            std::scoped_lock lk{ _mtx };
            _annotated = std::move(annotated);
            _detections = std::move(detections);
            _timestamp = frame->timestamp;
            ++_seq;
        }

        void on_sensor_stream_reset() override {}
        void on_sensor_stream_end() override {
            spdlog::debug("pipeline: worker signalled end of stream");
            _stream_ended = true;
        }

    private:
        const hw::sensor_frame_provider& _provider;
        double _tag_size_m{};
        bool _annotate{ false }; // keep an annotated frame copy for a monitor GUI
        std::optional<pose::tag_detector> _detector; // built once intrinsics are known (after open)
        std::mutex _mtx;
        cv::Mat _annotated; // annotated frame
        std::vector<pose::tag_detection_t> _detections;
        std::chrono::microseconds _timestamp{ 0 }; // device timestamp of the latched frame
        uint64_t _seq{ 0 };
        std::atomic<bool> _stream_ended{ false }; // set by the worker thread on stream end
    };

    // --- exo_pose_pipeline -------------------------------------------------------
    exo_pose_pipeline::exo_pose_pipeline(double default_tag_size_m, bool annotate_frames)
        : _default_tag_size_m{ default_tag_size_m }, _annotate_frames{ annotate_frames }
    { }

    exo_pose_pipeline::~exo_pose_pipeline() = default;

    bool exo_pose_pipeline::open_source(
        const app::source_address& source_addr,
        double tag_size_m,
        std::optional<int32_t> exposure_us,
        std::optional<int32_t> gain)
    {
        _status_changed = true; // opening a source changes the reported status (even on failure)

        this->stop_recording();

        auto new_provider = std::make_shared<hw::sensor_frame_provider>();
        auto new_observer = std::make_shared<pose_frame_observer>(*new_provider, tag_size_m, _annotate_frames);
        new_provider->add_observer(new_observer);

        const char* kind = source_addr.is_device() ? "device" : "recording";
        spdlog::info("pipeline: opening {} '{}' (tag size {:.3f} m, exposure {}, gain {})",
            kind, source_addr.to_string(), tag_size_m,
            exposure_us.has_value() ? std::format("{} us", exposure_us.value()) : "auto",
            gain.has_value() ? std::format("{}", gain.value()) : "auto");

        if (_provider) {
            spdlog::info("pipeline: replacing the open source '{}'", _provider->get_source_name());
        }

        const bool ok = source_addr.is_device()
            ? new_provider->open_device(source_addr.device_index(), exposure_us, gain)
            : new_provider->open_recording(source_addr.recording_path());

        if (!ok) {
            spdlog::error("pipeline: failed to open {} '{}'", kind, source_addr.to_string());
            return false;
        }

        _provider = std::move(new_provider); // old provider closes/joins here
        _observer = std::move(new_observer);
        _is_recording = source_addr.is_recording();
        _exposure_us = exposure_us;
        _gain = gain;
        _last_seq = 0;
        _estimator.clear_rest_pose(); // a new source invalidates the captured rest reference
        this->_reset_frame_log_state();

        const auto res = _provider->get_color_camera_resolution();
        spdlog::info("pipeline: {} '{}' opened ({}x{} color); rest pose cleared, awaiting first frame",
            kind, _provider->get_source_name(), res.x(), res.y());
        return true;
    }

    bool exo_pose_pipeline::open_device(uint32_t index, std::optional<int32_t> exposure_us, std::optional<int32_t> gain)
    {
        return this->open_source(app::source_address::device(index), _default_tag_size_m, exposure_us, gain);
    }

    bool exo_pose_pipeline::open_recording(const std::string& path)
    {
        return this->open_source(app::source_address::recording(path), _default_tag_size_m, std::nullopt, std::nullopt);
    }

    void exo_pose_pipeline::close_source()
    {
        if (!_provider) { return; } // nothing open; keep the status flag and the log quiet

        _status_changed = true;
        spdlog::info("pipeline: closing source '{}' after {} frames", _provider->get_source_name(), _provider->get_current_frame_id());

        this->stop_recording();

        _provider.reset(); // stops/joins the worker thread
        _observer.reset();
        _is_recording = false;
        _exposure_us.reset();
        _gain.reset();
        _last_seq = 0;
        this->_reset_frame_log_state();
    }

    bool exo_pose_pipeline::is_source_open() const { return static_cast<bool>(_provider); }
    bool exo_pose_pipeline::is_source_recording() const { return _is_recording; }

    bool exo_pose_pipeline::start_recording(
        const std::filesystem::path& path,
        const io::recording_options& options)
    {
        _status_changed = true;

        if (_is_recording) {
            spdlog::error("pipeline: cannot record a playback source");
            return false;
        }

        if (!_provider) {
            spdlog::error("pipeline: cannot record without an open source");
            return false;
        }
        if (_recorder) {
            spdlog::warn("pipeline: already recording to '{}'", _recorder->path().string());
            return false;
        }

        const io::camera_stream_info camera{
            .id = "color0", // first color stream
            .calibration = _provider->get_calibration(),
            .exposure_us = _exposure_us,
            .gain = _gain,
            .source_name = _provider->get_source_name(),
        };

        // Start the recorder before it is subscribed, 
        // so the first frame it sees is one it can already write.
        auto recorder = std::make_shared<io::frame_recorder>(options);
        if (!recorder->start(path, camera)) { return false; }

        _provider->add_observer(recorder);
        _recorder = std::move(recorder);

        spdlog::info("pipeline: recording '{}' to '{}'", _provider->get_source_name(), path.string());
        return true;
    }

    void exo_pose_pipeline::stop_recording()
    {
        if (!_recorder) { return; }

        _status_changed = true;

        if (_provider) { _provider->remove_observer(_recorder); }
        _recorder->stop();

        const io::recording_stats stats = _recorder->stats();
        if (stats.frames_dropped > 0) {
            spdlog::warn("pipeline: {} frame(s) dropped while recording; the disk or the encoder could not keep up",
                stats.frames_dropped);
        }
        spdlog::info("pipeline: recording stopped ({} frames over {:.1f} s)",
            stats.frames_written, std::chrono::duration<double>{ stats.duration }.count());

        _recorder.reset();
    }

    bool exo_pose_pipeline::is_recording() const
    {
        return _recorder && _recorder->is_started();
    }

    io::recording_stats exo_pose_pipeline::recording_stats() const
    {
        return _recorder ? _recorder->stats() : io::recording_stats{};
    }

    std::filesystem::path exo_pose_pipeline::recording_path() const
    {
        return _recorder ? _recorder->path() : std::filesystem::path{};
    }

    bool exo_pose_pipeline::calibrate_rest_pose()
    {
        _status_changed = true;

        // Which joints are contributing a reference is the first thing to know when a
        // calibration comes out wrong, so name them rather than just counting.
        std::string joints;
        for (const auto& info : pose::kJointsInfo)
        {
            if (!_estimator.get_joint_state(info.id).local_rot.has_value()) { continue; }
            if (!joints.empty()) { joints += ", "; }
            joints += info.name;
        }

        const bool ok = _estimator.calibrate_rest_pose();
        if (!ok) {
            spdlog::warn("pipeline: rest pose calibration failed; no joint had a computable local rotation "
                         "(is the source streaming and are the tags visible?)");
            return false;
        }

        spdlog::info("pipeline: rest pose calibrated from [{}]", joints);
        return true;
    }

    void exo_pose_pipeline::clear_rest_pose()
    {
        _status_changed = true;
        spdlog::info("pipeline: rest pose cleared");
        _estimator.clear_rest_pose();
    }

    pose::exo_pose_estimator& exo_pose_pipeline::estimator() { return _estimator; }
    const pose::exo_pose_estimator& exo_pose_pipeline::estimator() const { return _estimator; }

    exo_pose_pipeline::poll_result exo_pose_pipeline::poll()
    {
        poll_result r{};

        // Detections: pull the newest latched frame and recompute joint states.
        if (_observer && _observer->try_get(_detections, _last_timestamp, _last_seq))
        {
            _estimator.update(_detections, _last_timestamp);
            r.new_pose = true;

            spdlog::trace("pipeline: frame #{} (t={:.3f} s) with {} tag(s)",
                this->current_frame_id(),
                std::chrono::duration<double>{ _last_timestamp }.count(),
                _detections.size());

            this->_log_frame_diff();
            this->_log_periodic_stats();
        }

        // Stream end: consume the one-shot signal the worker thread raises at end of stream.
        r.stream_ended = _observer && _observer->consume_stream_ended_signal();
        if (r.stream_ended)
        {
            // A recording hitting EOF is expected; a live device going quiet is a loss.
            if (_is_recording) { spdlog::info("pipeline: recording '{}' reached the end of its stream", this->source_name()); }
            else { spdlog::warn("pipeline: device '{}' stopped streaming", this->source_name()); }
        }

        // Status: consume the flag set by the last source/rest command.
        r.status_changed = std::exchange(_status_changed, false);
        if (r.status_changed) { 
            spdlog::trace("pipeline: status changed (source open: {}, rest pose: {})",
                this->is_source_open(), _estimator.has_rest_pose());
        }
        return r;
    }

    // Tags appearing/disappearing and joints gaining/losing their local rotation are the two
    // things that explain a stalled or jumpy skeleton, and both are edges: log the transition,
    // not the state, so a steady stream stays silent.
    void exo_pose_pipeline::_log_frame_diff()
    {
        uint64_t tag_mask = 0;
        for (const auto& det : _detections)
        {
            if (det.id < 0 || det.id > kMaxLoggedTagId) { continue; }
            tag_mask |= (1ull << det.id);
        }

        if (tag_mask != _seen_tag_mask)
        {
            const uint64_t appeared = tag_mask & ~_seen_tag_mask;
            const uint64_t lost = _seen_tag_mask & ~tag_mask;
            if (appeared) { spdlog::debug("pipeline: tag(s) detected: {}", tag_list(appeared)); }
            if (lost) { spdlog::debug("pipeline: tag(s) lost: {}", tag_list(lost)); }
            _seen_tag_mask = tag_mask;
        }

        // A tag can be visible while its joint still has no local rotation (the parent's tag is
        // missing), so joint tracking is reported on its own rather than inferred from the tags.
        for (const auto& info : pose::kJointsInfo)
        {
            const bool tracked = _estimator.get_joint_state(info.id).local_rot.has_value();
            bool& was_tracked = _joint_tracked[static_cast<size_t>(info.id)];
            if (tracked == was_tracked) { continue; }

            if (tracked) { spdlog::debug("pipeline: joint '{}' tracking acquired", info.name); }
            else { spdlog::debug("pipeline: joint '{}' tracking lost", info.name); }
            was_tracked = tracked;
        }
    }

    // Periodic throughput line: the cheap way to see the pipeline is alive and keeping up
    // without a per-frame log. Detection rate matters as much as fps, since a stream at full
    // fps with no tags looks identical to a healthy one from the outside.
    void exo_pose_pipeline::_log_periodic_stats()
    {
        ++_stats_frames;
        _stats_detections += static_cast<uint32_t>(_detections.size());

        const auto now = std::chrono::steady_clock::now();
        if (_stats_since.time_since_epoch().count() == 0) { _stats_since = now; return; }

        const auto elapsed = now - _stats_since;
        if (elapsed < kStatsInterval) { return; }

        const double sec = std::chrono::duration<double>{ elapsed }.count();
        size_t tracked = 0;
        for (const auto& info : pose::kJointsInfo)
        {
            if (_estimator.get_joint_state(info.id).local_rot.has_value()) { ++tracked; }
        }

        spdlog::debug("pipeline: {} frames in {:.1f} s ({:.1f} fps polled, source at {:.1f} fps), "
                      "{:.1f} tag(s)/frame, {}/{} joint(s) tracked, rest pose {}",
            _stats_frames, sec, _stats_frames / sec, this->source_fps(),
            _stats_frames > 0 ? static_cast<double>(_stats_detections) / _stats_frames : 0.0,
            tracked, pose::kNumJoints,
            _estimator.has_rest_pose() ? "captured" : "not captured");

        _stats_since = now;
        _stats_frames = 0;
        _stats_detections = 0;
    }

    void exo_pose_pipeline::_reset_frame_log_state()
    {
        _seen_tag_mask = 0;
        _joint_tracked.fill(false);
        _stats_since = {};
        _stats_frames = 0;
        _stats_detections = 0;
    }

    bool exo_pose_pipeline::try_get_annotated_frame(
        cv::Mat& out_img,
        std::vector<pose::tag_detection_t>& out_dets,
        std::chrono::microseconds& out_ts,
        uint64_t& last_seq)
    {
        return _observer && _observer->try_get_frame(out_img, out_dets, out_ts, last_seq);
    }

    std::string exo_pose_pipeline::source_name() const
    {
        return _provider ? _provider->get_source_name() : std::string{};
    }

    Eigen::Vector2i exo_pose_pipeline::source_resolution() const
    {
        return _provider ? _provider->get_color_camera_resolution() : Eigen::Vector2i::Zero();
    }

    float exo_pose_pipeline::source_fps() const
    {
        return _provider ? _provider->get_current_update_rate() : 0.0f;
    }

    uint32_t exo_pose_pipeline::current_frame_id() const
    {
        return _provider ? _provider->get_current_frame_id() : 0;
    }

    bool exo_pose_pipeline::is_source_paused() const
    {
        return _provider && _provider->is_paused();
    }

    void exo_pose_pipeline::set_source_paused(bool paused)
    {
        if (!_provider) { return; }
        spdlog::info("pipeline: source {}", paused ? "paused" : "resumed");
        paused ? _provider->pause() : _provider->play();
    }

    void exo_pose_pipeline::seek_to_begin()
    {
        if (!_provider) { return; }
        spdlog::info("pipeline: seek to the beginning of the recording");
        _provider->seek_recording_to_begin();
    }

    void exo_pose_pipeline::seek_to_end()
    {
        if (!_provider) { return; }
        spdlog::info("pipeline: seek to the end of the recording");
        _provider->seek_recording_to_end();
    }

} // namespace net
