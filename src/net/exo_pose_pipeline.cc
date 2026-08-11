#include "exo_pose_pipeline.hh"

#include "hw/sensor_frame_observer.hh"
#include "io/calibration_io.hh"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <atomic>
#include <filesystem>
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

        // The camera controls arrive as integers; 
        // the VZ camera states its exposure and gain in fractional units.
        std::optional<double> to_optional_double(const std::optional<int32_t> value)
        {
            if (!value.has_value()) { return std::nullopt; }
            return static_cast<double>(*value);
        }

    } // namespace

    // --- observer (worker thread) ------------------------------------------------
    // Runs the tracker over each arriving frame and latches the annotated image for a monitor GUI.
    // What detection means and what it produces are the tracker's business, which the tracker
    // publishes to the loop thread itself; this class holds no knowledge of any marker technology.
    class pose_frame_observer final : public hw::sensor_frame_observer
    {
    public:
        pose_frame_observer(
            std::shared_ptr<pose::marker_tracker_base> tracker, 
            bool annotate)
            : _tracker{ std::move(tracker) }
            , _annotate{ annotate }
        { }

        // Newest annotated frame and the source frame it was drawn over; false if nothing new
        // since `last_seq`. The annotated image is empty if annotation is off.
        //
        // Both come out of one turn of the lock, so they describe the same capture. That is what
        // lets a caller read original pixels at a point it picked off the drawn one. Neither is
        // copied: `cv::Mat` shares its buffer, and the next frame assigns a new one rather than
        // writing over these.
        bool try_get_frame(
            cv::Mat& out_img, 
            cv::Mat& out_source, 
            uint64_t& last_seq)
        {
            std::scoped_lock lk{ _mtx };
            if (_seq == last_seq) { return false; }
            out_img = _annotated;
            out_source = _source;
            last_seq = _seq;
            return true;
        }

        // True once per stream-end signal (consumes the latched flag).
        bool consume_stream_ended_signal() noexcept {
            return _stream_ended.exchange(false);
        }

        // True once per stream-jump signal (consumes the latched flag).
        bool consume_stream_reset_signal() noexcept {
            return _stream_reset.exchange(false);
        }

        // True once per geometry-change signal (consumes the latched flag).
        bool consume_frame_geometry_signal() noexcept {
            return _frame_geometry_changed.exchange(false);
        }

    public:
        void on_sensor_frame_update(const std::shared_ptr<hw::sensor_frame>& frame) override
        {
            // The canvas is technology-independent, so it is prepared here and handed over to be
            // drawn on. Detection publishes itself; nothing comes back to be latched.
            cv::Mat annotated;
            if (_annotate) {
                if (frame->format() == hw::frame_format_t::gray8) {
                    cv::cvtColor(frame->image(), annotated, cv::COLOR_GRAY2BGR);
                } else {
                    annotated = frame->image().clone();
                }
            }

            _tracker->process_frame(
                frame->image(), 
                frame->format(), 
                frame->timestamp(),
                _annotate ? &annotated : nullptr
            );

            std::scoped_lock lk{ _mtx };
            _annotated = std::move(annotated);
            _source = frame->image();
            ++_seq;
        }

        void on_sensor_stream_reset() override {
            // Called from the worker thread; what the jump invalidates is dropped on the estimator thread,
            // so this only raises the flag that poll() acts on.
            _stream_reset = true;
        }

        void on_sensor_frame_geometry_changed() override {
            _frame_geometry_changed = true;
        }

        void on_sensor_stream_end() override {
            spdlog::debug("pipeline: worker signalled end of stream");
            _stream_ended = true;
        }

    private:
        const std::shared_ptr<pose::marker_tracker_base> _tracker;
        const bool _annotate; // keep an annotated frame copy for a monitor GUI

        std::mutex _mtx;
        cv::Mat _annotated;
        cv::Mat _source; // the same capture undrawn, for anything reading original pixels
        uint64_t _seq{ 0 };
        std::atomic<bool> _stream_ended{ false }; // set by the worker thread on stream end
        std::atomic<bool> _stream_reset{ false }; // set by the worker thread when the position jumps
        std::atomic<bool> _frame_geometry_changed{ false }; // ... and when the ROI changes
    };

    // --- exo_pose_pipeline -------------------------------------------------------
    exo_pose_pipeline::exo_pose_pipeline(bool annotate_frames)
        : _annotate_frames{ annotate_frames }
    { }

    exo_pose_pipeline::~exo_pose_pipeline() = default;

    void exo_pose_pipeline::_select_estimator(pose::view_plane_t view_plane)
    {
        if (_active && _view_plane == view_plane) { return; } // unchanged: the estimator stands

        _frontal.reset();
        _sagittal.reset();

        switch (view_plane)
        {
        case pose::view_plane_t::sagittal:
            _sagittal.emplace();
            _active = &_sagittal.value();
            break;
        case pose::view_plane_t::frontal:
        default:
            _frontal.emplace();
            _active = &_frontal.value();
            break;
        }
        _view_plane = view_plane;
    }

    bool exo_pose_pipeline::open_source(const app::app_config_t& config)
    {
        _status_changed = true; // opening a source changes the reported status (even on failure)

        // The same answer a config file is held to, asked of whatever assembled this one. A control
        // panel reaches every rule here that an authored profile does.
        if (std::string err; !app::validate_config(config, err)) {
            spdlog::error("pipeline: {}", err);
            return false;
        }

        if (!config.camera.source.has_value()) {
            spdlog::error("pipeline: the config names no source to open");
            return false;
        }

        const app::source_address& source_addr = *config.camera.source;
        const pose::view_plane_t view_plane = config.pose.estimator.view_plane;
        const app::marker_kind_t marker_kind = config.pose.detector.kind;

        const std::optional<int32_t> exposure_us = config.camera.exposure_us;
        const std::optional<int32_t> gain = config.camera.gain;
        const std::optional<hw::roi_t> roi = config.camera.roi;

        this->stop_recording();

        // A recording ignores this and replays the layout it was written with, which is what makes
        // a gray recording opened under a color profile something the tracker has to refuse.
        const auto kCameraFrameFormat = app::marker_frame_format(marker_kind);

        hw::source_config_t source_config;
        if (source_addr.is_k4a_device())
        {
            source_config = hw::k4a_device_config_t{
                .device_index = source_addr.k4a_device_index(),
                .exposure_us = exposure_us,
                .gain = gain,
                .frame_format = kCameraFrameFormat,
                .roi = roi,
            };
        }
        else if (source_addr.is_vz_device())
        {
            hw::vz_device_config_t vz{
                .device_index = source_addr.vz_device_index(),
                .exposure_us = to_optional_double(exposure_us),
                .gain = to_optional_double(gain),
                .frame_format = kCameraFrameFormat,
                .roi = roi,
            };

            if (!config.camera.intrinsics_file.empty())
            {
                const std::filesystem::path path{ config.camera.intrinsics_file };
                hw::intrinsic_t intr{};
                hw::distortion_t dist{};
                if (std::string err; io::load_camera_calibration(path, intr, dist, err)) {
                    vz.intrinsic = intr;
                    vz.distortion = dist;
                    spdlog::info("pipeline: read intrinsics from '{}' ({}x{})"
                        , path.string()
                        , intr.calib_resolution.x(), intr.calib_resolution.y()
                    );
                } else {
                    // frontal estimator will not solve tag poses, but the sagittal estimator still works off 2D tag centers
                    spdlog::warn("pipeline: '{}': {}; opening without intrinsics", path.string(), err);
                }
            }

            source_config = std::move(vz);
        }
        else
        {
            source_config = hw::recording_config_t{
                .file = source_addr.recording_path(),
                .roi = roi,
            };
        }

        const char* kind = source_addr.is_recording() ? "recording" : "camera";
        spdlog::info("pipeline: opening {} '{}' ({} view, {} markers, {} frames, exposure {}, gain {})"
            , kind
            , source_addr.to_string()
            , pose::view_plane_name(view_plane)
            , app::marker_kind_name(marker_kind)
            , hw::frame_format_to_str(kCameraFrameFormat)
            , exposure_us.has_value() ? std::format("{} us", exposure_us.value()) : "auto"
            , gain.has_value() ? std::format("{}", gain.value()) : "auto"
        );

        if (_provider) {
            spdlog::info("pipeline: replacing the open source '{}'", _provider->get_source_name());
        }

        auto new_provider = std::make_shared<hw::sensor_frame_provider>();
        const bool ok = new_provider->open(source_config);
        if (!ok) {
            spdlog::error("pipeline: failed to open {} '{}'", kind, source_addr.to_string());
            return false;
        }

        // Intrinsics are what turn the per-tag pose solve on, and with it the 3D measurements a
        // frontal estimator consumes. They are read now because only an opened source reports them,
        // and only a frontal run has any use for them: the 2D path works off marker centers.
        //
        // A source that carries none reports them zeroed, and a zero focal length solves to nonsense
        // instead of failing. Withholding them is the honest answer, since the joints then read as
        // untracked rather than as plausible wrong positions.
        std::optional<hw::intrinsic_t> intrinsics;
        if (view_plane == pose::view_plane_t::frontal)
        {
            if (const hw::intrinsic_t intr = new_provider->get_calibration().intrinsic;
                intr.fx > 0.0f && intr.fy > 0.0f) {
                intrinsics = intr;
            } else {
                spdlog::warn("pipeline: '{}' reports no intrinsics, so marker poses cannot be "
                             "solved and the frontal estimator will track nothing",
                    new_provider->get_source_name());
            }
        }

        // The one place that names a concrete tracker. Unlike the estimator, it is rebuilt on every
        // open: it is built around the source it will read, so carrying one over would leave it
        // describing the camera before.
        //
        // An open installs the config, so what is in effect right afterwards is what the file says.
        // Edits made from the control panel are scratch until they are saved back.
        if (marker_kind == app::marker_kind_t::color_marker)
        {
            // The colour and the blob filters were measured together on site and sit together in
            // the profile. Absent, the detector runs on defaults that carry no colour and finds
            // nothing, which it says once when it is built.
            const std::optional<app::color_marker_calibration_t>& calibration =
                config.pose.detector.color_marker.calibration;

            // The blob gates are counted in pixels, so they only mean what they meant if a marker
            // still covers as many of them. A different frame size moves every one of them at once.
            if (const Eigen::Vector2i frame_resolution = new_provider->get_frame_resolution();
                calibration.has_value() && calibration->frame_resolution != frame_resolution)
            {
                spdlog::warn("pipeline: the color was measured on {}x{} frames but this source "
                             "delivers {}x{}; the blob size gates were sized for the other one",
                    calibration->frame_resolution.x(), calibration->frame_resolution.y(),
                    frame_resolution.x(), frame_resolution.y());
            }

            _tracker = std::make_shared<pose::color_marker_tracker>(
                calibration.has_value() ? calibration->detector : pose::color_marker_detector::options_t{},
                config.pose.detector.color_marker.assigner
            );
        }
        else
        {
            _tracker = std::make_shared<pose::apriltag_tracker>(
                config.pose.detector.apriltag.detector,
                config.pose.detector.apriltag.tag_size_m,
                std::move(intrinsics)
            );
        }

        auto new_observer = std::make_shared<pose_frame_observer>(_tracker, _annotate_frames);
        new_provider->add_observer(new_observer);

        _provider = std::move(new_provider); // old provider closes/joins here
        _observer = std::move(new_observer);
        _is_recording = source_addr.is_recording();
        _exposure_us = exposure_us;
        _gain = gain;
        _has_pose = false; // whatever the previous source produced does not describe this one
        this->_select_estimator(view_plane); // swaps the estimator only when the viewing plane changed

        // `_select_estimator` leaves an estimator of the same plane standing, so the config's
        // options are assigned out here, where every open reaches them.
        if (_frontal) {
            _frontal->options() = config.pose.estimator.frontal;
        }

        if (_sagittal) {
            _sagittal->options() = config.pose.estimator.sagittal;
        }

        _active->clear_rest_pose(); // a new source invalidates the captured rest reference
        _active->reset_tracking();  // and its position filters/held points must not carry over
        _frame_log.reset();

        const auto res = _provider->get_frame_resolution();
        spdlog::info("pipeline: {} '{}' opened ({}x{} color, {} estimator); rest pose cleared, awaiting first frame",
            kind, _provider->get_source_name(), res.x(), res.y(), pose::view_plane_name(_view_plane));
        return true;
    }

    void exo_pose_pipeline::close_source()
    {
        if (!_provider) { return; } // nothing open; keep the status flag and the log quiet

        _status_changed = true;
        spdlog::info("pipeline: closing source '{}' after {} frames"
            , _provider->get_source_name()
            , _provider->get_current_frame_seq()
        );

        this->stop_recording();

        _provider.reset(); // stops/joins the worker thread
        _observer.reset();
        _is_recording = false;
        _exposure_us.reset();
        _gain.reset();
        _frame_log.reset();
    }

    bool exo_pose_pipeline::is_source_open() const { return static_cast<bool>(_provider); }
    bool exo_pose_pipeline::is_source_recording() const { return _is_recording; }

    bool exo_pose_pipeline::start_recording(
        const std::filesystem::path& path,
        const io::recording_options_t& options)
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

        const io::camera_stream_info_t color_stream_info{
            .stream_name = "color0", // first color stream
            .calibration = _provider->get_calibration(),
            .color_format = _provider->get_frame_format(),
            .source_backend = _provider->get_source_backend(),
            .source_name = _provider->get_source_name(),
            .exposure_us = _exposure_us,
            .gain = _gain,
        };

        // Start the recorder before it is subscribed, 
        // so the first frame it sees is one it can already write.
        auto recorder = std::make_shared<io::frame_recorder>(options);
        if (!recorder->start(path, color_stream_info)) { return false; }

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

        const io::recording_stats_t stats = _recorder->stats();
        if (stats.frames_dropped > 0) {
            spdlog::warn("pipeline: {} frame(s) dropped while recording; the disk or the encoder could not keep up"
                , stats.frames_dropped);
        }
        spdlog::info("pipeline: recording stopped ({} frames over {:.1f} s)"
            , stats.frames_written
            , std::chrono::duration<double>{ stats.duration }.count()
        );

        _recorder.reset();
    }

    bool exo_pose_pipeline::is_recording() const
    {
        return _recorder && _recorder->is_started();
    }

    io::recording_stats_t exo_pose_pipeline::recording_stats() const
    {
        return _recorder ? _recorder->stats() : io::recording_stats_t{};
    }

    std::filesystem::path exo_pose_pipeline::recording_path() const
    {
        return _recorder ? _recorder->path() : std::filesystem::path{};
    }

    bool exo_pose_pipeline::calibrate_rest_pose()
    {
        _status_changed = true;

        if (!_active) {
            spdlog::warn("pipeline: cannot calibrate a rest pose without an open source");
            return false;
        }

        const bool ok = _active->calibrate_rest_pose();
        if (!ok) {
            spdlog::warn("pipeline: rest pose calibration failed; no joint had a computable local rotation "
                         "(is the source streaming and are the tags visible?)");
            return false;
        }

        // Whatever the tracker latches at calibration belongs to this same moment: it is the one
        // point where an operator is watching the annotated frame and vouching for what it shows.
        _tracker->on_rest_pose_captured();

        // Which joints are contributing a reference is the first thing to know when a calibration
        // comes out wrong, so name the ones that actually latched (only freshly detected joints,
        // not held ones) rather than just counting.
        std::string joints;
        for (const auto& def : pose::get_joint_defs())
        {
            if (!_active->get_rest_position(def.joint_id).has_value()) { continue; }
            if (!joints.empty()) { joints += ", "; }
            joints += def.name;
        }

        spdlog::info("pipeline: rest pose calibrated from [{}]", joints);
        return true;
    }

    void exo_pose_pipeline::clear_rest_pose()
    {
        if (!_active) { return; }
        _status_changed = true;
        spdlog::info("pipeline: rest pose cleared");
        _active->clear_rest_pose();
        _tracker->on_rest_pose_cleared(); // captured together, so dropped together
    }

    bool exo_pose_pipeline::has_rest_pose() const
    {
        return _active && _active->has_rest_pose();
    }

    pose::pose_estimator_base* exo_pose_pipeline::estimator() { return _active; }
    const pose::pose_estimator_base* exo_pose_pipeline::estimator() const { return _active; }

    std::optional<pose::frontal_pose_estimator::options_t> exo_pose_pipeline::frontal_options() const
    {
        if (!_frontal) { return std::nullopt; }
        return _frontal->options();
    }

    void exo_pose_pipeline::set_frontal_options(const pose::frontal_pose_estimator::options_t& opt)
    {
        if (_frontal) { _frontal->options() = opt; }
    }

    std::optional<pose::sagittal_pose_estimator::options_t> exo_pose_pipeline::sagittal_options() const
    {
        if (!_sagittal) { return std::nullopt; }
        return _sagittal->options();
    }

    void exo_pose_pipeline::set_sagittal_options(const pose::sagittal_pose_estimator::options_t& opt)
    {
        if (_sagittal) { _sagittal->options() = opt; }
    }

    const pose::sagittal_pose_estimator* exo_pose_pipeline::sagittal_estimator() const
    {
        return _sagittal ? &_sagittal.value() : nullptr;
    }

    exo_pose_pipeline::poll_result_t exo_pose_pipeline::poll()
    {
        poll_result_t r{};

        {
            // Take the newest frame the tracker published, ahead of stepping the estimator on it.
            //
            // NOTE: `update()` is not part of the estimator base; each one takes the input its algorithm
            // needs, so the tracker is asked for the shape the built estimator consumes, and answers
            // false when it has nothing new or cannot produce that shape at all.
            hw::timestamp_t taken_at{};
            std::vector<pose::joint_3d_measurement_t> m3;
            std::vector<pose::joint_2d_measurement_t> m2;
            bool took_3d = false;
            bool took_2d = false;
            if (_tracker)
            {
                took_3d = _frontal && _tracker->try_get_3d_measurements(m3, taken_at);
                took_2d = !took_3d && _sagittal && _tracker->try_get_2d_measurements(m2, taken_at);
            }

            // The stream jumped. Reading this after the take is what makes the two agree: the
            // frame thread raises it before publishing anything of the new position, so a step
            // that took the first such frame is a step that sees it. What it took goes with it.
            if (_observer && _observer->consume_stream_reset_signal())
            {
                spdlog::debug("pipeline: the stream position jumped; dropping what described the last one");
                if (_tracker) { _tracker->on_stream_reset(); }
                if (_active) { _active->reset_tracking(); }
                took_3d = took_2d = false;
            }

            if (_observer && _observer->consume_frame_geometry_signal())
            {
                spdlog::debug("pipeline: the ROI changed; dropping what described the last one");
                if (_active) { _active->on_frame_geometry_changed(); }
            }

            if (took_3d)
            {
                _last_timestamp = taken_at;
                _frontal->update(m3, _last_timestamp);
                r.new_pose = true;
            }
            else if (took_2d)
            {
                _last_timestamp = taken_at;
                _sagittal->update(m2, _last_timestamp);
                r.new_pose = true;
            }
        }

        if (r.new_pose)
        {
            _has_pose = true;

            spdlog::trace("pipeline: frame #{} (t={:%H:%M:%S}) with {} detection(s)"
                , this->current_frame_seq()
                , _last_timestamp
                , _tracker->last_detection_count()
            );

            _frame_log.log_transitions(*_tracker, *_active);
            _frame_log.log_throughput(*_tracker, *_active, this->source_fps(), this->has_rest_pose());
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
                this->is_source_open(), this->has_rest_pose());
        }
        return r;
    }

    // The tracker losing its markers, and joints gaining or losing their local rotation, are what
    // explain a stalled or jumpy skeleton. What a marker technology has to say about its own
    // detections it says itself.
    void exo_pose_pipeline::frame_logger::log_transitions(
        const pose::marker_tracker_base& tracker,
        const pose::pose_estimator_base& estimator)
    {
        // Whether the tracker has identified its markers is what a run whose markers are anonymous
        // hangs on: until it has, no joint is named and every one reads untracked for a reason that
        // is not the detector's.
        if (const bool tracking = tracker.is_tracking();
            tracking != _tracker_was_tracking)
        {
            if (tracking) { spdlog::debug("pipeline: the tracker identified its markers"); }
            else { spdlog::debug("pipeline: the tracker lost its markers and is searching again"); }
            _tracker_was_tracking = tracking;
        }

        // A marker can be visible while its joint still has no local rotation (the parent's is
        // missing), so joint tracking is reported on its own rather than inferred from the markers.
        for (const auto& def : pose::get_joint_defs())
        {
            const bool tracked = estimator.get_joint_state(def.joint_id).position.has_value();
            bool& was_tracked = _joint_tracked[static_cast<size_t>(def.joint_id)];
            if (tracked == was_tracked) { continue; }

            if (tracked) { spdlog::debug("pipeline: joint '{}' tracking acquired", def.name); }
            else { spdlog::debug("pipeline: joint '{}' tracking lost", def.name); }
            was_tracked = tracked;
        }
    }

    // Periodic throughput line: the cheap way to see the pipeline is alive and keeping up
    // without a per-frame log. Detection rate matters as much as fps, since a stream at full
    // fps with no markers looks identical to a healthy one from the outside.
    void exo_pose_pipeline::frame_logger::log_throughput(
        const pose::marker_tracker_base& tracker,
        const pose::pose_estimator_base& estimator,
        const float source_fps,
        const bool has_rest_pose)
    {
        ++_frames;
        _detections += static_cast<uint32_t>(tracker.last_detection_count());

        const auto now = std::chrono::steady_clock::now();
        if (_since.time_since_epoch().count() == 0) { _since = now; return; }

        const auto elapsed = now - _since;
        if (elapsed < kStatsInterval) { return; }

        const double sec = std::chrono::duration<double>{ elapsed }.count();
        size_t tracked = 0;
        for (const auto& def : pose::get_joint_defs())
        {
            if (estimator.get_joint_state(def.joint_id).position.has_value()) { ++tracked; }
        }

        spdlog::debug("pipeline: {} frames in {:.1f} s ({:.1f} fps polled, source at {:.1f} fps), "
                      "{:.1f} detection(s)/frame, {}/{} joint(s) tracked, rest pose {}"
            , _frames, sec, _frames / sec, source_fps
            , _frames > 0 ? static_cast<double>(_detections) / _frames : 0.0
            , tracked, pose::kNumJoints
            , has_rest_pose ? "captured" : "not captured"
        );

        _since = now;
        _frames = 0;
        _detections = 0;
    }

    void exo_pose_pipeline::frame_logger::reset()
    {
        *this = frame_logger{};
    }

    bool exo_pose_pipeline::try_get_annotated_frame(
        cv::Mat& out_img, 
        cv::Mat& out_source, 
        uint64_t& last_seq)
    {
        return _observer && _observer->try_get_frame(out_img, out_source, last_seq);
    }

    hw::source_backend_t exo_pose_pipeline::source_backend() const
    {
        return _provider ? _provider->get_source_backend() : hw::source_backend_t{};
    }

    std::string exo_pose_pipeline::source_name() const
    {
        return _provider ? _provider->get_source_name() : std::string{};
    }

    Eigen::Vector2i exo_pose_pipeline::source_resolution() const
    {
        return _provider ? _provider->get_frame_resolution() : Eigen::Vector2i::Zero();
    }

    Eigen::Vector2i exo_pose_pipeline::source_full_resolution() const
    {
        return _provider ? _provider->get_full_frame_resolution() : Eigen::Vector2i::Zero();
    }

    std::optional<hw::roi_t> exo_pose_pipeline::effective_roi() const
    {
        return _provider ? _provider->get_effective_roi() : std::nullopt;
    }

    void exo_pose_pipeline::set_roi(const std::optional<hw::roi_t>& roi)
    {
        if (!_provider) { return; }

        // A recording declares one calibration up front, frame size included, so the frames that
        // follow have to keep it.
        if (this->is_recording()) {
            spdlog::error("pipeline: cannot move the ROI while a recording is being written");
            return;
        }
        _provider->set_roi(roi);
    }

    float exo_pose_pipeline::source_fps() const
    {
        return _provider ? _provider->get_current_update_rate() : 0.0f;
    }

    std::optional<hw::intrinsic_t> exo_pose_pipeline::intrinsics() const
    {
        // Color intrinsics of the open source; lets a diagnostic dump reproject
        // corners independently of whatever the pipeline computed.
        if (!_provider || !_provider->is_opened()) { return std::nullopt; }
        return _provider->get_calibration().intrinsic;
    }

    uint32_t exo_pose_pipeline::current_frame_seq() const
    {
        return _provider ? _provider->get_current_frame_seq() : 0;
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

    bool exo_pose_pipeline::is_auto_repeat_enabled() const
    {
        return _provider && _provider->is_auto_repeat_enabled();
    }

    void exo_pose_pipeline::set_auto_repeat(bool enable)
    {
        if (!_provider) { return; }
        spdlog::info("pipeline: playback {} at the end of the recording",
            enable ? "starts over" : "stops");
        _provider->set_auto_repeat(enable);
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
