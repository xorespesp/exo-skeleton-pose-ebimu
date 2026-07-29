#pragma once
#include "source_address.hh"

#include "hw/calibration.hh"
#include "hw/sensor_frame_provider.hh"
#include "io/frame_recorder.hh"
#include "pose/exo_pose_estimator.hh"
#include "pose/tag_detector.hh"

#include <opencv2/core.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace net
{
    // forward declaration of the worker-thread tag detection observer
    class pose_frame_observer;

    // The pose pipeline: owns the source, the detection worker, and the joint estimator, and
    // steps them independently of any network transport. A server or GUI holds one and drives
    // it via poll(). Not thread-safe: call from a single thread (the loop/GUI thread). Only the
    // provider's worker crosses threads, and it latches into the observer under a lock.
    class exo_pose_pipeline final
    {
    public:
        exo_pose_pipeline(double default_tag_size_m, bool annotate_frames);
        ~exo_pose_pipeline();

        exo_pose_pipeline(const exo_pose_pipeline&) = delete;
        exo_pose_pipeline& operator=(const exo_pose_pipeline&) = delete;

        // --- source control -----------------------------------------------------------
        // `open_device`/`open_recording` are `open_source` with the default tag size.
        bool open_source(
            const app::source_address& source_addr,
            double tag_size_m,
            std::optional<int32_t> exposure_us,
            std::optional<int32_t> gain
        );
        bool open_device(uint32_t index, std::optional<int32_t> exposure_us, std::optional<int32_t> gain);
        bool open_recording(const std::string& path);
        void close_source();

        bool is_source_open() const;
        bool is_source_recording() const;

        // --- recording ----------------------------------------------------------------
        // Captures the live source's frames to recording file. 
        // (Refused without an open live source)
        bool start_recording(const std::filesystem::path& path, const io::recording_options& options);
        void stop_recording(); // drains what is queued, then finalizes the file

        bool is_recording() const;
        io::recording_stats recording_stats() const;
        std::filesystem::path recording_path() const;

        // --- rest pose ----------------------------------------------------------------
        bool calibrate_rest_pose();
        void clear_rest_pose();

        // --- estimator (read + live option edits) -------------------------------------
        pose::exo_pose_estimator& estimator();
        const pose::exo_pose_estimator& estimator() const;

        // --- stepping -----------------------------------------------------------------
        // Advance one step: pull the newest latched detections and recompute joint states.
        // Each flag reports something that happened this step and is cleared once returned.
        struct poll_result { 
            bool new_pose{ false }; 
            bool stream_ended{ false }; 
            bool status_changed{ false };
        };
        poll_result poll();

        // Latest annotated frame for display; false if nothing new since `last_seq`.
        bool try_get_annotated_frame(
            cv::Mat& out_img,
            std::vector<pose::tag_detection_t>& out_dets,
            std::chrono::microseconds& out_ts,
            uint64_t& last_seq
        );

        // --- source metadata ----------------------------------------------------------
        std::string source_name() const;
        Eigen::Vector2i source_resolution() const;
        float source_fps() const;
        std::optional<hw::intrinsic_t> intrinsics() const; // color intrinsics of the open source (empty if none)
        uint32_t current_frame_id() const;
        std::chrono::microseconds last_timestamp() const { return _last_timestamp; }

        // --- recording playback (no-op without an open recording source) ---------------
        bool is_source_paused() const;
        void set_source_paused(bool paused);
        void seek_to_begin();
        void seek_to_end();

    private:
        // Log what changed since the last frame (tags appearing/disappearing, joints gaining or
        // losing tracking) instead of restating the same state every frame, plus a throughput
        // line on a timer. Cleared whenever the source changes.
        void _log_frame_diff();
        void _log_periodic_stats();
        void _reset_frame_log_state();

    private:
        double _default_tag_size_m;
        bool _annotate_frames; // observer keeps an annotated frame for a monitor GUI

        std::shared_ptr<hw::sensor_frame_provider> _provider;
        std::shared_ptr<pose_frame_observer> _observer;
        std::shared_ptr<io::frame_recorder> _recorder; // non-null only while recording
        pose::exo_pose_estimator _estimator;
        std::vector<pose::tag_detection_t> _detections;
        uint64_t _last_seq{ 0 };
        std::chrono::microseconds _last_timestamp{ 0 }; // device time of the latched frame
        bool _is_recording{ false }; // the open source is a recording file (vs a live camera)
        bool _status_changed{ false }; // a source/rest command changed the reported status; consumed by poll()

        // Capture settings of the open source, recorded alongside its frames.
        std::optional<int32_t> _exposure_us;
        std::optional<int32_t> _gain;

        // --- frame-log state (transitions + throughput; see _log_frame_diff) --------------
        uint64_t _seen_tag_mask{ 0 };  // bit t = tag id t was detected in the previous frame
        std::array<bool, pose::kNumJoints> _joint_tracked{}; // per joint: had a local rotation last frame
        std::chrono::steady_clock::time_point _stats_since{}; // start of the current summary window
        uint32_t _stats_frames{ 0 };   // frames polled in the window
        uint32_t _stats_detections{ 0 }; // tags detected across those frames
    };

} // namespace net
