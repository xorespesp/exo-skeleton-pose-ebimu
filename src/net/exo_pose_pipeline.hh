#pragma once
#include "source_address.hh"

#include "hw/calibration.hh"
#include "hw/sensor_frame_provider.hh"
#include "io/frame_recorder.hh"
#include "pose/frontal_pose_estimator.hh"
#include "pose/sagittal_pose_estimator.hh"
#include "pose/pose_estimator_base.hh"
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
        explicit exo_pose_pipeline(bool annotate_frames);
        ~exo_pose_pipeline();

        exo_pose_pipeline(const exo_pose_pipeline&) = delete;
        exo_pose_pipeline& operator=(const exo_pose_pipeline&) = delete;

        // --- source control -----------------------------------------------------------
        bool open_source(
            const app::source_address& source_addr, // camera device or recording file to stream
            pose::view_plane_t view_plane,   // picks the estimator; a different one swaps it in, tuning fresh
            double tag_size_m,               // printed black-square edge length [m]
            std::optional<int32_t> exposure_us, // optional (nullopt: auto exposure)
            std::optional<int32_t> gain         // optional (nullopt: auto gain)
        );
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
        // All three are no-ops / false until a source has been opened, since the estimator holding
        // the reference is created then. Closing a source leaves the estimator in place, so a
        // captured rest survives until the next open replaces it.
        bool has_rest_pose() const;
        bool calibrate_rest_pose();
        void clear_rest_pose();

        // --- estimator ----------------------------------------------------------------
        // The active estimator, null until the first source is opened (the viewing plane it is built
        // for comes from that source). It outlives close_source(), so the last frame's joint state
        // stays readable. Enough for anything that only reads that state.
        pose::pose_estimator_base* estimator();
        const pose::pose_estimator_base* estimator() const;

        // Which plane the active estimator solves in; meaningful only once one exists.
        pose::view_plane_t view_plane() const { return _view_plane; }

        // Live tuning of the active estimator. At most one is non-null, matching the plane, which
        // lets a caller render the right controls without testing the plane itself.
        pose::frontal_pose_estimator::options_t* frontal_options();
        pose::sagittal_pose_estimator::options_t* sagittal_options();

        // The sagittal estimator's readouts (tracked leg, joint angles); null in a frontal run.
        const pose::sagittal_pose_estimator* sagittal_estimator() const;

        // --- tag detection tuning (tag size, decimation, pose method, ...), live -------
        // Persisted across source reopens; the detection worker rebuilds on the next frame.
        void set_tag_tuning(const pose::tag_tuning_t& tuning);
        pose::tag_tuning_t tag_tuning() const;

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

        // Build the estimator for `view_plane` when it differs from the active one, and point
        // _active at it. A no-op when unchanged, so reopening a source keeps its tuning.
        void _select_estimator(pose::view_plane_t view_plane);

    private:
        bool _annotate_frames; // observer keeps an annotated frame for a monitor GUI

        std::shared_ptr<hw::sensor_frame_provider> _provider;
        std::shared_ptr<pose_frame_observer> _observer;
        pose::tag_tuning_t _tuning{}; // detector tuning, applied to each observer (persists across reopens)
        std::shared_ptr<io::frame_recorder> _recorder; // non-null only while recording

        // At most one estimator is engaged at a time and _active points at it, so the readers of
        // joint state never learn which plane produced it. Both are empty until a source is opened.
        pose::view_plane_t _view_plane{ pose::view_plane_t::frontal };
        std::optional<pose::frontal_pose_estimator> _frontal;
        std::optional<pose::sagittal_pose_estimator> _sagittal;
        pose::pose_estimator_base* _active{ nullptr };
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
