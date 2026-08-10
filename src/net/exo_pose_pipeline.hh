#pragma once
#include "app_config.hh"
#include "source_address.hh"

#include "hw/calibration.hh"
#include "hw/frame_format.hh"
#include "hw/roi.hh"
#include "hw/sensor_frame_provider.hh"
#include "hw/source_backend.hh"
#include "io/frame_recorder.hh"
#include "pose/frontal_pose_estimator.hh"
#include "pose/marker_tracker.hh"
#include "pose/sagittal_pose_estimator.hh"
#include "pose/pose_estimator_base.hh"
#include "pose/view_plane.hh"

#include <opencv2/core.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace net
{
    // forward declaration of the worker-thread marker detection observer
    class pose_frame_observer;

    // The pose pipeline: owns the source, the detection worker, and the joint estimator, and
    // steps them independently of any network transport. A server or GUI holds one and drives
    // it via poll(). Not thread-safe: call from a single thread (the loop/GUI thread). Only the
    // provider's worker crosses threads: it publishes measurements into the tracker and the drawn
    // frame into the observer, each under that object's own lock.
    class exo_pose_pipeline final
    {
    public:
        explicit exo_pose_pipeline(bool annotate_frames);
        ~exo_pose_pipeline();

        exo_pose_pipeline(const exo_pose_pipeline&) = delete;
        exo_pose_pipeline& operator=(const exo_pose_pipeline&) = delete;

        // --- source control -----------------------------------------------------------
        bool open_source(const app::app_config_t& config);
        void close_source();

        bool is_source_open() const;
        bool is_source_recording() const;

        // --- recording ----------------------------------------------------------------
        // Captures the live source's frames to a recording file.
        // (Refused without an open live source)
        bool start_recording(const std::filesystem::path& path, const io::recording_options_t& options);
        void stop_recording(); // drains what is queued, then finalizes the file

        bool is_recording() const;
        io::recording_stats_t recording_stats() const;
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

        // --- marker tracking ----------------------------------------------------------
        // The tracker owns everything that differs between marker technologies. It is null until
        // the first source is opened (the config it is built from arrives then) and outlives a
        // close, so the last frame's readouts stay available.
        //
        // A caller wanting one technology's own controls asks for that type:
        //
        //   if (auto* t = dynamic_cast<pose::apriltag_tracker*>(pipe.tracker())) { ... }
        //
        // which reads null both when nothing is open and when another technology is running. The
        // running tracker's type is the one record of which is which, so adding a technology leaves
        // this class alone.
        pose::marker_tracker_base* tracker() { return _tracker.get(); }
        const pose::marker_tracker_base* tracker() const { return _tracker.get(); }

        // --- stepping -----------------------------------------------------------------
        // Advance one step: pull the newest latched detections and recompute joint states.
        // Each flag reports something that happened this step and is cleared once returned.
        struct poll_result_t { 
            bool new_pose{ false }; 
            bool stream_ended{ false }; 
            bool status_changed{ false };
        };
        poll_result_t poll();

        // Latest annotated frame for display, with the source frame it was drawn over; false if
        // nothing new since `last_seq`. The detections drawn on it are the tracker's to hand out,
        // since only it knows their shape.
        //
        // The two describe the same capture, so a point picked off the drawn image names the same
        // pixel in the source. Sampling a colour reads that source: the overlay would otherwise
        // contribute its own pixels to what a marker is measured to be.
        bool try_get_annotated_frame(cv::Mat& out_img, cv::Mat& out_source, uint64_t& last_seq);

        // --- source metadata ----------------------------------------------------------
        hw::source_backend_t source_backend() const;
        std::string source_name() const;
        Eigen::Vector2i source_resolution() const;
        float source_fps() const;
        std::optional<hw::intrinsic_t> intrinsics() const; // color intrinsics of the open source (empty if none)

        // Position of the newest frame in the open source's stream; restarts on every open.
        uint32_t current_frame_seq() const;

        // Capture time of the frame the estimator last stepped on, which is the moment the joint
        // state describes. Meaningful only once `has_pose()`.
        hw::timestamp_t last_timestamp() const { return _last_timestamp; }

        // True once the estimator has stepped on a frame. Frames arriving is not the same thing:
        // a tracker can refuse every one of them, as the color one does for a gray source.
        bool has_pose() const { return _has_pose; }

        // --- recording playback (no-op without an open recording source) ---------------
        bool is_source_paused() const;
        void set_source_paused(bool paused);
        void seek_to_begin();
        void seek_to_end();

    private:
        // Log what changed since the last frame (markers appearing/disappearing, joints gaining or
        // losing tracking) instead of restating the same state every frame, plus a throughput
        // line on a timer. Cleared whenever the source changes.
        void _log_frame_diff();
        void _log_periodic_stats();
        void _reset_frame_log_state();

        // Build the estimator for `view_plane` when it differs from the active one, and point _active at it.
        // A no-op when unchanged, leaving the existing estimator in place.
        void _select_estimator(pose::view_plane_t view_plane);

    private:
        bool _annotate_frames; // observer keeps an annotated frame for a monitor GUI

        std::shared_ptr<hw::sensor_frame_provider> _provider;
        std::shared_ptr<pose_frame_observer> _observer;

        // Shared with the observer, which calls into it from the provider's worker thread.
        std::shared_ptr<pose::marker_tracker_base> _tracker;

        std::shared_ptr<io::frame_recorder> _recorder; // non-null only while recording

        // At most one estimator is engaged at a time and _active points at it, so the readers of
        // joint state never learn which plane produced it. Both are empty until a source is opened.
        pose::view_plane_t _view_plane{ pose::view_plane_t::frontal };
        std::optional<pose::frontal_pose_estimator> _frontal;
        std::optional<pose::sagittal_pose_estimator> _sagittal;
        pose::pose_estimator_base* _active{ nullptr };
        hw::timestamp_t _last_timestamp{}; // capture time of the frame the estimator last stepped on
        bool _has_pose{ false };           // that frame exists; cleared when a new source is opened
        bool _is_recording{ false }; // the open source is a recording file (vs a live camera)
        bool _status_changed{ false }; // a source/rest command changed the reported status; consumed by poll()

        // Capture settings of the open source, recorded alongside its frames.
        std::optional<int32_t> _exposure_us;
        std::optional<int32_t> _gain;

        // --- frame-log state (transitions + throughput; see _log_frame_diff) --------------
        bool _tracker_was_tracking{ false }; // the tracker had identified its markers last frame
        std::array<bool, pose::kNumJoints> _joint_tracked{}; // per joint: had a local rotation last frame
        std::chrono::steady_clock::time_point _stats_since{}; // start of the current summary window
        uint32_t _stats_frames{ 0 };   // frames polled in the window
        uint32_t _stats_detections{ 0 }; // markers detected across those frames
    };

} // namespace net
