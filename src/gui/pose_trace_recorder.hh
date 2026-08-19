#pragma once
#include "hw/calibration.hh"
#include "hw/timestamp.hh"
#include "pose/pose_estimator_base.hh"
#include "pose/joints_def.hh"
#include "pose/tag_detector.hh"
#include "pose/view_plane.hh"

#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace gui
{
    // Per-frame gates the estimator ran under, recorded so a dump reads correctly even if the
    // operator toggles them mid-capture. Which ones exist is estimator specific (tuning is not part
    // of the estimator base), so the caller fills what its estimator exposes.
    struct trace_gates_t
    {
        double max_hold_ms{ 0.0 };  // occlusion hold window
        double reset_gap_ms{ 0.0 }; // gap after which the filter reseeds
        std::optional<Eigen::Vector3d> hinge_axis; // set only while a 1-DOF hinge constraint is in force
    };

    // Rolling per-frame trace of the pose pipeline for offline debugging.
    //
    // Each captured frame stores what is needed to re-derive the estimator's output by hand: the tag
    // detections (id, corners, chosen camera-space position), the per-joint raw/smoothed rig positions
    // and the joint animation rotation, plus the per-frame gates (rest pose / smoothing / ...) and the
    // captured rest positions. Kept in a fixed-capacity ring so a transient glitch can be dumped with
    // its lead-up after it is seen.
    //
    // write_json() serializes the ring plus the static context (source, intrinsics, viewing plane,
    // rig) to a self-describing JSON file. Not thread-safe: capture()/write_json() from the GUI thread.
    class pose_trace_recorder
    {
    public:
        explicit pose_trace_recorder(std::size_t capacity = 600); // ~20 s @30 fps

        void set_capacity(std::size_t capacity); // trims the ring if it shrinks
        std::size_t capacity() const { return _capacity; }
        std::size_t size() const { return _frames.size(); }
        bool empty() const { return _frames.empty(); }
        void clear() { _frames.clear(); }

        // Append one frame. `detections` are the raw detections behind the current joint states.
        // Copies what it needs; holds no reference afterwards.
        void capture(
            hw::timestamp_t sensor_ts,
            std::span<const pose::tag_detection_t> detections,
            const pose::pose_estimator_base& estimator,
            const trace_gates_t& gates
        );

        // Serialize the ring + static context to `path` (pretty-printed JSON). The source metadata
        // is stamped once at dump time. Returns false on an I/O / serialization error (logged).
        bool write_json(
            const std::filesystem::path& path,
            const std::string& source_name,
            const Eigen::Vector2i& source_resolution,
            float source_fps,
            const std::optional<hw::intrinsic_t>& intrinsics, // empty when the source reported none
            pose::view_plane_t view_plane // names which estimator the frames came from
        ) const;

    private:
        // --- copied per-frame trace (POD-ish; no live handles) ---------------------------------

        struct detection_rec_t
        {
            int id{ 0 };
            int hamming{ 0 };
            float decision_margin{ 0.0f };
            Eigen::Vector2f center{ Eigen::Vector2f::Zero() };
            std::array<Eigen::Vector2f, 4> corners{};
            std::optional<pose::joint_id_t> joint_id; // rig joint this tag maps to (nullopt if off-rig)
            std::optional<Eigen::Vector3d> position;  // chosen pose translation, tag->camera [m]
        };

        struct joint_rec_t
        {
            bool detected{ false }; // a fresh position was measured this frame
            bool held{ false };     // no detection; reused the last smoothed position (hold window)
            bool lost{ false };     // no position available this frame
            std::optional<Eigen::Vector3d> raw_position;      // raw measured rig-space position [m]
            std::optional<Eigen::Vector3d> position;          // smoothed + held rig-space position [m]
            std::optional<Eigen::Quaterniond> local_anim_rot; // parent-relative animation rotation, vs the captured rest
            std::optional<double> sagittal_segment_angle;        // bone attitude from vertical [rad], anterior +
            std::optional<double> sagittal_clinical_angle;       // bend vs the parent bone [rad], flexion/dorsiflexion +
            std::optional<double> sagittal_included_angle;       // inter-bone angle [rad], pi when collinear
            std::optional<double> sagittal_clinical_angle_delta; // change of the clinical angle since the captured rest [rad]
            std::optional<double> sagittal_segment_angle_delta;  // change of the segment angle since the captured rest [rad]
        };

        struct frame_rec_t
        {
            std::uint64_t seq{ 0 }; // monotonic capture index (survives ring eviction; not the array position)
            hw::timestamp_t t{}; // capture time of the frame these values were computed from
            std::vector<detection_rec_t> detections;
            std::array<joint_rec_t, pose::kNumJoints> joints{};

            // Gates in effect this frame.
            bool has_rest_pose{ false };
            bool smoothing_enabled{ false };
            trace_gates_t gates{};
            std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> rest_position{}; // captured rest position per joint
        };

        std::size_t _capacity;
        std::deque<frame_rec_t> _frames;
        std::uint64_t _next_seq{ 0 }; // monotonic capture index, stamped into each frame
    };

} // namespace gui
