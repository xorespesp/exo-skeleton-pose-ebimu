#pragma once
#include "hw/calibration.hh"
#include "pose/exo_pose_estimator.hh"
#include "pose/tag_detector.hh"

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
    // Rolling per-frame trace of the position pipeline for offline debugging.
    //
    // Each captured frame stores what is needed to re-derive the estimator's output by hand: the tag
    // detections (id, corners, chosen camera-space position), the per-joint raw/smoothed 3D positions
    // and the IK animation rotation, plus the per-frame gates (rest pose / smoothing / hinge) and the
    // captured rest positions. Kept in a fixed-capacity ring so a transient glitch can be dumped with
    // its lead-up after it is seen.
    //
    // write_json() serializes the ring plus the static context (source, intrinsics, options, rig) to
    // a self-describing JSON file. Not thread-safe: capture()/write_json() from the GUI thread.
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
            std::chrono::microseconds sensor_ts,
            std::span<const pose::tag_detection_t> detections,
            const pose::exo_pose_estimator& estimator
        );

        // Serialize the ring + static context to `path` (pretty-printed JSON). The source metadata
        // is stamped once at dump time. Returns false on an I/O / serialization error (logged).
        bool write_json(
            const std::filesystem::path& path,
            const std::string& source_name,
            const Eigen::Vector2i& source_resolution,
            float source_fps,
            const std::optional<hw::intrinsic_t>& intrinsics
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
            std::optional<Eigen::Vector3d> raw_position;      // raw measured position [m]
            std::optional<Eigen::Vector3d> position;          // smoothed + held position [m]
            std::optional<Eigen::Quaterniond> local_anim_rot; // IK animation rotation
        };

        struct frame_rec_t
        {
            std::uint64_t seq{ 0 }; // monotonic capture index (survives ring eviction; not the array position)
            std::chrono::microseconds t{ 0 };
            std::vector<detection_rec_t> detections;
            std::array<joint_rec_t, pose::kNumJoints> joints{};

            // Gates in effect this frame (captured so a dump reads correctly even if the operator
            // toggles them mid-capture).
            bool has_rest_pose{ false };
            bool smoothing_enabled{ false };
            bool hinge_enabled{ false };
            double max_hold_ms{ 0.0 };
            double reset_gap_ms{ 0.0 };
            Eigen::Vector3d hinge_axis{ Eigen::Vector3d::Zero() };
            std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> rest_position{}; // captured rest position per joint
        };

        std::size_t _capacity;
        std::deque<frame_rec_t> _frames;
        std::uint64_t _next_seq{ 0 }; // monotonic capture index, stamped into each frame
    };

} // namespace gui
