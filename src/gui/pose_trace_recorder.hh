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
    // Rolling per-frame trace of the pose pipeline for offline debugging.
    //
    // Each captured frame stores the full picture needed to re-derive the estimator's decisions
    // by hand from the algorithm source: the raw tag detections (both planar-ambiguity pose
    // candidates + their obj_err), the estimator's chosen candidate and every output rotation,
    // plus the per-frame gates (rest pose / hinge / smoothing) that steer selection. Kept in a
    // fixed-capacity ring so a transient glitch can be dumped with its lead-up after it is seen.
    //
    // write_json() serializes the ring plus the static context (source, intrinsics, options, rig)
    // to a self-describing JSON file. Not thread-safe: capture()/write_json() from the GUI thread.
    class pose_trace_recorder
    {
    public:
        explicit pose_trace_recorder(std::size_t capacity = 600); // ~20 s @30 fps

        void set_capacity(std::size_t capacity); // trims the ring if it shrinks
        std::size_t capacity() const { return _capacity; }
        std::size_t size() const { return _frames.size(); }
        bool empty() const { return _frames.empty(); }
        void clear() { _frames.clear(); }

        // Append one frame. `detections` are the raw detections behind the current joint states
        // (both carry the same frame). Copies what it needs; holds no reference afterwards.
        void capture(
            std::chrono::microseconds sensor_ts,
            std::span<const pose::tag_detection_t> detections,
            const pose::exo_pose_estimator& estimator);

        // Serialize the ring + static context to `path` (pretty-printed JSON). The source metadata
        // is stamped once at dump time. Returns false on an I/O / serialization error (logged).
        bool write_json(
            const std::filesystem::path& path,
            const std::string& source_name,
            const Eigen::Vector2i& source_resolution,
            float source_fps,
            const std::optional<hw::intrinsic_t>& intrinsics) const;

    private:
        // --- copied per-frame trace (POD-ish; no live handles) ---------------------------------

        struct pose_rec_t
        {
            double obj_err{ 0.0 };
            Eigen::Quaterniond q{ Eigen::Quaterniond::Identity() }; // rotation of tag->camera
            Eigen::Vector3d t{ Eigen::Vector3d::Zero() };           // translation of tag->camera [m]
        };

        struct detection_rec_t
        {
            int id{ 0 };
            int hamming{ 0 };
            float decision_margin{ 0.0f };
            Eigen::Vector2f center{ Eigen::Vector2f::Zero() };
            std::array<Eigen::Vector2f, 4> corners{};
            std::optional<pose::joint_id_t> joint; // rig joint this tag maps to (nullopt if off-rig)
            int num_candidates{ 0 };
            std::array<pose_rec_t, 2> candidates{}; // obj_err ascending
            std::optional<pose_rec_t> chosen;       // detector's selected pose (min-error policy)
        };

        struct joint_rec_t
        {
            bool detected{ false };      // a fresh candidate was bound and drove this joint
            bool held{ false };          // no detection; reused the last smoothed rotation (hold window)
            bool lost{ false };          // no rotation available this frame
            bool locked_pelvis{ false }; // rest-locked constant base orientation
            int  selected_candidate{ -1 };
            std::optional<Eigen::Quaterniond> view_q;    // absolute selected rotation (camera frame)
            std::optional<Eigen::Vector3d>    view_t;    // absolute selected translation [m]
            std::optional<Eigen::Quaterniond> global_rot;
            std::optional<Eigen::Quaterniond> local_rot;
            std::optional<Eigen::Quaterniond> local_anim_rot;
        };

        struct frame_rec_t
        {
            std::uint64_t seq{ 0 }; // monotonic capture index (survives ring eviction; not the array position)
            std::chrono::microseconds t{ 0 };
            std::vector<detection_rec_t> detections;
            std::array<joint_rec_t, pose::kNumJoints> joints{};

            // Gates in effect this frame (they steer candidate selection; captured so a dump reads
            // correctly even if the operator toggles them mid-capture).
            bool   has_rest_pose{ false };
            bool   hinge_enabled{ false };
            bool   smoothing_enabled{ false };
            double max_hold_ms{ 0.0 };
            double reset_gap_ms{ 0.0 };
            std::array<std::optional<Eigen::Quaterniond>, pose::kNumJoints> rest_rot{}; // rest reference per joint
        };

        std::size_t _capacity;
        std::deque<frame_rec_t> _frames;
        std::uint64_t _next_seq{ 0 }; // monotonic capture index, stamped into each frame
    };

} // namespace gui
