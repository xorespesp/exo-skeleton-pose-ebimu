#pragma once
#include "tag_detector.hh"
#include "rotation_filter.hh"

#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <cstddef>
#include <cstdint>

namespace pose
{
    using millis_f64 = std::chrono::duration<double, std::milli>;
    using seconds_f64 = std::chrono::duration<double>;

    // ---------------------------------------------------------------------------
    // Skeleton definition
    // ---------------------------------------------------------------------------

    enum class joint_id_t
    {
        pelvis = 0, r_knee, l_knee, r_ankle, l_ankle, r_foot, l_foot,
        count
    };

    inline constexpr size_t kNumJoints{ static_cast<size_t>(joint_id_t::count) };

    // Static per-joint definition: binds a joint to its tag and its parent.
    struct joint_info_t
    {
        joint_id_t id;
        std::string_view name;
        int tag_id; // tag id bound to this joint
        joint_id_t parent; // parent joint (== id itself for a root)
    };

    // Single source of truth for the rig. Add a joint => add an enum value + one row.
    inline constexpr std::array<joint_info_t, kNumJoints> kJointsInfo{ {
        { joint_id_t::pelvis,  "pelvis",  0, joint_id_t::pelvis  }, // root (self-parent)
        { joint_id_t::r_knee,  "r_knee",  1, joint_id_t::pelvis  },
        { joint_id_t::l_knee,  "l_knee",  2, joint_id_t::pelvis  },
        { joint_id_t::r_ankle, "r_ankle", 3, joint_id_t::r_knee  },
        { joint_id_t::l_ankle, "l_ankle", 4, joint_id_t::l_knee  },
        { joint_id_t::r_foot,  "r_foot",  5, joint_id_t::r_ankle },
        { joint_id_t::l_foot,  "l_foot",  6, joint_id_t::l_ankle },
    } };

    // Table lookups over kJointsInfo.
    constexpr const joint_info_t& joint_info(joint_id_t j) {
        return kJointsInfo[static_cast<size_t>(j)];
    }

    // Reverse lookup: tag id -> joint (linear over kJointsInfo).
    constexpr std::optional<joint_id_t> tag_to_joint(int tag_id) {
        for (const auto& info : kJointsInfo) {
            if (info.tag_id == tag_id) { return info.id; }
        }
        return std::nullopt;
    }

    // NOTE: a root joint is its own parent (parent == id).
    constexpr bool is_root_joint(joint_id_t j) {
        return joint_info(j).parent == j;
    }

    // NOTE: The estimator computes local rotations in a single forward pass over
    // kJointsInfo, so every parent must precede its child. (parent index <= own)
    static_assert([] {
        for (const auto& j : kJointsInfo) {
            if (static_cast<size_t>(j.parent) > static_cast<size_t>(j.id)) { return false; }
        }
        return true;
    }(), "kJointsInfo must be parent-before-child ordered");

    // ---------------------------------------------------------------------------
    // Leg-hinge constraint (1-DOF joint)
    // ---------------------------------------------------------------------------
    //
    // IMPORTANT: Each leg joint rotates about a single leg-hinge axis.
    // By mounting convention every tag's local X axis is aligned with its joint's flexion axis (see docs/README.md),
    // so in the rest-relative frame the valid motion is a pure rotation about X for every joint.
    inline const Eigen::Vector3d kExoHingeLocalAxis = Eigen::Vector3d::UnitX();

    // ---------------------------------------------------------------------------
    // Per-joint result of one estimation step
    // ---------------------------------------------------------------------------
    struct joint_state_t
    {
        std::optional<Eigen::Isometry3d> view_pose; // tag -> camera; set iff the tag was detected this frame
        std::optional<Eigen::Quaterniond> global_rot; // smoothed+held global (camera-frame) rotation available (fresh or held; consumed by the relative pass)
        std::optional<Eigen::Quaterniond> local_rot; // rotation relative to the parent joint's tag
        std::optional<Eigen::Quaterniond> local_anim_rot; // local_rot relative to the captured rest pose (drives the skeleton)
        int selected_candidate{ -1 }; // which pose candidate `view_pose` came from this frame; -1 if held/lost (diagnostic)
    };

    // ---------------------------------------------------------------------------
    // exo skeleton pose estimator: tag detections -> per-joint local / animation rotations
    // ---------------------------------------------------------------------------
    //
    // Poses are tag->camera; the camera is not a fixed world frame, so only
    // rotations relative to a parent tag (local_rot = R_parent^-1 * R_child; root: vs camera)
    // or to a captured rest pose (local_anim_rot = R_rest^-1 * R_local) are meaningful.
    // local_anim_rot drives the skeleton; the rest pose is captured by
    // calibrate_rest_pose() in any neutral stance. (not necessarily a T-pose)
    class exo_pose_estimator
    {
    public:
        struct options_t
        {
            bool enable_smoothing = true; // master switch for the smoothing kernel (hold still applies)

            // Joint occlusion policy (filter-agnostic, owned by the estimator).
            millis_f64 max_hold{ 200.0 };  // hold a lost joint's last rotation up to this long (~6 frames @30fps)
            millis_f64 reset_gap{ 400.0 }; // beyond this gap, reseed the kernel to the raw sample
            seconds_f64 dt_min{ 0.001 };   // dt clamp floor [s]
            seconds_f64 dt_max{ 0.100 };   // dt clamp ceiling [s] (avoids a jump after a long pause)

            // Rotation-smoothing kernel selection + params (only one_euro for now).
            rotation_filter_config filter{};

            // Hinge constraint: reject the planar-ambiguity flip and constrain each joint to its 1-DOF leg-hinge axis. 
            // Active only once a rest pose is captured.
            bool enable_hinge_constraint = true;
        };

        explicit exo_pose_estimator(const options_t& opt = {});
        ~exo_pose_estimator();

        exo_pose_estimator(const exo_pose_estimator&) = delete;
        exo_pose_estimator& operator=(const exo_pose_estimator&) = delete;

        options_t& options() noexcept { return _opt; }
        const options_t& options() const noexcept { return _opt; }

        // Ingest one frame's detections and recompute every joint state.
        void update(
            std::span<const tag_detection_t> tag_detections, 
            std::chrono::microseconds sensor_timestamp // sensor timestamp of the frame
        );

        // Latch the current per-joint local_rot as the rest (bind) reference.
        // Returns false if no joint had a computable local_rot this frame.
        bool calibrate_rest_pose();
        void clear_rest_pose();
        bool has_rest_pose() const;

        const joint_state_t& get_joint_state(joint_id_t j) const;
        std::span<const joint_state_t> get_joint_states() const;

        // Captured rest (bind) reference rotation for `j`, in the joint's parent frame.
        // Empty when no rest pose is calibrated or that joint had no computable local_rot at
        // capture time. Read-only view for diagnostics/serialization. (see calibrate_rest_pose)
        std::optional<Eigen::Quaterniond> rest_rotation(joint_id_t j) const;

    private:
        struct context_t;

        options_t _opt;
        std::unique_ptr<context_t> _ctx;
    };

} // namespace pose
