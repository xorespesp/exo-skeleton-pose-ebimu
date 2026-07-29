#pragma once
#include "tag_detector.hh"
#include "dsp/one_euro_filter.hh"

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

    // NOTE: The estimator walks kJointsInfo in a single forward pass, 
    // so every parent must precede its child. (parent index <= own)
    static_assert([] {
        for (const auto& j : kJointsInfo) {
            if (static_cast<size_t>(j.parent) > static_cast<size_t>(j.id)) { return false; }
        }
        return true;
    }(), "kJointsInfo must be parent-before-child ordered");

    // ---------------------------------------------------------------------------
    // Per-joint result of one estimation step
    // ---------------------------------------------------------------------------
    struct joint_state_t
    {
        std::optional<Eigen::Vector3d> raw_position;      // raw camera-space position this frame (fresh detection)
        std::optional<Eigen::Vector3d> position;          // smoothed + held camera-space position (drives the skeleton)
        std::optional<Eigen::Quaterniond> local_anim_rot; // parent-relative IK rotation vs the captured rest (drives the rig)
    };

    // ---------------------------------------------------------------------------
    // exo skeleton pose estimator: tag detections -> per-joint 3D points + leg IK rotations
    // ---------------------------------------------------------------------------
    //
    // A joint's position is the detected tag's camera-space translation, smoothed and held through
    // occlusion. Once a rest pose is captured (any neutral stance), leg IK maps the position track to
    // per-joint local_anim_rot: the minimal-swing rotation of each bone from its rest direction,
    // constrained to a 1-DOF hinge (every exo leg joint is a forward/back hinge). local_anim_rot
    // drives the skeleton and is the value broadcast to clients.
    class exo_pose_estimator
    {
    public:
        struct options_t
        {
            // --- position track (camera-space 3D points) ---
            bool enable_position_smoothing{ true }; // One Euro per axis (hold still applies)
            dsp::one_euro_params position_filter{};

            // Joint occlusion policy (shared by the point track).
            millis_f64 max_hold{ 200.0 };  // hold a lost joint's last point up to this long (~6 frames @30fps)
            millis_f64 reset_gap{ 400.0 }; // beyond this gap, reseed the filter to the raw sample
            seconds_f64 dt_min{ 0.001 };   // dt clamp floor [s]
            seconds_f64 dt_max{ 0.100 };   // dt clamp ceiling [s] (avoids a jump after a long pause)

            // --- leg IK (1-DOF hinge) ---
            // Constrain every leg joint (hip/knee/ankle) to a single forward/back hinge about the
            // lateral axis below, dropping off-hinge components as tag-position error. When off, each
            // joint gets the free minimal-swing rotation.
            bool enable_hinge_constraint{ true };
            // Lateral hinge axis in the camera frame: ~(1,0,0) for a frontal view, ~(0,0,1) for a
            // sagittal (side) view. All leg joints share this axis.
            Eigen::Vector3d hinge_axis_world{ Eigen::Vector3d::UnitX() };
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

        // Latch the current per-joint 3D points as the rest (bind) reference.
        // Returns false if no joint had a point this frame.
        bool calibrate_rest_pose();
        void clear_rest_pose();
        bool has_rest_pose() const;

        // Drop the position track: per-joint filters, occlusion timers, and held points. The next
        // frame starts cold (the filter reseeds to its raw sample). Call when the input stream
        // changes so a new source is not smoothed or held against the previous one's stale state.
        void reset_tracking();

        const joint_state_t& get_joint_state(joint_id_t j) const;
        std::span<const joint_state_t> get_joint_states() const;

        // Captured rest camera-space position of `j` (the bind position). Empty when uncalibrated or
        // the joint had no position at capture. Rest bone directions/lengths derive from these.
        std::optional<Eigen::Vector3d> get_rest_position(joint_id_t j) const;

    private:
        struct context_t;

        options_t _opt;
        std::unique_ptr<context_t> _ctx;
    };

} // namespace pose
