#pragma once
#include "joints_def.hh"

#include <Eigen/Geometry>

#include <chrono>
#include <optional>
#include <span>

namespace pose
{
    using millis_f64 = std::chrono::duration<double, std::milli>;
    using seconds_f64 = std::chrono::duration<double>;

    // Per-joint result of one estimation step. Positions are in rig space (see joints_def.hh).
    //
    // The rotation and both angles express motion measured from the captured rest pose: they read
    // identity and zero while the exo holds the pose that was captured, and grow as it leaves it.
    // The two angles are flexion in the exo's sagittal plane [rad], signed by the right-hand rule
    // about the rig's lateral axis. (`kRigLateralAxis`)
    //
    // NOTE: the rotation and the angles can disagree. The angles are measured in the hinge plane,
    //       while the rotation is whatever the estimator solved: a solve left free of a 1-DOF hinge
    //       constraint carries off-hinge components, and `quat_hinge_angle()` over it then returns
    //       less than the angles state. A reader picks one form rather than mixing them.
    struct joint_state_t
    {
        std::optional<Eigen::Vector3d> raw_position;      // raw position this frame (fresh detection)
        std::optional<Eigen::Vector3d> position;          // smoothed + held position (drives the skeleton)
        std::optional<Eigen::Quaterniond> local_anim_rot; // parent-relative rotation vs rest (drives the rig)

        // This joint's turn relative to its parent bone: 
        // the angle form of `local_anim_rot`, taken in the hinge plane (see hinge_angle.hh).
        std::optional<double> local_sagittal_angle;

        // This joint's own bone's turn in the rig frame. 
        // The running total of `local_sagittal_angle` down the chain, 
        // which the leg's joints sharing one hinge axis makes a plain sum, 
        // so it states a bone's attitude without walking its ancestors.
        std::optional<double> absolute_sagittal_angle;
    };

    // ---------------------------------------------------------------------------
    // Pose estimator base: per-joint rig state, readable without knowing the estimator
    // ---------------------------------------------------------------------------
    //
    // Every estimator fills `joint_state_t` for every joint of the rig, so the readers of that
    // state (plots, diagnostic trace, protocol broadcast) see one shape and need no knowledge of
    // which implementation produced it.
    //
    // Feeding an estimator is deliberately NOT part of this base. What one frame's measurement looks
    // like depends on the viewing plane and on the marker technology (tag ids and corners, a blob
    // centroid, ...), so each implementation declares its own update() taking the input it needs.
    // Whoever constructed a concrete estimator calls that directly; every other reader works through
    // this base.
    //
    // Tuning knobs are absent for the same reason: they describe an algorithm rather than the rig.
    class pose_estimator_base
    {
    public:
        // The rig frame's lateral axis, pointing to the exo's left (see joints_def.hh). Every leg
        // joint hinges about this one axis and every rotation and angle below is expressed about
        // it, so a reader takes a flexion back out of `local_anim_rot` about this and nothing else.
        //
        // It names a direction on the exo, not on any camera, so it holds for every viewpoint. What
        // differs per viewpoint is where it falls in camera coordinates: a frontal camera's frame is
        // the rig frame outright, which is why that mount is installed level and square; a side view
        // looks along it, and the estimator working from one carries its measurements over itself.
        static inline const Eigen::Vector3d kRigLateralAxis{ Eigen::Vector3d::UnitX() };

    public:
        virtual ~pose_estimator_base() = default;

        pose_estimator_base(const pose_estimator_base&) = delete;
        pose_estimator_base& operator=(const pose_estimator_base&) = delete;

        // Latch the current per-joint measurements as the rest (bind) reference. Joint rotations are
        // expressed against this reference, so none are produced until it is captured.
        // Returns false if no joint was measured this frame.
        virtual bool calibrate_rest_pose() = 0;
        virtual void clear_rest_pose() = 0;
        virtual bool has_rest_pose() const = 0;

        // Drop the tracking state: per-joint filters, occlusion timers, and held measurements. The
        // next frame starts cold. Call when the input stream changes so a new source is not smoothed
        // or held against the previous one's stale state.
        virtual void reset_tracking() = 0;

        virtual const joint_state_t& get_joint_state(joint_id_t j) const = 0;
        virtual std::span<const joint_state_t> get_joint_states() const = 0;

        // Captured rest rig-space position of `j` (the bind position). Empty when uncalibrated or
        // the joint had no measurement at capture.
        virtual std::optional<Eigen::Vector3d> get_rest_position(joint_id_t j) const = 0;

        // Whether `joint_state_t::position` currently carries a smoothed (and possibly held) value
        // rather than this frame's raw measurement. A viewer uses this to pick which of the two to
        // draw without reaching into implementation-specific options.
        virtual bool uses_smoothed_positions() const = 0;

    protected:
        pose_estimator_base() = default;
    };

} // namespace pose
