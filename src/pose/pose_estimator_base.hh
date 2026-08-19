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
    // A joint's state describes the articulation at it: the rotation and every angle turn the
    // bone that leaves it toward its child, so a marker end site (the feet) carries a position
    // and nothing else.
    //
    // The three measured angles follow the sagittal-plane biomechanics conventions of
    // docs/joint_angle_convention.md, in radians, and involve no captured rest pose. The
    // estimators measure in the rig's own hinge sign and convert through the per-joint table in
    // joints_def.hh at exactly this boundary, so these fields are the complete published set and
    // the only angle convention a reader of this state ever sees; the wire and the plots read
    // them as they are.
    //
    // The rotation and the two delta angles express motion measured from the captured rest pose:
    // identity and zero while the exo holds that pose, growing as it leaves it. The rotation is
    // what drives a rig from its bind pose.
    //
    // NOTE: the rotation and the delta angles can disagree. The deltas are measured in the hinge
    //       plane, while the rotation is whatever the estimator solved: a solve left free of a
    //       1-DOF hinge constraint carries off-hinge components, and `quat_hinge_angle()` over
    //       it then returns less than the deltas state. A reader picks one form rather than
    //       mixing them.
    struct joint_state_t
    {
        std::optional<Eigen::Vector3d> raw_position;      // raw position this frame (fresh detection)
        std::optional<Eigen::Vector3d> position;          // smoothed + held position (drives the skeleton)
        std::optional<Eigen::Quaterniond> local_anim_rot; // parent-relative rotation vs rest (drives the rig), about `kRigLateralAxis`

        // Segment Angle: the attitude of this joint's bone, measured from vertically down
        // (`kRigDownAxis`), positive tilted toward the exo's front. 0 for a bone hanging
        // straight down; the root reads 0 by definition, its bone being the torso standing on
        // the reference axis. A mount tilt about the rig's lateral axis (a side camera's roll)
        // enters this reading directly, which is part of what the level, square mount
        // requirement covers.
        std::optional<double> sagittal_segment_angle;

        // Clinical Joint Angle (Neutral Zero Method): the bend at this joint versus its parent
        // bone, 0 at the neutral stance, positive in flexion (hip, knee) and dorsiflexion
        // (ankle). The ankle's neutral holds the foot MARKER line perpendicular to the shank;
        // its offset to the robot's own foot axis is a fixed constant of the tag placement. A
        // mount tilt about the rig's lateral axis shifts both adjacent segment angles equally
        // and cancels out of this reading, except at the hip, whose reference is the virtual
        // torso.
        std::optional<double> sagittal_clinical_angle;

        // Included Angle: the angle between the two bones meeting at this joint, as a signed
        // continuous quantity: pi when the bones are collinear, shrinking with flexion and
        // growing past pi in extension; the ankle's neutral reads pi/2. Filled from
        // `sagittal_clinical_angle` in the same statement block, through the one formula in
        // joints_def.hh, so the pair always states one bend.
        std::optional<double> sagittal_included_angle;

        // Changes since the captured rest pose, in the same conventions as the measured angles
        // above: the angle forms of `local_anim_rot`, 0 while the exo holds the captured pose.
        //
        // NOTE: diagnostic-only fields; nothing downstream requires them. They feed the angle
        //       plots' vs-rest views, which compare the rotation against the hinge-plane reading.
        std::optional<double> sagittal_clinical_angle_delta;
        std::optional<double> sagittal_segment_angle_delta;
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

        // The rig frame's down axis (see joints_def.hh): the reference direction segment angles
        // are measured from, so a bone hanging straight down reads 0. It doubles as the torso
        // segment's own direction (the exo stands upright on its fixed frame), which is what
        // makes a thigh's segment angle the hip joint's angle outright and starts the chain sums
        // at a pelvis of 0. The decomposition built on it is laid out at the top of
        // sagittal_pose_estimator.cc.
        static inline const Eigen::Vector3d kRigDownAxis{ Eigen::Vector3d::UnitY() };

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

        // The delivered images now cover a different part of the sensor, so pixel coordinates no
        // longer mean what they meant. An estimator drops everything it holds in them. What it
        // holds in rig space is unaffected, which is why doing nothing is the default.
        virtual void on_frame_geometry_changed() {}

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
