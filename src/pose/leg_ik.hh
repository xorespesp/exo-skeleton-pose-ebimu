#pragma once
#include <Eigen/Geometry>

#include <optional>

// Analytic minimal-swing leg IK: given the four observed 3D joint positions of one leg
// (hip, knee, ankle, foot) plus the rest (bind) bone directions, recover each joint's
// parent-relative rotation. No iteration / optimization; each bone's rotation is the
// shortest-arc rotation that maps its rest direction onto its observed direction, taken in
// the parent's accumulated frame. Ported from leg-pose-ik-demo/leg_ik.py.
//
// Only swing is recovered (three points cannot determine a bone's axial twist), which matches
// a hinge-jointed leg. Bone lengths are ignored: output preserves the rig's rest lengths, so it
// is an orientation solver, not a reaching solver.
//
// Every joint of the exo leg is a 1-DOF forward/back hinge about a common lateral axis. When a
// world hinge axis is supplied, each joint's rotation is constrained to that single axis: the
// world axis is carried into the joint's parent frame and the rotation is projected onto it.
namespace pose
{
    inline Eigen::Vector3d ik_normalize(const Eigen::Vector3d& v)
    {
        const double n = v.norm();
        return (n < 1e-8) ? v : (v / n);
    }

    // One joint's parent-relative rotation: in the parent's accumulated frame, the minimal rotation
    // mapping `rest_dir` onto the observed bone direction (bone_end - bone_start). When
    // `world_hinge_axis` is set, the result is constrained to a 1-DOF rotation about that world axis
    // (expressed in this joint's parent frame), dropping any off-hinge component.
    inline Eigen::Quaterniond swing_local(
        const Eigen::Vector3d& bone_start,
        const Eigen::Vector3d& bone_end,
        const Eigen::Vector3d& rest_dir,
        const Eigen::Quaterniond& R_parent,
        const std::optional<Eigen::Vector3d>& world_hinge_axis = std::nullopt)
    {
        const Eigen::Vector3d d_world = ik_normalize(bone_end - bone_start);
        const Eigen::Vector3d d_local = R_parent.conjugate() * d_world; // into the parent frame
        // FromTwoVectors(a, b): rotates a onto b. We want rest_dir -> d_local.
        Eigen::Quaterniond R_local = Eigen::Quaterniond::FromTwoVectors(ik_normalize(rest_dir), d_local);

        if (world_hinge_axis.has_value())
        {
            const Eigen::Vector3d hinge_local = ik_normalize(R_parent.conjugate() * world_hinge_axis.value());
            const Eigen::AngleAxisd aa{ R_local };
            const Eigen::Vector3d rotvec = aa.axis() * aa.angle();
            const double a = rotvec.dot(hinge_local); // keep only the hinge-axis component
            R_local = Eigen::Quaterniond{ Eigen::AngleAxisd{ a, hinge_local } };
        }
        return R_local.normalized();
    }

    // Per-leg parent-relative rotations (each relative to its parent bone, from rest).
    struct leg_ik_result_t
    {
        Eigen::Quaterniond hip{ Eigen::Quaterniond::Identity() };   // thigh swing (pelvis->knee bone)
        Eigen::Quaterniond knee{ Eigen::Quaterniond::Identity() };  // shin swing (knee->ankle bone)
        Eigen::Quaterniond ankle{ Eigen::Quaterniond::Identity() }; // foot swing (ankle->foot bone)
    };

    // Solve one leg's chain pelvis -> hip -> knee -> ankle -> foot. `*_rest` are the normalized rest
    // bone directions; `R_pelvis` is the root's world rotation (identity when the pelvis is a fixed
    // base). `hinge_axis_world`, if set, constrains every joint to 1 DOF about that common axis.
    inline leg_ik_result_t solve_leg_ik(
        const Eigen::Vector3d& hip,
        const Eigen::Vector3d& knee,
        const Eigen::Vector3d& ankle,
        const Eigen::Vector3d& foot,
        const Eigen::Vector3d& thigh_rest,
        const Eigen::Vector3d& shin_rest,
        const Eigen::Vector3d& foot_rest,
        const Eigen::Quaterniond& R_pelvis = Eigen::Quaterniond::Identity(),
        const std::optional<Eigen::Vector3d>& hinge_axis_world = std::nullopt)
    {
        leg_ik_result_t r;

        r.hip = swing_local(hip, knee, thigh_rest, R_pelvis, hinge_axis_world);
        const Eigen::Quaterniond R_hip_world = (R_pelvis * r.hip).normalized();

        r.knee = swing_local(knee, ankle, shin_rest, R_hip_world, hinge_axis_world);
        const Eigen::Quaterniond R_knee_world = (R_hip_world * r.knee).normalized();

        r.ankle = swing_local(ankle, foot, foot_rest, R_knee_world, hinge_axis_world);
        return r;
    }

} // namespace pose
