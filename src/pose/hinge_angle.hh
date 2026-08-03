#pragma once
#include <Eigen/Geometry>

#include <cmath>
#include <numbers>

// Flexion measurement for the exo's lower limb. Every leg joint is a forward/back hinge about one
// shared lateral axis, so a joint's flexion is the turn its bone made within the plane perpendicular
// to that axis, taken relative to its parent bone's turn.
//
// The measurement runs on the bone directions, projected into that plane. The projection is what
// makes the reading a pure hinge quantity: a bone tilted along the axis (the thigh runs from the
// midline pelvis out to the knee) contributes only its in-plane part.
namespace pose
{
    // Fold an angle into [-pi, pi] so a difference of two angles does not jump by 2pi.
    inline double wrap_pi(double a)
    {
        constexpr double two_pi = 2.0 * std::numbers::pi;
        a = std::fmod(a + std::numbers::pi, two_pi);
        if (a < 0.0) { a += two_pi; }
        return a - std::numbers::pi;
    }

    // Signed turn from `from` to `to` about `axis`, in [-pi, pi]. Both directions are projected onto
    // the plane perpendicular to `axis` first, so their tilt along it stays out of the result.
    // Zero when either direction collapses onto the axis and leaves no in-plane part to measure.
    inline double hinge_plane_angle(
        const Eigen::Vector3d& from,
        const Eigen::Vector3d& to,
        const Eigen::Vector3d& axis)
    {
        constexpr double kMinNorm2 = 1e-18;

        const Eigen::Vector3d u = axis.normalized();
        const Eigen::Vector3d a = from - u * u.dot(from);
        const Eigen::Vector3d b = to - u * u.dot(to);
        if (a.squaredNorm() < kMinNorm2 || b.squaredNorm() < kMinNorm2) { return 0.0; }

        return std::atan2(u.dot(a.cross(b)), a.dot(b));
    }

    // The flexion a consumer recovers from a joint's rotation alone: the turn `q` carries about
    // `axis`. It equals the hinge angle when `q` turns purely about that axis, and shrinks in
    // proportion to how far `q`'s own axis tilts away from it.
    inline double quat_hinge_angle(const Eigen::Quaterniond& q, const Eigen::Vector3d& axis)
    {
        const Eigen::Vector3d u = axis.normalized();
        // q and -q name the same rotation. Pinning w >= 0 puts the half-angle in [-pi/2, pi/2], so
        // the sign below follows the axis component and not the hemisphere the quaternion arrived in.
        const double s = (q.w() < 0.0) ? -1.0 : 1.0;
        return wrap_pi(2.0 * std::atan2(s * q.vec().dot(u), s * q.w()));
    }

} // namespace pose
