#pragma once
#include <Eigen/Core>

#include <cmath>

namespace hw
{
    // Pinhole intrinsics. SDK-agnostic value type;
    // backends copy their native intrinsics (e.g. OBCameraIntrinsic, k4a) into this.
    struct intrinsic_t
    {
        float fx{ 0.0f }, fy{ 0.0f };  // focal length [px]
        float cx{ 0.0f }, cy{ 0.0f };  // principal point [px]

        // Resolution these values were calibrated at; they are only valid at this size.
        Eigen::Vector2i calib_resolution{ Eigen::Vector2i::Zero() }; // (width, height)

        // (h_fov, v_fov) [deg]: 2*atan(extent / (2*focal)).
        // Zero on an axis whose focal length is unset.
        Eigen::Vector2f get_fov() const
        {
            constexpr double kPi = 3.14159265358979323846;
            const auto deg = [](float focal, int extent) {
                return (focal > 0.0f)
                    ? static_cast<float>(2.0 * std::atan(extent / (2.0 * focal)) * 180.0 / kPi)
                    : 0.0f;
            };
            return Eigen::Vector2f{
                deg(fx, calib_resolution.x()),
                deg(fy, calib_resolution.y())
            };
        }
    };

    // Brown-Conrady (rational) distortion coefficients.
    //
    //   - k1, k2, k3      : radial (basic)
    //   - k4, k5, k6      : radial (rational denominator; 0 if unsupported)
    //   - p1, p2          : tangential
    struct distortion_t
    {
        float k1{ 0.0f }, k2{ 0.0f }, k3{ 0.0f };
        float k4{ 0.0f }, k5{ 0.0f }, k6{ 0.0f };
        float p1{ 0.0f }, p2{ 0.0f };
    };

    // Per-device calibration, filled once at source open.
    struct calibration_t
    {
        intrinsic_t  intrinsic;
        distortion_t distortion;

        // Size of the frames actually being delivered.
        // Its own field because a source knows this even when it has no intrinsics to report.
        Eigen::Vector2i frame_resolution{ Eigen::Vector2i::Zero() }; // (width, height)
    };

} // namespace hw
