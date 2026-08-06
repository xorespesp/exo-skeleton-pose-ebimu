#pragma once
#include "joints_def.hh"

#include <Eigen/Core>

#include <optional>

namespace pose
{
    // ---------------------------------------------------------------------------
    // Estimator input: one frame's per-joint measurements
    // ---------------------------------------------------------------------------
    //
    // A measurement names the joint it belongs to, so an estimator never resolves identity itself.
    // Whoever holds the detections does that: 
    // a tag carries an id, a color blob is placed by its position among its neighbours.
    //
    // One shape per estimator dimensionality. Neither carries orientation: every rig rotation this
    // project reports is solved from positions (`joint_state_t::local_anim_rot`).

    // Image-plane measurement, for an estimator that reads angles off pixels (sagittal).
    struct joint_2d_measurement_t
    {
        joint_id_t joint_id{ joint_id_t::pelvis };
        Eigen::Vector2d center_px{ Eigen::Vector2d::Zero() };

        // Metric scale this one measurement supplies:
        // the marker's printed size over its apparent size in pixels. 
        // An estimator averages what a frame supplies and reports positions in approximate meters. 
        // Empty when the marker's apparent size was not measurable, which
        // leaves the center usable and only withholds a vote on the scale.
        std::optional<double> meters_per_pixel{};
    };

    // Rig-space position measurement [m], for an estimator that works in 3D (frontal).
    struct joint_3d_measurement_t
    {
        joint_id_t joint_id{ joint_id_t::pelvis };
        Eigen::Vector3d position{ Eigen::Vector3d::Zero() };
    };

} // namespace pose
