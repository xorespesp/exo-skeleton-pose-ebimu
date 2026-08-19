#pragma once
#include "pose_estimator_base.hh"
#include "joints_def.hh"
#include "joint_measurement.hh"
#include "dsp/one_euro_filter.hh"

#include "hw/timestamp.hh"
#include "utils/serializable.hh"

#include <Eigen/Geometry>

#include <chrono>
#include <memory>
#include <optional>
#include <span>

namespace pose
{
    // ---------------------------------------------------------------------------
    // Frontal-view estimator: rig-space joint points -> leg IK rotations
    // ---------------------------------------------------------------------------
    //
    // The camera faces the exo squarely, so its frame is the rig frame and measurements need no
    // reorientation. A joint's position is the measured point, smoothed and held through occlusion.
    // Measuring one at all takes a marker whose distance can be solved, which for a camera is what
    // camera intrinsics are for.
    // Once a rest pose is captured (any neutral stance), leg IK maps the position track to per-joint
    // local_anim_rot: the minimal-swing rotation of each bone from its rest direction, constrained
    // to a 1-DOF hinge (every exo leg joint is a forward/back hinge). local_anim_rot drives the
    // skeleton and is the value broadcast to clients.
    //
    // Angles follow the model laid out at the top of sagittal_pose_estimator.cc: geometry is
    // measured in the rig's hinge sign off the same position track, and `joint_state_t` receives
    // it in the biomechanics conventions of docs/joint_angle_convention.md (Segment / Clinical /
    // Included Angle, no rest pose involved) beside the rest-relative motion (`local_anim_rot`
    // and the delta angles).
    //
    // TODO: lens distortion is not corrected. Undistort the marker points before the pose solve (or
    // remap the frame) once markers sit near the image border or the lens gets wider.
    class frontal_pose_estimator final : public pose_estimator_base
    {
    public:
        struct options_t
        {
            // --- position track (rig-space 3D points) ---
            bool enable_position_smoothing{ true }; // One Euro per axis (hold still applies)
            dsp::one_euro_params_t position_filter{};

            // Joint occlusion policy (shared by the point track).
            millis_f64 max_hold{ 200.0 };  // hold a lost joint's last point up to this long (~6 frames @30fps)
            millis_f64 reset_gap{ 400.0 }; // beyond this gap, reseed the filter to the raw sample
            seconds_f64 dt_min{ 0.001 };   // dt clamp floor [s]
            seconds_f64 dt_max{ 0.100 };   // dt clamp ceiling [s] (avoids a jump after a long pause)

            // --- leg IK (1-DOF hinge) ---
            // Constrain every leg joint (hip/knee/ankle) to a single forward/back hinge about
            // `kRigLateralAxis`, dropping off-hinge components as tag-position error. When off,
            // each joint gets the free minimal-swing rotation.
            // NOTE: off, `local_anim_rot` keeps components the sagittal delta angles cannot see,
            //       and reading the bend back out of the rotation returns less than the deltas
            //       state.
            bool enable_hinge_constraint{ false };

            DECLARE_SERIALIZABLE_FIELDS(
                v("enable_position_smoothing", o.enable_position_smoothing);
                v("position_filter",           o.position_filter);
                v("max_hold_ms",               o.max_hold);
                v("reset_gap_ms",              o.reset_gap);
                v("dt_min_s",                  o.dt_min);
                v("dt_max_s",                  o.dt_max);
                v("enable_hinge_constraint",   o.enable_hinge_constraint);
            )
        };

        explicit frontal_pose_estimator(const options_t& opt = {});
        ~frontal_pose_estimator() override;

        options_t& options() noexcept { return _opt; }
        const options_t& options() const noexcept { return _opt; }

        // Ingest one frame's measurements and recompute every joint state.
        void update(
            std::span<const joint_3d_measurement_t> measurements,
            hw::timestamp_t sensor_timestamp // when the source captured the frame
        );

        // Latch the current per-joint 3D points as the rest (bind) reference.
        // Returns false if no joint had a point this frame.
        bool calibrate_rest_pose() override;
        void clear_rest_pose() override;
        bool has_rest_pose() const override;

        // Drop the position track: per-joint filters, occlusion timers, and held points. The next
        // frame starts cold (the filter reseeds to its raw sample). Call when the input stream
        // changes so a new source is not smoothed or held against the previous one's stale state.
        void reset_tracking() override;

        const joint_state_t& get_joint_state(joint_id_t j) const override;
        std::span<const joint_state_t> get_joint_states() const override;

        // Captured rest rig-space position of `j` (the bind position). Empty when uncalibrated or
        // the joint had no position at capture. Rest bone directions/lengths derive from these.
        std::optional<Eigen::Vector3d> get_rest_position(joint_id_t j) const override;

        bool uses_smoothed_positions() const override { return _opt.enable_position_smoothing; }

    private:
        struct context_t;

        options_t _opt;
        std::unique_ptr<context_t> _ctx;
    };

} // namespace pose
