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
    // Sagittal-view estimator: image-plane joint points -> in-plane joint angles
    // ---------------------------------------------------------------------------
    //
    // For a camera looking at the exo from the side, every leg joint flexes within the sagittal
    // plane, which projects onto the image plane. The angles are therefore recoverable from 2D joint
    // points alone: no marker->camera pose, no depth, no camera intrinsics. That makes this the cheap
    // path and immune to depth noise.
    //
    // Every rig joint comes out with a rotation, so a consumer always sees a whole skeleton and
    // never has to know how many legs the camera could actually see. Positions are a separate
    // matter: only measured joints get one, which keeps the raw-position views honest.
    //
    // Reaching that from one side is an internal concern. The marked leg follows from the joints
    // this frame measured, and the far leg (occluded by the near one) takes its rotations from its
    // mirror joints.
    //
    // A bone's angle is atan2 over its two endpoints; each joint's rotation is that angle's change
    // since the captured rest, taken relative to its parent bone's change. Angles are computed on
    // pixel coordinates, which is scale free, so the metric approximation below cannot leak in.
    //
    // Output is expressed in the rig frame, not the camera's. A side view looks along the rig's
    // lateral axis, so the image plane holds the rig's sagittal plane: flexion becomes a rotation
    // about the rig's lateral axis and measured points land on the mid-sagittal plane (X = 0).
    // Which side the camera views from decides the sign of both, and the tagged leg gives it away.
    //
    // `joint_state_t::position` is reported in approximate meters: the scales the measurements
    // supply are averaged into one meters-per-pixel factor for the whole rig. This keeps one unit
    // convention across estimators for the plots and the diagnostic trace, and being a single factor
    // it sets the skeleton's size without touching its shape.
    //
    // TODO: lens distortion is not corrected. Undistort the joint points before the angle math once
    // markers sit near the image border or the lens gets wider.
    class sagittal_pose_estimator final : public pose_estimator_base
    {
    public:
        // Per-joint flexion relative to the rest pose, as measured in the image plane. These are the
        // raw in-plane readings: the sign that carries them into the rig frame is applied separately,
        // so a value here may run opposite to the rotation the same joint reports.
        struct leg_angles_t
        {
            double hip{ 0.0 };   // thigh swing [rad]
            double knee{ 0.0 };  // shin vs thigh [rad]
            double ankle{ 0.0 }; // foot vs shin [rad]
        };

        struct options_t
        {
            // --- position track (image-plane points) ---
            // Smoothing and holding run on pixel coordinates, so the angles read off them are
            // smoothed too.
            bool enable_position_smoothing{ true }; // One Euro per image axis (hold still applies)
            dsp::one_euro_params_t position_filter{};

            // Joint occlusion policy (shared by the point track).
            millis_f64 max_hold{ 200.0 };  // hold a lost joint's last point up to this long (~6 frames @30fps)
            millis_f64 reset_gap{ 400.0 }; // beyond this gap, reseed the filter to the raw sample
            seconds_f64 dt_min{ 0.001 };   // dt clamp floor [s]
            seconds_f64 dt_max{ 0.100 };   // dt clamp ceiling [s] (avoids a jump after a long pause)

            DECLARE_SERIALIZABLE_FIELDS(
                v("enable_position_smoothing", o.enable_position_smoothing);
                v("position_filter",           o.position_filter);
                v("max_hold_ms",               o.max_hold);
                v("reset_gap_ms",              o.reset_gap);
                v("dt_min_s",                  o.dt_min);
                v("dt_max_s",                  o.dt_max);
            )
        };

        explicit sagittal_pose_estimator(const options_t& opt = {});
        ~sagittal_pose_estimator() override;

        options_t& options() noexcept { return _opt; }
        const options_t& options() const noexcept { return _opt; }

        // Ingest one frame's measurements and recompute every joint state.
        // NOTE: the metric scale rides on the measurements and reaches the reported positions only.
        //       The angles this estimator produces do not depend on it.
        void update(
            std::span<const joint_2d_measurement_t> measurements,
            hw::timestamp_t sensor_timestamp // when the source captured the frame
        );

        // Latch the current per-joint image-plane points as the rest (bind) reference. Bone angles
        // are measured against it, so no rotation is produced until it is captured.
        // Returns false if no joint had a point this frame.
        bool calibrate_rest_pose() override;
        void clear_rest_pose() override;
        bool has_rest_pose() const override;

        // Drop the position track: per-joint filters, occlusion timers, held points, and the tracked
        // near-leg side. The next frame starts cold (the filter reseeds to its raw sample) and the
        // side is read again. Call when the input stream changes so a new source is not smoothed or
        // held against the previous one's stale state.
        void reset_tracking() override;

        const joint_state_t& get_joint_state(joint_id_t j) const override;
        std::span<const joint_state_t> get_joint_states() const override;

        // Captured rest position of `j` in the same rig-space meters as `joint_state_t::position`,
        // sharing this frame's metric scale so the two can be drawn together.
        // Empty when uncalibrated or the joint had no point at capture.
        std::optional<Eigen::Vector3d> get_rest_position(joint_id_t j) const override;

        bool uses_smoothed_positions() const override { return _opt.enable_position_smoothing; }

        // Angles are measured in the image plane, which the rig meets at its lateral axis.
        Eigen::Vector3d hinge_axis() const override { return Eigen::Vector3d::UnitX(); }

        // --- sagittal specifics -----------------------------------------------------------

        // Knee joint of the leg the measurements came from; 
        // empty until a leg joint has been measured. 
        // Its `get_joint_name()` identifies the side for an operator.
        std::optional<joint_id_t> tracked_leg_knee() const;

        // This frame's joint angles for the tracked leg. Empty until a rest pose is captured and the
        // whole chain is visible.
        std::optional<leg_angles_t> leg_angles() const;

    private:
        // +1 or -1, applied wherever the image plane is carried into rig space.
        double _side() const noexcept;

        struct context_t;

        options_t _opt;
        std::unique_ptr<context_t> _ctx;
    };

} // namespace pose
