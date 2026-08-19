#include "frontal_pose_estimator.hh"
#include "hinge_angle.hh"
#include "leg_ik.hh"

#include <algorithm>
#include <array>

namespace pose
{
    namespace
    {
        size_t index_of(joint_id_t jid) { return static_cast<size_t>(jid); }

        // Rig-sign segment angle of a bone direction (angle model: top of
        // sagittal_pose_estimator.cc): signed turn from the rig's down axis, backward swing positive.
        double rig_sagittal_segment_angle_from_dir(const Eigen::Vector3d& bone_dir)
        {
            return hinge_plane_angle(
                pose_estimator_base::kRigDownAxis, bone_dir, pose_estimator_base::kRigLateralAxis);
        }

        // The same angle off a bone's two rig-space endpoints. Empty for a missing endpoint, and
        // for coincident ones, which leave no direction to measure.
        std::optional<double> rig_sagittal_segment_angle_from_pos(
            const std::optional<Eigen::Vector3d>& start,
            const std::optional<Eigen::Vector3d>& end)
        {
            constexpr double kMinBoneLengthSq_m2 = 1e-8; // coincident-endpoint gate [m^2]

            if (!start.has_value() || !end.has_value()) { return std::nullopt; }
            const Eigen::Vector3d d = end.value() - start.value();
            if (d.squaredNorm() < kMinBoneLengthSq_m2) { return std::nullopt; }
            return rig_sagittal_segment_angle_from_dir(d);
        }
    } // namespace

    struct frontal_pose_estimator::context_t
    {
        // Per-joint position filter + occlusion timers, persisting across frames. One Euro per axis.
        struct joint_filter_state_t
        {
            std::array<dsp::OneEuroFilter, 3> pos_smoother{};
            std::optional<Eigen::Vector3d> last_pos_out; // last smoothed position (hold output)
            hw::timestamp_t last_seen{}; // time of the last fresh detection (hold origin)
            hw::timestamp_t last_step_time{}; // time of the last fresh filter step (dt source)
        };

        std::array<joint_filter_state_t, kNumJoints> filter_states{}; // persists across frames
        std::array<joint_state_t, kNumJoints> last_frame_joint_states{}; // per-frame output; reset every update()
        std::array<std::optional<Eigen::Vector3d>, kNumJoints> last_frame_raw_positions{}; // per-frame raw detected positions
        std::array<bool, kNumJoints> last_frame_detection_flags{}; // per-joint fresh-detection flag

        // The captured rest (bind) reference. Rest bone directions and lengths for the leg IK
        // derive from these positions.
        struct rest_pose_info_t
        {
            // Per-joint rig-space position at capture; empty for a joint that was not measured then.
            std::array<std::optional<Eigen::Vector3d>, kNumJoints> joint_position{};
        };
        std::optional<rest_pose_info_t> rest_pose; // empty until calibrated
    };

    frontal_pose_estimator::frontal_pose_estimator(const options_t& opt)
        : _opt{ opt }
        , _ctx{ std::make_unique<context_t>() }
    { }

    frontal_pose_estimator::~frontal_pose_estimator() = default;

    const joint_state_t& frontal_pose_estimator::get_joint_state(joint_id_t j) const
    {
        return _ctx->last_frame_joint_states[index_of(j)];
    }

    std::span<const joint_state_t> frontal_pose_estimator::get_joint_states() const
    {
        return _ctx->last_frame_joint_states;
    }

    bool frontal_pose_estimator::has_rest_pose() const
    {
        return _ctx->rest_pose.has_value();
    }

    std::optional<Eigen::Vector3d> frontal_pose_estimator::get_rest_position(joint_id_t j) const
    {
        if (_ctx->rest_pose.has_value()) {
            return _ctx->rest_pose->joint_position[index_of(j)];
        }
        return std::nullopt;
    }

    void frontal_pose_estimator::update(
        const std::span<const joint_3d_measurement_t> measurements,
        const hw::timestamp_t sensor_timestamp)
    {
        const auto t = sensor_timestamp;

        // Reset the per-frame output; filter_states (filters/timers) persist.
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_raw_positions = {};
        _ctx->last_frame_detection_flags = {};

        // ----- Pass 1: take this frame's rig-space points -----
        for (const auto& m : measurements)
        {
            const size_t i = index_of(m.joint_id);
            _ctx->last_frame_raw_positions[i] = m.position;
            _ctx->last_frame_detection_flags[i] = true;
            _ctx->last_frame_joint_states[i].raw_position = m.position; // raw
        }

        // ----- Pass 2: smooth + hold each joint's rig-space position -----
        for (const auto& def : get_joint_defs())
        {
            const size_t i = index_of(def.joint_id);
            auto& fs = _ctx->filter_states[i];
            joint_state_t& st = _ctx->last_frame_joint_states[i];

            if (_ctx->last_frame_raw_positions[i].has_value())
            {
                const Eigen::Vector3d p = _ctx->last_frame_raw_positions[i].value();
                for (auto& lpf : fs.pos_smoother) {
                    lpf.set_min_cutoff(_opt.position_filter.min_cutoff_hz);
                    lpf.set_beta(_opt.position_filter.beta);
                    lpf.set_derivate_cutoff(_opt.position_filter.dcutoff_hz);
                }

                // Reseed on cold start / long gap / smoothing off; else low-pass each axis.
                if (!_opt.enable_position_smoothing
                    || !fs.last_pos_out.has_value()
                    || (t - fs.last_seen) > _opt.reset_gap)
                {
                    for (auto& lpf : fs.pos_smoother) { lpf.reset(); }
                    fs.last_pos_out = p;
                }
                else
                {
                    const seconds_f64 dt = std::clamp(seconds_f64{ t - fs.last_step_time }, _opt.dt_min, _opt.dt_max);
                    Eigen::Vector3d out;
                    for (int a = 0; a < 3; ++a) { out[a] = fs.pos_smoother[a].filter(p[a], dt.count()); }
                    fs.last_pos_out = out;
                }
                fs.last_step_time = t;
                fs.last_seen = t;
                st.position = fs.last_pos_out;
            }
            else if (fs.last_pos_out.has_value() && (t - fs.last_seen) <= _opt.max_hold)
            {
                st.position = fs.last_pos_out; // no fresh detection: hold within the window
            }
        }

        // ----- Pass 3: bone directions -> published angles + rest-relative motion + leg IK -----
        // The angle model is laid out at the top of sagittal_pose_estimator.cc: geometry in the
        // rig's hinge sign, published in the conventions of docs/joint_angle_convention.md at
        // the `joint_state_t` boundary. Here the bone directions come straight from rig-space
        // positions, and `hinge_plane_angle()` projects each onto the plane perpendicular to
        // `kRigLateralAxis`, so a bone's lateral lean (the thigh runs from the hip point on the
        // midline pelvis marker out to the knee) stays out of every reading. Data driven over
        // `get_joint_defs()`: a leg is (hip = child of root, knee = child of hip, ankle = child
        // of knee, foot = child of ankle).
        {
            auto& states = _ctx->last_frame_joint_states;
            const auto position_of = [&states](joint_id_t j) { return states[index_of(j)].position; };
            const joint_id_t root = get_root_joint();

            // The root's bone is the torso, which stands along the reference axis, so its
            // segment angle is 0 by definition, with no measurement involved. It has no parent
            // bone, hence no clinical angle.
            states[index_of(root)].sagittal_segment_angle = 0.0;

            // Pelvis on the rest-relative motion: identity animation, unmoved torso.
            if (_ctx->rest_pose.has_value())
            {
                states[index_of(root)].local_anim_rot = Eigen::Quaterniond::Identity();
                states[index_of(root)].sagittal_segment_angle_delta = 0.0;
            }

            const auto hinge_axis = _opt.enable_hinge_constraint
                ? std::make_optional(kRigLateralAxis) : std::nullopt;

            for (const auto& hip_def : get_joint_defs())
            {
                if (is_root_joint(hip_def.joint_id) || hip_def.parent != root) { continue; } // hip = child of root
                const joint_id_t hip = hip_def.joint_id;
                const std::optional<joint_id_t> knee = get_child_joint(hip);
                const std::optional<joint_id_t> ankle = knee.has_value() ? get_child_joint(knee.value()) : std::nullopt;
                const std::optional<joint_id_t> foot = ankle.has_value() ? get_child_joint(ankle.value()) : std::nullopt;
                if (!knee.has_value() || !ankle.has_value() || !foot.has_value()) { continue; }

                const auto hip_pos = position_of(hip), knee_pos = position_of(knee.value());
                const auto ankle_pos = position_of(ankle.value()), foot_pos = position_of(foot.value());

                // --- this frame's rig-sign segment angles, one per bone of this leg ---
                // The hip point rides the shared pelvis tag, so the thigh runs from it. Each is
                // empty where an endpoint is missing, and every output below asks for exactly
                // the segments it is defined on, so one lost tag costs only the angles built on
                // it. The foot joint is a marker end site: it lends the foot bone its far
                // endpoint and carries no angles of its own.
                const std::optional<double> rig_sagittal_segment_ang_thigh = rig_sagittal_segment_angle_from_pos(hip_pos, knee_pos);
                const std::optional<double> rig_sagittal_segment_ang_shin  = rig_sagittal_segment_angle_from_pos(knee_pos, ankle_pos);
                const std::optional<double> rig_sagittal_segment_ang_foot  = rig_sagittal_segment_angle_from_pos(ankle_pos, foot_pos);

                // --- published measured angles (no rest pose involved) ---
                // The per-joint bend is the difference of adjacent rig segment angles (the hip's
                // parent bone is the torso on the reference axis at 0), carried into the
                // document conventions at this boundary; every per-joint sign and neutral offset
                // lives in the joints_def.hh table.
                if (rig_sagittal_segment_ang_thigh.has_value()) {
                    const std::optional<double> hip_clinical =
                        sagittal_clinical_angle_from_rig_bend(hip, rig_sagittal_segment_ang_thigh.value());
                    states[index_of(hip)].sagittal_segment_angle = sagittal_segment_angle_from_rig(rig_sagittal_segment_ang_thigh.value());
                    states[index_of(hip)].sagittal_clinical_angle = hip_clinical;
                    states[index_of(hip)].sagittal_included_angle = hip_clinical.has_value()
                        ? sagittal_included_angle_from_clinical(hip, hip_clinical.value())
                        : std::nullopt;
                }
                if (rig_sagittal_segment_ang_shin.has_value()) {
                    states[index_of(knee.value())].sagittal_segment_angle = sagittal_segment_angle_from_rig(rig_sagittal_segment_ang_shin.value());
                    if (rig_sagittal_segment_ang_thigh.has_value()) {
                        const std::optional<double> knee_clinical = sagittal_clinical_angle_from_rig_bend(knee.value(),
                            wrap_pi(rig_sagittal_segment_ang_shin.value() - rig_sagittal_segment_ang_thigh.value()));
                        states[index_of(knee.value())].sagittal_clinical_angle = knee_clinical;
                        states[index_of(knee.value())].sagittal_included_angle = knee_clinical.has_value()
                            ? sagittal_included_angle_from_clinical(knee.value(), knee_clinical.value())
                            : std::nullopt;
                    }
                }
                if (rig_sagittal_segment_ang_foot.has_value()) {
                    states[index_of(ankle.value())].sagittal_segment_angle = sagittal_segment_angle_from_rig(rig_sagittal_segment_ang_foot.value());
                    if (rig_sagittal_segment_ang_shin.has_value()) {
                        const std::optional<double> ankle_clinical = sagittal_clinical_angle_from_rig_bend(ankle.value(),
                            wrap_pi(rig_sagittal_segment_ang_foot.value() - rig_sagittal_segment_ang_shin.value()));
                        states[index_of(ankle.value())].sagittal_clinical_angle = ankle_clinical;
                        states[index_of(ankle.value())].sagittal_included_angle = ankle_clinical.has_value()
                            ? sagittal_included_angle_from_clinical(ankle.value(), ankle_clinical.value())
                            : std::nullopt;
                    }
                }

                // --- rest-relative motion + leg IK (needs the captured rest bone directions) ---
                // Each joint is a 1-DOF forward/back hinge about the shared lateral axis when the
                // hinge constraint is on. The whole chain is required on both ends: a rig driven
                // by these rotations needs all of it to describe one pose.
                if (!hip_pos || !knee_pos || !ankle_pos || !foot_pos) { continue; }
                if (!_ctx->rest_pose.has_value()) { continue; }
                const auto& rest = _ctx->rest_pose->joint_position;
                const auto& r_hip = rest[index_of(hip)];
                const auto& r_knee = rest[index_of(knee.value())];
                const auto& r_ankle = rest[index_of(ankle.value())];
                const auto& r_foot = rest[index_of(foot.value())];
                if (!r_hip || !r_knee || !r_ankle || !r_foot) { continue; }

                const Eigen::Vector3d thigh_rest = ik_normalize(r_knee.value() - r_hip.value());
                const Eigen::Vector3d shin_rest = ik_normalize(r_ankle.value() - r_knee.value());
                const Eigen::Vector3d foot_rest = ik_normalize(r_foot.value() - r_ankle.value());

                const leg_ik_result_t r = solve_leg_ik(
                    hip_pos.value(), knee_pos.value(), ankle_pos.value(), foot_pos.value(),
                    thigh_rest, shin_rest, foot_rest,
                    Eigen::Quaterniond::Identity(),
                    hinge_axis
                );

                // Each joint drives the bone that leaves it: hip the thigh, knee the shin, ankle
                // the foot bone.
                states[index_of(hip)].local_anim_rot = r.hip;
                states[index_of(knee.value())].local_anim_rot = r.knee;
                states[index_of(ankle.value())].local_anim_rot = r.ankle;

                // The change of each rig-sign segment angle since the captured rest, then the
                // same subtraction cascade on the changes for the per-joint bends; the reference
                // axis cancels in each change. Taken from the measured positions, so the reading
                // follows the observed bone directions, while the rotation above is whatever the
                // IK solved. All three bones are required, the deltas being one joint's motion
                // alongside the rotation.
                if (!rig_sagittal_segment_ang_thigh.has_value()
                    || !rig_sagittal_segment_ang_shin.has_value()
                    || !rig_sagittal_segment_ang_foot.has_value())
                {
                    continue;
                }

                const double rig_sagittal_segment_delta_thigh = wrap_pi(rig_sagittal_segment_ang_thigh.value() - rig_sagittal_segment_angle_from_dir(thigh_rest));
                const double rig_sagittal_segment_delta_shin  = wrap_pi(rig_sagittal_segment_ang_shin.value()  - rig_sagittal_segment_angle_from_dir(shin_rest));
                const double rig_sagittal_segment_delta_foot  = wrap_pi(rig_sagittal_segment_ang_foot.value()  - rig_sagittal_segment_angle_from_dir(foot_rest));

                states[index_of(hip)].sagittal_segment_angle_delta = sagittal_segment_angle_from_rig(rig_sagittal_segment_delta_thigh);
                states[index_of(knee.value())].sagittal_segment_angle_delta = sagittal_segment_angle_from_rig(rig_sagittal_segment_delta_shin);
                states[index_of(ankle.value())].sagittal_segment_angle_delta = sagittal_segment_angle_from_rig(rig_sagittal_segment_delta_foot);

                states[index_of(hip)].sagittal_clinical_angle_delta =
                    sagittal_clinical_angle_delta_from_rig_bend_delta(hip, rig_sagittal_segment_delta_thigh);
                states[index_of(knee.value())].sagittal_clinical_angle_delta = sagittal_clinical_angle_delta_from_rig_bend_delta(
                    knee.value(), wrap_pi(rig_sagittal_segment_delta_shin - rig_sagittal_segment_delta_thigh));
                states[index_of(ankle.value())].sagittal_clinical_angle_delta = sagittal_clinical_angle_delta_from_rig_bend_delta(
                    ankle.value(), wrap_pi(rig_sagittal_segment_delta_foot - rig_sagittal_segment_delta_shin));
            }
        }
    }

    bool frontal_pose_estimator::calibrate_rest_pose()
    {
        context_t::rest_pose_info_t new_rest{};

        bool any = false;
        for (size_t i = 0; i < kNumJoints; ++i) {
            // Only latch freshly detected joints; a held position is a stale reference.
            if (_ctx->last_frame_detection_flags[i]) {
                new_rest.joint_position[i] = _ctx->last_frame_joint_states[i].raw_position;
            }
            any = any || new_rest.joint_position[i].has_value();
        }

        if (any) { _ctx->rest_pose = new_rest; }
        else { _ctx->rest_pose.reset(); }

        return any;
    }

    void frontal_pose_estimator::clear_rest_pose()
    {
        _ctx->rest_pose.reset();
    }

    void frontal_pose_estimator::reset_tracking()
    {
        // Fresh filters/timers and no held positions, so the first frame of the next stream reseeds
        // rather than filtering or holding against a prior stream's position and timestamps.
        _ctx->filter_states = {};
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_raw_positions = {};
        _ctx->last_frame_detection_flags = {};
    }

} // namespace pose
