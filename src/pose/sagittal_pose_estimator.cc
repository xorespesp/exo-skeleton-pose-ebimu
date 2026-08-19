#include "sagittal_pose_estimator.hh"
#include "hinge_angle.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_set>

namespace pose
{
    // ---------------------------------------------------------------------------
    // Angle model: rig-sign geometry, published in the document conventions
    // ---------------------------------------------------------------------------
    //
    // All geometry in this file is measured in ONE sign system, the rig hinge sign:
    //
    //   rig segment angle   A bone's signed turn from the rig's down axis (`kRigDownAxis`) to
    //                       its direction, about the rig's lateral axis (`kRigLateralAxis`).
    //                       A bone hanging straight down reads 0; its far end swung toward the
    //                       exo's back (rig +Z) reads positive, toward its front negative.
    //
    //   rig bend            The bend at a joint: the turn of the bone leaving it away from the
    //                       bone entering it, 0 when the two are collinear. Within one plane the
    //                       angle between two directions is the difference of the angles each
    //                       makes with any common reference, so a bend is a difference of
    //                       adjacent rig segment angles:
    //
    //                         (down -> shin) - (down -> thigh) = (thigh -> shin)
    //
    // With the bones of one leg (thigh = hip->knee, shin = knee->ankle, foot bone = ankle->foot;
    // the hip is co-sited on the pelvis marker):
    //
    //   bend at the hip   = seg(thigh)
    //   bend at the knee  = seg(shin) - seg(thigh)
    //   bend at the ankle = seg(foot) - seg(shin)
    //   foot              = marker end site; no bone leaves it, so no bend
    //
    // The hip's parent bone is the torso, which carries no marker. The exo stands upright on its
    // fixed frame, so the torso runs along the rig's down axis: the reference direction IS the
    // torso segment at 0, and the thigh's segment angle is the hip's bend outright.
    //
    // One sign system for all geometry keeps this file free of per-joint sign choices: every
    // bend is the same subtraction, and `local_anim_rot` is a rest-relative bend put straight
    // onto `kRigLateralAxis` with no sign work at all. The outputs are then produced at the
    // `joint_state_t` boundary:
    //
    //   sagittal_segment_angle / sagittal_clinical_angle / sagittal_included_angle
    //     The biomechanics conventions of docs/joint_angle_convention.md, carried over from the
    //     rig-sign values by the conversion functions beside the rig table (joints_def.hh),
    //     which own every per-joint sign and neutral offset; the included angle is filled from
    //     the clinical one in the same statement block. No captured rest enters: these state the
    //     leg's geometry itself, which is what an external validation compares with the exo's
    //     own joint encoders.
    //
    //   local_anim_rot / sagittal_clinical_angle_delta / sagittal_segment_angle_delta
    //     The change since the captured rest pose,
    //
    //       delta(bone) = wrap_pi(rig_segment_now(bone) - rig_segment_rest(bone))
    //
    //     with the same subtraction cascade on the deltas for the per-joint bends. Zero while
    //     the exo holds the captured pose, which is what drives a rig from its bind pose. The
    //     reference axis cancels in every delta, and so do the conventions' neutral offsets,
    //     leaving only the clinical sign between the delta fields and the rotation.
    //
    // Every angle difference goes through `wrap_pi()`, so no reading jumps by 2*pi.

    namespace
    {
        size_t index_of(joint_id_t jid) { return static_cast<size_t>(jid); }

        // Entry point of the tracked leg's chain (its hip), once a side has been settled on.
        std::optional<joint_id_t> hip_of_side(std::optional<joint_side_t> side) {
            if (!side.has_value()) { return std::nullopt; }
            return get_leg_root_joint(side.value());
        }

        // A side view's camera frame is not the rig frame: the camera looks along the rig's lateral
        // axis, so what the image plane holds is the rig's sagittal (Y-Z) plane. The exo's left lies
        // at positive rig X, so a camera on that side maps
        //
        //   camera Z -> rig -X   (looking at the exo from its left)
        //   image y  -> rig Y    (down)
        //   image x  -> rig Z    (behind the exo; the remaining right-handed axis)
        //
        // Flexion is therefore a rotation about the rig's lateral axis, and an image-plane point
        // lands on the rig's mid-sagittal plane (X = 0). Viewing from the right mirrors the image,
        // reversing front-to-back and the swing together, which is what `side` (+1 left, -1 right)
        // carries.

        // Image-plane point as an approximate rig-space position, flat on the sagittal plane.
        Eigen::Vector3d to_rig_space(const Eigen::Vector2d& px, double meters_per_pixel, double side) {
            return Eigen::Vector3d{ 0.0, px.y() * meters_per_pixel, side * px.x() * meters_per_pixel };
        }

        // Direction of the bone from `start` to `end`, lifted from the image plane into rig
        // space: the axis mapping of `to_rig_space()` with the metric scale left out, since a
        // direction has no length to convert. This is what keeps every angle below scale free.
        // Positions and directions sharing one mapping puts the camera-side handedness in a
        // single seam, so the two cannot fall out of step.
        Eigen::Vector3d lift_image_direction(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double side) {
            const Eigen::Vector2d d = end - start;
            return Eigen::Vector3d{ 0.0, d.y(), side * d.x() };
        }

        // Rig-sign segment angle of the bone from `start` to `end`, read off image-plane points
        // (see the angle model atop this file): the signed turn from the rig's down axis to the
        // bone's direction, about the rig's lateral axis, backward swing positive. Empty when an
        // endpoint is missing, and when the two coincide, since a zero-length bone has no
        // direction to measure. Conversion into the published conventions happens where
        // `joint_state_t` is filled.
        std::optional<double> rig_sagittal_segment_angle_from_px(
            const std::optional<Eigen::Vector2d>& start,
            const std::optional<Eigen::Vector2d>& end,
            double side)
        {
            constexpr double kMinBoneLengthSq_px = 1e-6; // coincident-endpoint gate [px^2]

            if (!start.has_value() || !end.has_value()) { return std::nullopt; }
            const Eigen::Vector3d dir = lift_image_direction(start.value(), end.value(), side);
            if (dir.squaredNorm() < kMinBoneLengthSq_px) { return std::nullopt; }
            return hinge_plane_angle(
                pose_estimator_base::kRigDownAxis, dir, pose_estimator_base::kRigLateralAxis);
        }

    } // namespace

    struct sagittal_pose_estimator::context_t
    {
        // Per-joint image-plane filter + occlusion timers, persisting across frames. One Euro per
        // image axis. Smoothing runs on pixels so the angles read off them are smoothed too.
        struct joint_filter_state_t
        {
            std::array<dsp::OneEuroFilter, 2> px_smoother{};
            std::optional<Eigen::Vector2d> last_px_out; // last smoothed point (hold output)
            hw::timestamp_t last_seen{}; // time of the last fresh detection (hold origin)
            hw::timestamp_t last_step_time{}; // time of the last fresh filter step (dt source)
        };

        std::array<joint_filter_state_t, kNumJoints> filter_states{}; // persists across frames
        std::array<joint_state_t, kNumJoints> last_frame_joint_states{}; // per-frame output; reset every update()
        std::array<std::optional<Eigen::Vector2d>, kNumJoints> last_frame_raw_px{}; // per-frame measured points
        std::array<std::optional<Eigen::Vector2d>, kNumJoints> last_frame_px{}; // smoothed + held centers (angle source)
        std::array<bool, kNumJoints> last_frame_detection_flags{}; // per-joint fresh-detection flag

        // One conversion factor for the whole rig, averaged over this frame's measurements and held
        // while none arrive. A per-joint factor would stretch each point away from the image origin
        // by a different amount, skewing the skeleton's shape; a single one keeps the image geometry
        // and only sets its size.
        double meters_per_pixel{ 0.0 };

        // The captured rest (bind) reference. Kept in pixels: that is what the rest bone angles
        // derive from, and it makes them independent of how far the exo stood at capture.
        struct rest_pose_info_t
        {
            // Per-joint image-plane point at capture; empty for a joint that was not measured then.
            std::array<std::optional<Eigen::Vector2d>, kNumJoints> joint_px{};

            // The leg those points name, so a rest travels with the leg it describes.
            std::optional<joint_side_t> side{};
        };
        std::optional<rest_pose_info_t> rest_pose; // empty until calibrated

        // Which leg is being seen, and with it which way the camera faces (the sign that carries the
        // image plane into rig space). Held across frames so a frame that detects nothing keeps the
        // last answer.
        std::optional<joint_side_t> tracked_side;
    };

    sagittal_pose_estimator::sagittal_pose_estimator(const options_t& opt)
        : _opt{ opt }
        , _ctx{ std::make_unique<context_t>() }
    { }

    // Which side the camera views from follows from the marked leg, since the far leg is hidden
    // behind the near one. Before any leg joint has been measured the left-hand mapping is
    // assumed; nothing downstream depends on it yet, because every angle needs the tracked leg's
    // points, and any frame that supplies those (fresh or held) has settled the side in pass 1.
    double sagittal_pose_estimator::_side() const noexcept
    {
        return (_ctx->tracked_side == joint_side_t::right) ? -1.0 : 1.0;
    }

    sagittal_pose_estimator::~sagittal_pose_estimator() = default;

    const joint_state_t& sagittal_pose_estimator::get_joint_state(joint_id_t j) const
    {
        return _ctx->last_frame_joint_states[index_of(j)];
    }

    std::span<const joint_state_t> sagittal_pose_estimator::get_joint_states() const
    {
        return _ctx->last_frame_joint_states;
    }

    bool sagittal_pose_estimator::has_rest_pose() const
    {
        if (!_ctx->rest_pose.has_value()) { return false; }

        // The points name one leg's joints, so a rest taken on the other measures nothing here.
        // An undecided side on either end is not a disagreement.
        const std::optional<joint_side_t>& captured = _ctx->rest_pose->side;
        return !captured.has_value()
            || !_ctx->tracked_side.has_value()
            || captured == _ctx->tracked_side;
    }

    // Converted with the current frame's factor rather than the one at capture, so a rest skeleton
    // drawn next to the measured one shares its size and only their angles can differ.
    std::optional<Eigen::Vector3d> sagittal_pose_estimator::get_rest_position(joint_id_t j) const
    {
        if (!this->has_rest_pose()) { return std::nullopt; }
        const auto& px = _ctx->rest_pose->joint_px[index_of(j)];
        if (!px.has_value()) { return std::nullopt; }
        return to_rig_space(px.value(), _ctx->meters_per_pixel, this->_side());
    }

    std::optional<joint_id_t> sagittal_pose_estimator::tracked_leg_knee() const
    {
        // The knee rather than the hip: the hip sits on the shared pelvis tag, so the knee's tag
        // is the first one that identifies the leg to an operator.
        const std::optional<joint_id_t> hip = hip_of_side(_ctx->tracked_side);
        if (!hip.has_value()) { return std::nullopt; }
        return get_child_joint(hip.value());
    }

    void sagittal_pose_estimator::update(
        const std::span<const joint_2d_measurement_t> measurements,
        const hw::timestamp_t sensor_timestamp)
    {
        const auto t = sensor_timestamp;

        // Reset the per-frame output; filter_states and the tracked side persist.
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_raw_px = {};
        _ctx->last_frame_px = {};
        _ctx->last_frame_detection_flags = {};

        // ----- Pass 1: which leg is in view -----
        // Only one leg is marked, so counting the sides of this frame's joints answers it outright.
        // The hips are co-sited on the shared pelvis tag and arrive as a left/right pair, adding
        // one to each count, so only the leg's own tags decide. A tie means nothing decisive was
        // seen (nothing measured at all, or only the shared and midline points), and the previous
        // answer stands.
        {
            int right_seen = 0, left_seen = 0;
            for (const auto& m : measurements) {
                const auto side = get_joint_side(m.joint_id);
                if (side == joint_side_t::right)     { ++right_seen; }
                else if (side == joint_side_t::left) { ++left_seen; }
                // midline: belongs to neither leg
            }

            if (right_seen != left_seen) {
                _ctx->tracked_side = (right_seen > left_seen) ? joint_side_t::right : joint_side_t::left;
            }
        }

        // ----- Pass 2: take this frame's image-plane points and metric scale -----
        // The scale is averaged with one vote per physical marker: the rig sits at one distance
        // from a side camera, so one factor describes it and the spread between markers is
        // measurement noise rather than depth. Joints co-sited on one marker (the pelvis marker
        // carries the pelvis and both hips) arrive as one measurement each, all repeating that
        // marker's scale, so a vote is keyed by the tag the joint is bound to; the shared tag_id
        // in the rig table is the very datum that makes joints co-sited, so the keying follows
        // any change to the co-siting on its own.
        double scale_sum = 0.0;
        int scale_count = 0;
        std::unordered_set<int> scale_voted_tags;
        for (const auto& m : measurements)
        {
            const size_t i = index_of(m.joint_id);
            _ctx->last_frame_raw_px[i] = m.center_px;
            _ctx->last_frame_detection_flags[i] = true;

            const std::optional<joint_definition_t> def = get_joint_def(m.joint_id);
            if (m.meters_per_pixel.has_value()
                && def.has_value()
                && scale_voted_tags.insert(def->tag_id).second)
            {
                scale_sum += m.meters_per_pixel.value();
                ++scale_count;
            }
        }
        // Nothing measured this frame: keep the last factor so held points stay where they were.
        if (scale_count > 0) {
            _ctx->meters_per_pixel = scale_sum / scale_count;
        }

        // ----- Pass 3: smooth + hold each joint's image-plane point -----
        for (const auto& def : get_joint_defs())
        {
            const size_t i = index_of(def.joint_id);
            auto& fs = _ctx->filter_states[i];
            joint_state_t& st = _ctx->last_frame_joint_states[i];

            if (_ctx->last_frame_raw_px[i].has_value())
            {
                const Eigen::Vector2d p = _ctx->last_frame_raw_px[i].value();
                for (auto& lpf : fs.px_smoother) {
                    lpf.set_min_cutoff(_opt.position_filter.min_cutoff_hz);
                    lpf.set_beta(_opt.position_filter.beta);
                    lpf.set_derivate_cutoff(_opt.position_filter.dcutoff_hz);
                }

                // Reseed on cold start / long gap / smoothing off; else low-pass each image axis.
                if (!_opt.enable_position_smoothing
                    || !fs.last_px_out.has_value()
                    || (t - fs.last_seen) > _opt.reset_gap)
                {
                    for (auto& lpf : fs.px_smoother) { lpf.reset(); }
                    fs.last_px_out = p;
                }
                else
                {
                    const seconds_f64 dt = std::clamp(seconds_f64{ t - fs.last_step_time }, _opt.dt_min, _opt.dt_max);
                    Eigen::Vector2d out;
                    for (int a = 0; a < 2; ++a) { out[a] = fs.px_smoother[a].filter(p[a], dt.count()); }
                    fs.last_px_out = out;
                }
                fs.last_step_time = t;
                fs.last_seen = t;

                _ctx->last_frame_px[i] = fs.last_px_out;
                st.raw_position = to_rig_space(p, _ctx->meters_per_pixel, this->_side());
                st.position = to_rig_space(fs.last_px_out.value(), _ctx->meters_per_pixel, this->_side());
            }
            else if (fs.last_px_out.has_value() && (t - fs.last_seen) <= _opt.max_hold)
            {
                // No fresh detection: hold the point within the window.
                _ctx->last_frame_px[i] = fs.last_px_out;
                st.position = to_rig_space(fs.last_px_out.value(), _ctx->meters_per_pixel, this->_side());
            }
        }

        // ----- Pass 4: rig-sign segment angles -> published angles + rest-relative motion -----
        // The model is laid out at the top of this file. Every angle is read off the smoothed +
        // held image points (`last_frame_px`), so the occlusion policy above applies to the
        // angles as well; a direction is scale free, so the metric approximation never enters
        // any of them.
        {
            auto& states = _ctx->last_frame_joint_states;
            const joint_id_t root = get_root_joint();
            const auto at = [](joint_id_t j) { return index_of(j); };

            // The root's bone is the torso, which stands along the reference axis, so its
            // segment angle is 0 by definition, with no measurement involved. It has no parent
            // bone, hence no clinical angle.
            states[at(root)].sagittal_segment_angle = 0.0;

            const std::optional<joint_id_t> hip = hip_of_side(_ctx->tracked_side);
            const std::optional<joint_id_t> knee =
                hip.has_value() ? get_child_joint(hip.value()) : std::nullopt;
            const std::optional<joint_id_t> ankle =
                knee.has_value() ? get_child_joint(knee.value()) : std::nullopt;
            const std::optional<joint_id_t> foot =
                ankle.has_value() ? get_child_joint(ankle.value()) : std::nullopt;

            if (hip.has_value() && knee.has_value() && ankle.has_value() && foot.has_value())
            {
                const double side = this->_side();
                const auto& px = _ctx->last_frame_px;

                // --- this frame's rig-sign segment angles, one per bone of the tracked leg ---
                // The hip point rides the shared pelvis tag, so the thigh runs from it. Each is
                // empty where an endpoint is missing, and every output below asks for exactly
                // the segments it is defined on, so one lost tag costs only the angles built on
                // it. The foot joint is a marker end site: it lends the foot bone its far
                // endpoint and carries no angles of its own.
                const std::optional<double> rig_sagittal_segment_ang_thigh = rig_sagittal_segment_angle_from_px(px[at(hip.value())], px[at(knee.value())], side);
                const std::optional<double> rig_sagittal_segment_ang_shin  = rig_sagittal_segment_angle_from_px(px[at(knee.value())], px[at(ankle.value())], side);
                const std::optional<double> rig_sagittal_segment_ang_foot  = rig_sagittal_segment_angle_from_px(px[at(ankle.value())], px[at(foot.value())], side);

                // --- published measured angles (no rest pose involved) ---
                // Carried into the document conventions at this boundary; every per-joint sign
                // and neutral offset lives in the joints_def.hh table.
                if (rig_sagittal_segment_ang_thigh.has_value()) {
                    const std::optional<double> hip_clinical =
                        sagittal_clinical_angle_from_rig_bend(hip.value(), rig_sagittal_segment_ang_thigh.value());
                    states[at(hip.value())].sagittal_segment_angle = sagittal_segment_angle_from_rig(rig_sagittal_segment_ang_thigh.value());
                    states[at(hip.value())].sagittal_clinical_angle = hip_clinical;
                    states[at(hip.value())].sagittal_included_angle = hip_clinical.has_value()
                        ? sagittal_included_angle_from_clinical(hip.value(), hip_clinical.value())
                        : std::nullopt;
                }
                if (rig_sagittal_segment_ang_shin.has_value()) {
                    states[at(knee.value())].sagittal_segment_angle = sagittal_segment_angle_from_rig(rig_sagittal_segment_ang_shin.value());
                    if (rig_sagittal_segment_ang_thigh.has_value()) {
                        const std::optional<double> knee_clinical = sagittal_clinical_angle_from_rig_bend(knee.value(),
                            wrap_pi(rig_sagittal_segment_ang_shin.value() - rig_sagittal_segment_ang_thigh.value()));
                        states[at(knee.value())].sagittal_clinical_angle = knee_clinical;
                        states[at(knee.value())].sagittal_included_angle = knee_clinical.has_value()
                            ? sagittal_included_angle_from_clinical(knee.value(), knee_clinical.value())
                            : std::nullopt;
                    }
                }
                if (rig_sagittal_segment_ang_foot.has_value()) {
                    states[at(ankle.value())].sagittal_segment_angle = sagittal_segment_angle_from_rig(rig_sagittal_segment_ang_foot.value());
                    if (rig_sagittal_segment_ang_shin.has_value()) {
                        const std::optional<double> ankle_clinical = sagittal_clinical_angle_from_rig_bend(ankle.value(),
                            wrap_pi(rig_sagittal_segment_ang_foot.value() - rig_sagittal_segment_ang_shin.value()));
                        states[at(ankle.value())].sagittal_clinical_angle = ankle_clinical;
                        states[at(ankle.value())].sagittal_included_angle = ankle_clinical.has_value()
                            ? sagittal_included_angle_from_clinical(ankle.value(), ankle_clinical.value())
                            : std::nullopt;
                    }
                }

                // --- rest-relative motion ---
                // The change of each rig-sign segment angle since the captured rest, then the
                // same subtraction cascade on the changes for the per-joint bends; the reference
                // axis cancels in each change. All three bones are required on both ends: a rig
                // driven by these rotations needs the whole chain to describe one pose.
                if (this->has_rest_pose())
                {
                    const auto& rest_px = _ctx->rest_pose->joint_px;
                    const std::optional<double> rest_rig_sagittal_segment_ang_thigh = rig_sagittal_segment_angle_from_px(rest_px[at(hip.value())], rest_px[at(knee.value())], side);
                    const std::optional<double> rest_rig_sagittal_segment_ang_shin  = rig_sagittal_segment_angle_from_px(rest_px[at(knee.value())], rest_px[at(ankle.value())], side);
                    const std::optional<double> rest_rig_sagittal_segment_ang_foot  = rig_sagittal_segment_angle_from_px(rest_px[at(ankle.value())], rest_px[at(foot.value())], side);

                    if (rig_sagittal_segment_ang_thigh.has_value() && rig_sagittal_segment_ang_shin.has_value() && rig_sagittal_segment_ang_foot.has_value()
                        && rest_rig_sagittal_segment_ang_thigh.has_value() && rest_rig_sagittal_segment_ang_shin.has_value() && rest_rig_sagittal_segment_ang_foot.has_value())
                    {
                        const double rig_sagittal_segment_delta_thigh = wrap_pi(rig_sagittal_segment_ang_thigh.value() - rest_rig_sagittal_segment_ang_thigh.value());
                        const double rig_sagittal_segment_delta_shin  = wrap_pi(rig_sagittal_segment_ang_shin.value()  - rest_rig_sagittal_segment_ang_shin.value());
                        const double rig_sagittal_segment_delta_foot  = wrap_pi(rig_sagittal_segment_ang_foot.value()  - rest_rig_sagittal_segment_ang_foot.value());

                        const double rig_sagittal_bend_delta_hip   = rig_sagittal_segment_delta_thigh;
                        const double rig_sagittal_bend_delta_knee  = wrap_pi(rig_sagittal_segment_delta_shin - rig_sagittal_segment_delta_thigh);
                        const double rig_sagittal_bend_delta_ankle = wrap_pi(rig_sagittal_segment_delta_foot - rig_sagittal_segment_delta_shin);

                        // The rotation is the rig-sign bend delta put straight onto the shared
                        // hinge axis; the delta fields are the same motion in the document
                        // conventions. Pelvis is the fixed base.
                        states[at(root)].local_anim_rot = Eigen::Quaterniond::Identity();
                        states[at(root)].sagittal_segment_angle_delta = 0.0;
                        states[at(hip.value())].local_anim_rot = Eigen::Quaterniond{
                            Eigen::AngleAxisd{ rig_sagittal_bend_delta_hip, pose_estimator_base::kRigLateralAxis } };
                        states[at(knee.value())].local_anim_rot = Eigen::Quaterniond{
                            Eigen::AngleAxisd{ rig_sagittal_bend_delta_knee, pose_estimator_base::kRigLateralAxis } };
                        states[at(ankle.value())].local_anim_rot = Eigen::Quaterniond{
                            Eigen::AngleAxisd{ rig_sagittal_bend_delta_ankle, pose_estimator_base::kRigLateralAxis } };

                        states[at(hip.value())].sagittal_clinical_angle_delta =
                            sagittal_clinical_angle_delta_from_rig_bend_delta(hip.value(), rig_sagittal_bend_delta_hip);
                        states[at(knee.value())].sagittal_clinical_angle_delta =
                            sagittal_clinical_angle_delta_from_rig_bend_delta(knee.value(), rig_sagittal_bend_delta_knee);
                        states[at(ankle.value())].sagittal_clinical_angle_delta =
                            sagittal_clinical_angle_delta_from_rig_bend_delta(ankle.value(), rig_sagittal_bend_delta_ankle);

                        states[at(hip.value())].sagittal_segment_angle_delta = sagittal_segment_angle_from_rig(rig_sagittal_segment_delta_thigh);
                        states[at(knee.value())].sagittal_segment_angle_delta = sagittal_segment_angle_from_rig(rig_sagittal_segment_delta_shin);
                        states[at(ankle.value())].sagittal_segment_angle_delta = sagittal_segment_angle_from_rig(rig_sagittal_segment_delta_foot);
                    }
                }
            }
        }

    }

    bool sagittal_pose_estimator::calibrate_rest_pose()
    {
        context_t::rest_pose_info_t new_rest{};

        bool any = false;
        for (size_t i = 0; i < kNumJoints; ++i) {
            // Only latch freshly detected joints; a held point is a stale reference.
            if (_ctx->last_frame_detection_flags[i]) {
                new_rest.joint_px[i] = _ctx->last_frame_raw_px[i];
            }
            any = any || new_rest.joint_px[i].has_value();
        }

        // Stamped with the leg this frame was seen on, which is what the points below belong to.
        new_rest.side = _ctx->tracked_side;

        if (any) { _ctx->rest_pose = new_rest; }
        else { _ctx->rest_pose.reset(); }

        return any;
    }

    void sagittal_pose_estimator::clear_rest_pose()
    {
        _ctx->rest_pose.reset();
    }

    void sagittal_pose_estimator::reset_tracking()
    {
        // Fresh filters/timers and no held points, so the first frame of the next stream reseeds
        // rather than filtering or holding against a prior stream's point and timestamps. The
        // tracked leg goes with it: a new source may well be shot from the other side.
        _ctx->filter_states = {};
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_raw_px = {};
        _ctx->last_frame_px = {};
        _ctx->last_frame_detection_flags = {};
        _ctx->meters_per_pixel = 0.0;

        _ctx->tracked_side.reset();
    }

    void sagittal_pose_estimator::on_frame_geometry_changed()
    {
        // Both the rest reference and the position track are held in image-plane points, 
        // so a moved ROI leaves them describing pixels that are no longer there.
        this->clear_rest_pose();
        this->reset_tracking();
    }

} // namespace pose
