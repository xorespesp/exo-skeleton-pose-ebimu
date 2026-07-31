#include "sagittal_pose_estimator.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace pose
{
    namespace
    {
        size_t index_of(joint_id_t jid) { return static_cast<size_t>(jid); }

        joint_id_t root_joint() {
            for (const auto& j : kJointsInfo) { if (is_root_joint(j.id)) { return j.id; } }
            return kJointsInfo[0].id;
        }

        std::optional<joint_id_t> child_of(joint_id_t parent) {
            for (const auto& c : kJointsInfo) {
                if (!is_root_joint(c.id) && c.parent == parent) { return c.id; }
            }
            return std::nullopt;
        }

        // Entry point of one leg's chain: the root's child on `side`.
        std::optional<joint_id_t> knee_of_side(std::optional<joint_side_t> side) {
            if (!side.has_value()) { return std::nullopt; }
            const joint_id_t root = root_joint();
            for (const auto& c : kJointsInfo) {
                if (is_root_joint(c.id) || c.parent != root) { continue; }
                if (c.side == side.value()) { return c.id; }
            }
            return std::nullopt;
        }

        // Fold an angle into [-pi, pi] so a bone crossing the atan2 branch cut does not jump by 2pi.
        double wrap_pi(double a) {
            constexpr double two_pi = 2.0 * std::numbers::pi;
            a = std::fmod(a + std::numbers::pi, two_pi);
            if (a < 0.0) { a += two_pi; }
            return a - std::numbers::pi;
        }

        // Direction of the bone from `start` to `end` in the image plane.
        double bone_angle(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
            const Eigen::Vector2d d = end - start;
            return std::atan2(d.y(), d.x());
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

        // Measured in-plane angle as a rotation about the rig's lateral axis. The camera's optical
        // axis runs along rig -X on a left-side view, so a positive in-plane angle is a negative
        // rotation about rig X.
        //
        // NOTE: `side` ties this sign to the one in to_rig_space(). Mirroring the view flips both
        // together, and getting them out of step shows up as legs swinging opposite to the footage.
        Eigen::Quaterniond flexion_rotation(double angle, double side) {
            return Eigen::Quaterniond{ Eigen::AngleAxisd{ -side * angle, Eigen::Vector3d::UnitX() } };
        }

        // Mean edge length of a tag quad [px]; against the tag's physical size it gives a
        // meters-per-pixel scale.
        double mean_edge_px(const std::array<cv::Point2f, 4>& corners) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                const cv::Point2f d = corners[(k + 1) % 4] - corners[k];
                sum += std::sqrt(static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y);
            }
            return sum / 4.0;
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
            std::chrono::microseconds last_seen{ 0 }; // time of the last fresh detection (hold origin)
            std::chrono::microseconds last_step_time{ 0 }; // time of the last fresh filter step (dt source)
        };

        std::array<joint_filter_state_t, kNumJoints> filter_states{}; // persists across frames
        std::array<joint_state_t, kNumJoints> last_frame_joint_states{}; // per-frame output; reset every update()
        std::array<std::optional<Eigen::Vector2d>, kNumJoints> last_frame_raw_px{}; // per-frame raw tag centers
        std::array<std::optional<Eigen::Vector2d>, kNumJoints> last_frame_px{}; // smoothed + held centers (angle source)
        std::array<bool, kNumJoints> last_frame_detection_flags{}; // per-joint fresh-detection flag

        // One conversion factor for the whole rig, averaged over this frame's tags and held while
        // none are seen. Per-tag factors would stretch each point away from the image origin by a
        // different amount, skewing the skeleton's shape; a single one keeps the image geometry and
        // only sets its size.
        double meters_per_pixel{ 0.0 };

        // The captured rest (bind) reference. Kept in pixels: that is what the rest bone angles
        // derive from, and it makes them independent of how far the exo stood at capture.
        struct rest_pose_info_t
        {
            // Per-joint image-plane point at capture; empty for a joint that was not measured then.
            std::array<std::optional<Eigen::Vector2d>, kNumJoints> joint_px{};
        };
        std::optional<rest_pose_info_t> rest_pose; // empty until calibrated

        // Which leg is being seen, and with it which way the camera faces (the sign that carries the
        // image plane into rig space). Held across frames so a frame that detects nothing keeps the
        // last answer.
        std::optional<joint_side_t> tracked_side;

        std::optional<leg_angles_t> angles; // this frame's angles; empty until rest + full chain
    };

    sagittal_pose_estimator::sagittal_pose_estimator(const options_t& opt)
        : _opt{ opt }
        , _ctx{ std::make_unique<context_t>() }
    { }

    // Which side the camera views from follows from the tagged leg, since the far leg is hidden
    // behind the near one. Before any tag has been seen the left-hand mapping is assumed; nothing
    // downstream depends on it yet, because rotations need a rest pose that cannot exist by then.
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
        return _ctx->rest_pose.has_value();
    }

    // Converted with the current frame's factor rather than the one at capture, so a rest skeleton
    // drawn next to the measured one shares its size and only their angles can differ.
    std::optional<Eigen::Vector3d> sagittal_pose_estimator::get_rest_position(joint_id_t j) const
    {
        if (!_ctx->rest_pose.has_value()) { return std::nullopt; }
        const auto& px = _ctx->rest_pose->joint_px[index_of(j)];
        if (!px.has_value()) { return std::nullopt; }
        return to_rig_space(px.value(), _ctx->meters_per_pixel, this->_side());
    }

    std::optional<joint_id_t> sagittal_pose_estimator::tracked_leg_knee() const
    {
        return knee_of_side(_ctx->tracked_side);
    }

    std::optional<sagittal_pose_estimator::leg_angles_t> sagittal_pose_estimator::leg_angles() const
    {
        return _ctx->angles;
    }

    void sagittal_pose_estimator::update(
        const std::span<const tag_detection_t> tag_detections,
        const std::chrono::microseconds sensor_timestamp)
    {
        const auto t = sensor_timestamp;

        // Reset the per-frame output; filter_states and the tracked side persist.
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_raw_px = {};
        _ctx->last_frame_px = {};
        _ctx->last_frame_detection_flags = {};
        _ctx->angles.reset();

        // ----- Pass 1: which leg is in view -----
        // Only one leg is tagged, so counting the sides of this frame's ids answers it outright.
        // A tie means nothing decisive was seen (no tags at all, or only midline ones), and the
        // previous answer stands.
        {
            int right_seen = 0, left_seen = 0;
            for (const auto& det : tag_detections) {
                const auto joint = tag_to_joint(det.id);
                if (!joint.has_value()) { continue; } // tag id not part of the rig
                switch (joint_side(joint.value())) {
                case joint_side_t::right: ++right_seen; break;
                case joint_side_t::left:  ++left_seen;  break;
                default: break; // midline: belongs to neither leg
                }
            }

            if (right_seen != left_seen) {
                _ctx->tracked_side = (right_seen > left_seen) ? joint_side_t::right : joint_side_t::left;
            }
        }

        // ----- Pass 2: bind each detection's tag center to its joint (via the static tag table) -----
        // The tags also fix the frame's metric scale, averaged over all of them: the rig sits at one
        // distance from a side camera, so one scale describes it and the spread between tags is
        // measurement noise rather than depth.
        double edge_px_sum = 0.0;
        int edge_px_count = 0;
        for (const auto& det : tag_detections)
        {
            const auto joint = tag_to_joint(det.id);
            if (!joint.has_value()) { continue; } // tag id not part of the rig

            const double edge_px = mean_edge_px(det.corners);
            if (edge_px < 1e-6) { continue; } // degenerate quad: no usable scale

            const size_t i = index_of(joint.value());
            _ctx->last_frame_raw_px[i] = Eigen::Vector2d{ det.center.x, det.center.y };
            _ctx->last_frame_detection_flags[i] = true;

            edge_px_sum += edge_px;
            ++edge_px_count;
        }
        // No tags this frame: keep the last factor so held points stay where they were.
        if (edge_px_count > 0) { _ctx->meters_per_pixel = _opt.tag_size_m / (edge_px_sum / edge_px_count); }

        // ----- Pass 3: smooth + hold each joint's image-plane point -----
        for (const auto& info : kJointsInfo)
        {
            const size_t i = index_of(info.id);
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

        // ----- Pass 4: in-plane angles (image points -> per-joint local_anim_rot) -----
        // Needs a captured rest pose (for the rest bone angles) and a leg to walk. Each bone's angle
        // change since rest, minus its parent bone's, is that joint's flexion. Measured on pixels,
        // so the metric scale never enters the result.
        const std::optional<joint_id_t> tracked_knee = knee_of_side(_ctx->tracked_side);
        if (_ctx->rest_pose.has_value() && tracked_knee.has_value())
        {
            const joint_id_t root = root_joint();
            const joint_id_t knee = tracked_knee.value();
            const std::optional<joint_id_t> ankle = child_of(knee);
            const std::optional<joint_id_t> foot = ankle.has_value() ? child_of(ankle.value()) : std::nullopt;

            if (ankle.has_value() && foot.has_value())
            {
                const auto& px = _ctx->last_frame_px;
                const auto& rest_px = _ctx->rest_pose->joint_px;
                const auto at = [](joint_id_t j) { return index_of(j); };

                const bool have_all =
                    px[at(root)] && px[at(knee)] && px[at(ankle.value())] && px[at(foot.value())] &&
                    rest_px[at(root)] && rest_px[at(knee)] && rest_px[at(ankle.value())] && rest_px[at(foot.value())];

                if (have_all)
                {
                    const double d_thigh = wrap_pi(
                        bone_angle(px[at(root)].value(), px[at(knee)].value())
                        - bone_angle(rest_px[at(root)].value(), rest_px[at(knee)].value()));
                    const double d_shin = wrap_pi(
                        bone_angle(px[at(knee)].value(), px[at(ankle.value())].value())
                        - bone_angle(rest_px[at(knee)].value(), rest_px[at(ankle.value())].value()));
                    const double d_foot = wrap_pi(
                        bone_angle(px[at(ankle.value())].value(), px[at(foot.value())].value())
                        - bone_angle(rest_px[at(ankle.value())].value(), rest_px[at(foot.value())].value()));

                    const leg_angles_t a{
                        .hip = d_thigh,
                        .knee = wrap_pi(d_shin - d_thigh),
                        .ankle = wrap_pi(d_foot - d_shin),
                    };

                    // Bone->joint map (matches the webview rig): knee joint drives the thigh, ankle
                    // the shin, foot the foot bone. Pelvis is the fixed base.
                    const double side = this->_side();
                    auto& states = _ctx->last_frame_joint_states;
                    states[at(root)].local_anim_rot = Eigen::Quaterniond::Identity();
                    states[at(knee)].local_anim_rot = flexion_rotation(a.hip, side);
                    states[at(ankle.value())].local_anim_rot = flexion_rotation(a.knee, side);
                    states[at(foot.value())].local_anim_rot = flexion_rotation(a.ankle, side);

                    _ctx->angles = a;
                }
            }
        }

        // ----- Pass 5: complete the rig -----
        // Every joint owes the consumer a rotation, so a joint left without one takes its mirror's.
        // Positions stay as measured: only the tagged leg was observed, and the plots should say so.
        auto& states = _ctx->last_frame_joint_states;
        for (const auto& info : kJointsInfo)
        {
            if (!has_mirror_joint(info.id)) { continue; }
            const size_t src = index_of(info.id);
            const size_t dst = index_of(info.mirror);
            if (states[src].local_anim_rot.has_value() && !states[dst].local_anim_rot.has_value()) {
                states[dst].local_anim_rot = states[src].local_anim_rot;
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
        _ctx->angles.reset();

        _ctx->tracked_side.reset();
    }

} // namespace pose
