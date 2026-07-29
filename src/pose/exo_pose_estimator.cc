#include "exo_pose_estimator.hh"
#include "leg_ik.hh"

#include <algorithm>
#include <array>

namespace pose
{
    namespace
    {
        size_t index_of(joint_id_t jid) { return static_cast<size_t>(jid); }

        // Raw camera-space position of a detected tag: the chosen pose's translation, falling back to
        // the first pose candidate. A tag's pose candidates share the same translation, so the choice
        // does not affect the position.
        std::optional<Eigen::Vector3d> detection_position(const tag_detection_t& det)
        {
            if (det.pose.has_value()) { return det.pose->transform.translation(); }
            if (det.num_pose_candidates > 0) { return det.pose_candidates[0].transform.translation(); }
            return std::nullopt;
        }
    } // namespace

    struct exo_pose_estimator::context_t
    {
        // Per-joint position filter + occlusion timers, persisting across frames. One Euro per axis.
        struct joint_filter_state_t
        {
            std::array<dsp::OneEuroFilter, 3> pos_smoother{};
            std::optional<Eigen::Vector3d> last_pos_out; // last smoothed position (hold output)
            std::chrono::microseconds last_seen{ 0 }; // time of the last fresh detection (hold origin)
            std::chrono::microseconds last_step_time{ 0 }; // time of the last fresh filter step (dt source)
        };

        std::array<joint_filter_state_t, kNumJoints> filter_states{}; // persists across frames
        std::array<joint_state_t, kNumJoints> last_frame_joint_states{}; // per-frame output; reset every update()
        std::array<std::optional<Eigen::Vector3d>, kNumJoints> last_frame_raw_positions{}; // per-frame raw detected positions
        std::array<bool, kNumJoints> last_frame_detection_flags{}; // per-joint fresh-detection flag

        // Captured rest camera-space positions (bind positions). Outer optional == "calibrated";
        // inner per-joint optional == "that joint had a position at capture time". Rest bone
        // directions/lengths for the leg IK derive from these.
        std::optional<std::array<std::optional<Eigen::Vector3d>, kNumJoints>> rest_positions;
    };

    exo_pose_estimator::exo_pose_estimator(const options_t& opt)
        : _opt{ opt }
        , _ctx{ std::make_unique<context_t>() }
    { }

    exo_pose_estimator::~exo_pose_estimator() = default;

    const joint_state_t& exo_pose_estimator::get_joint_state(joint_id_t j) const
    {
        return _ctx->last_frame_joint_states[index_of(j)];
    }

    std::span<const joint_state_t> exo_pose_estimator::get_joint_states() const
    {
        return _ctx->last_frame_joint_states;
    }

    bool exo_pose_estimator::has_rest_pose() const
    {
        return _ctx->rest_positions.has_value();
    }

    std::optional<Eigen::Vector3d> exo_pose_estimator::get_rest_position(joint_id_t j) const
    {
        if (_ctx->rest_positions.has_value()) {
            return _ctx->rest_positions.value()[index_of(j)];
        }
        return std::nullopt;
    }

    void exo_pose_estimator::update(
        const std::span<const tag_detection_t> tag_detections,
        const std::chrono::microseconds sensor_timestamp)
    {
        const auto t = sensor_timestamp;

        // Reset the per-frame output; filter_states (filters/timers) persist.
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_raw_positions = {};
        _ctx->last_frame_detection_flags = {};

        // ----- Pass 1: bind each detection's 3D position to its joint (via the static tag table) -----
        for (const auto& det : tag_detections)
        {
            const auto joint = tag_to_joint(det.id);
            if (!joint.has_value()) { continue; } // tag id not part of the rig
            const auto p = detection_position(det);
            if (!p.has_value()) { continue; } // no pose (no intrinsics / undetected)

            const size_t i = index_of(joint.value());
            _ctx->last_frame_raw_positions[i] = p;
            _ctx->last_frame_detection_flags[i] = true;
            _ctx->last_frame_joint_states[i].raw_position = p; // raw
        }

        // ----- Pass 2: smooth + hold each joint's camera-space position -----
        for (const auto& info : kJointsInfo)
        {
            const size_t i = index_of(info.id);
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

        // ----- Pass 3: leg IK (3D positions -> per-joint local_anim_rot) -----
        // Needs a captured rest pose (for rest bone directions). Data driven over kJointsInfo: a leg
        // is (knee = child of root, ankle = child of knee, foot = child of ankle). Each joint is a
        // 1-DOF forward/back hinge about the shared lateral axis when the hinge constraint is on.
        if (_ctx->rest_positions.has_value())
        {
            const auto& rest = _ctx->rest_positions.value();
            const auto position_of = [this](joint_id_t j) { return _ctx->last_frame_joint_states[index_of(j)].position; };
            const auto child_of = [](joint_id_t parent) -> std::optional<joint_id_t> {
                for (const auto& c : kJointsInfo) {
                    if (!is_root_joint(c.id) && c.parent == parent) { return c.id; }
                }
                return std::nullopt;
            };

            joint_id_t root = kJointsInfo[0].id;
            for (const auto& j : kJointsInfo) { if (is_root_joint(j.id)) { root = j.id; break; } }

            const std::optional<Eigen::Vector3d> hinge = _opt.enable_hinge_constraint
                ? std::optional<Eigen::Vector3d>{ ik_normalize(_opt.hinge_axis_world) } : std::nullopt;

            // Pelvis is the fixed base: identity animation.
            _ctx->last_frame_joint_states[index_of(root)].local_anim_rot = Eigen::Quaterniond::Identity();

            for (const auto& knee_info : kJointsInfo)
            {
                if (is_root_joint(knee_info.id) || knee_info.parent != root) { continue; } // knee = child of root
                const joint_id_t knee = knee_info.id;
                const std::optional<joint_id_t> ankle = child_of(knee);
                const std::optional<joint_id_t> foot = ankle.has_value() ? child_of(ankle.value()) : std::nullopt;
                if (!ankle.has_value() || !foot.has_value()) { continue; }

                const auto hip_pos = position_of(root), knee_pos = position_of(knee);
                const auto ankle_pos = position_of(ankle.value()), foot_pos = position_of(foot.value());
                const auto& r_hip = rest[index_of(root)];
                const auto& r_knee = rest[index_of(knee)];
                const auto& r_ankle = rest[index_of(ankle.value())];
                const auto& r_foot = rest[index_of(foot.value())];
                if (!hip_pos || !knee_pos || !ankle_pos || !foot_pos || !r_hip || !r_knee || !r_ankle || !r_foot) { continue; }

                const Eigen::Vector3d thigh_rest = ik_normalize(r_knee.value() - r_hip.value());
                const Eigen::Vector3d shin_rest = ik_normalize(r_ankle.value() - r_knee.value());
                const Eigen::Vector3d foot_rest = ik_normalize(r_foot.value() - r_ankle.value());

                const leg_ik_result r = solve_leg_ik(
                    hip_pos.value(), knee_pos.value(), ankle_pos.value(), foot_pos.value(),
                    thigh_rest, shin_rest, foot_rest, 
                    Eigen::Quaterniond::Identity(), 
                    hinge
                );

                // Bone->joint map (matches the webview rig): knee joint drives the thigh, ankle the
                // shin, foot the foot bone.
                _ctx->last_frame_joint_states[index_of(knee)].local_anim_rot = r.hip;
                _ctx->last_frame_joint_states[index_of(ankle.value())].local_anim_rot = r.knee;
                _ctx->last_frame_joint_states[index_of(foot.value())].local_anim_rot = r.ankle;
            }
        }
    }

    bool exo_pose_estimator::calibrate_rest_pose()
    {
        std::array<std::optional<Eigen::Vector3d>, kNumJoints> new_rest_positions{};

        bool any = false;
        for (size_t i = 0; i < kNumJoints; ++i) {
            // Only latch freshly detected joints; a held position is a stale reference.
            if (_ctx->last_frame_detection_flags[i]) {
                new_rest_positions[i] = _ctx->last_frame_joint_states[i].raw_position;
            }
            any = any || new_rest_positions[i].has_value();
        }

        if (any) { _ctx->rest_positions = new_rest_positions; }
        else { _ctx->rest_positions.reset(); }

        return any;
    }

    void exo_pose_estimator::clear_rest_pose()
    {
        _ctx->rest_positions.reset();
    }

    void exo_pose_estimator::reset_tracking()
    {
        // Fresh filters/timers and no held positions, so the first frame of the next stream reseeds
        // rather than filtering or holding against a prior stream's position and timestamps.
        _ctx->filter_states = {};
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_raw_positions = {};
        _ctx->last_frame_detection_flags = {};
    }

} // namespace pose
