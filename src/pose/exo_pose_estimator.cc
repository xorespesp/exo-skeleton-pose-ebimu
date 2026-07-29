#include "exo_pose_estimator.hh"
#include "rotation_constraint.hh"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <limits>

namespace pose
{
    namespace
    {
        size_t index_of(joint_id_t jid) { return static_cast<size_t>(jid); }

        // Orthonormalized rotation part of a tag->camera pose as a quaternion.
        Eigen::Quaterniond rot_of(const Eigen::Isometry3d& pose)
        {
            return Eigen::Quaterniond{ pose.rotation() }.normalized();
        }

        // Rotation of `child` expressed in `parent`'s frame: R = parent^-1 * child.
        Eigen::Quaterniond q_relative(const Eigen::Quaterniond& parent, const Eigen::Quaterniond& child)
        {
            return (parent.conjugate() * child).normalized();
        }

        constexpr double kRadToDeg = 57.29577951308232;

        // How a candidate pose reads as joint motion: its rotation relative to the captured rest
        // pose, expressed in the parent's frame, split about the hinge axis. These are the two
        // quantities a physical joint-limit gate would test.
        struct hinge_angles_t
        {
            double flex_deg;     // signed rotation about the hinge axis (the joint's only real DOF)
            double off_axis_deg; // magnitude of what is left after removing the flexion (0 for an ideal hinge)
        };

        // `cand_global` / `parent_global` are camera-frame rotations;
        // `rest_local` is the joint's captured rest rotation in its parent's frame.
        hinge_angles_t hinge_angles_of(
            const Eigen::Quaterniond& cand_global,
            const Eigen::Quaterniond& parent_global,
            const Eigen::Quaterniond& rest_local)
        {
            const Eigen::Quaterniond anim = q_relative(rest_local, q_relative(parent_global, cand_global));
            return {
                twist_angle(anim, kExoHingeLocalAxis) * kRadToDeg,
                swing_angle(anim, kExoHingeLocalAxis) * kRadToDeg
            };
        }

    } // namespace

    struct exo_pose_estimator::context_t
    {
        // Per-joint state that persists across frames (filter + occlusion timers).
        struct joint_filter_state_t
        {
            std::unique_ptr<rotation_filter_base> smoother; // swappable smoothing kernel
            std::optional<Eigen::Quaterniond> last_out; // last smoothed global rotation (hold output)
            std::chrono::microseconds last_seen{ 0 };   // time of the last fresh detection (hold timer origin)
            std::chrono::microseconds t_prev{ 0 };      // time of the last fresh filter step (dt source)
        };

        // A joint's tag pose candidates for the current frame (reset every update()).
        struct tag_pose_candidates_t
        {
            std::array<Eigen::Isometry3d, 2> transform{}; // tag->camera per candidate (obj_err ascending)
            std::array<double, 2> obj_err{};              // object-space error per candidate (drives flip rejection)
            int n{ 0 };
        };

        rotation_filter_kind built_kind{}; // kernel kind currently instantiated in filter_states
        std::array<joint_filter_state_t, kNumJoints> filter_states{};  // persists across frames
        std::array<joint_state_t, kNumJoints> last_frame_joint_states{}; // per-frame output; reset every update()
        std::array<bool, kNumJoints> last_frame_detection_flags{}; // per-joint fresh-detection flag (gates rest capture)
        std::array<tag_pose_candidates_t, kNumJoints> last_frame_tag_pose_candidates{}; // per-joint tag pose candidates

        // Captured rest pose: outer optional == "calibrated";
        // inner per-joint optional == "that joint had a computable local_rot at capture time".
        std::optional<std::array<std::optional<Eigen::Quaterniond>, kNumJoints>> rest_pose;
    };

    exo_pose_estimator::exo_pose_estimator(const options_t& opt)
        : _opt{ opt }
        , _ctx{ std::make_unique<context_t>() }
    {
        for (auto& fs : _ctx->filter_states) {
            fs.smoother = make_rotation_filter(_opt.filter);
        }
        _ctx->built_kind = _opt.filter.kind;
    }

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
        return _ctx->rest_pose.has_value();
    }

    std::optional<Eigen::Quaterniond> exo_pose_estimator::rest_rotation(joint_id_t j) const
    {
        if (_ctx->rest_pose.has_value()) {
            return _ctx->rest_pose.value()[index_of(j)];
        }
        return std::nullopt;
    }

    void exo_pose_estimator::update(
        const std::span<const tag_detection_t> tag_detections,
        const std::chrono::microseconds sensor_timestamp)
    {
        const auto t = sensor_timestamp;

        // ----- Pass 0: rebuild / reconfigure kernels -----
        // Pick up any edited kernel selection / params before this frame.
        {
            if (_opt.filter.kind != _ctx->built_kind) {
                // Algorithm switched: rebuild the kernels (loses filter state; rare, GUI-driven).
                for (auto& fs : _ctx->filter_states) {
                    fs.smoother = make_rotation_filter(_opt.filter);
                }
                _ctx->built_kind = _opt.filter.kind;
            } else {
                // Same algorithm: push live params (cheap, no state loss).
                for (auto& fs : _ctx->filter_states) {
                    fs.smoother->configure(_opt.filter);
                }
            }
        }

        // Reset only the per-frame output; filter_states (filter/timers) persist.
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_detection_flags = {};
        _ctx->last_frame_tag_pose_candidates = {};

        // Captured rest reference rotation for joint `jid`, if calibrated and computable.
        const auto rest_rotation_of = [this](joint_id_t jid) -> std::optional<Eigen::Quaterniond> {
            if (_ctx->rest_pose.has_value() && 
                _ctx->rest_pose.value()[index_of(jid)].has_value()) {
                return _ctx->rest_pose.value()[index_of(jid)];
            }
            return std::nullopt;
        };

        // pelvis is a fixed base: once a rest is captured, lock its rotation to that constant.
        const auto is_locked_pelvis_joint = [this, &rest_rotation_of](joint_id_t jid) {
            return _opt.enable_hinge_constraint 
                && is_root_joint(jid) 
                && rest_rotation_of(jid).has_value();
        };

        // ----- Pass 1: bind pose candidates to joints -----
        // Bind each detection's pose candidates to its joint via the static tag table.
        for (const auto& curr_tag_det : tag_detections)
        {
            if (curr_tag_det.num_pose_candidates <= 0) { continue; } // no pose (no intrinsics / undetected)

            const auto curr_tag_joint = tag_to_joint(curr_tag_det.id);
            if (!curr_tag_joint.has_value()) { continue; } // tag id not part of the rig

            auto& curr_j_pose_candidates = _ctx->last_frame_tag_pose_candidates[index_of(curr_tag_joint.value())];
            curr_j_pose_candidates.n = curr_tag_det.num_pose_candidates;
            for (int k = 0; k < curr_tag_det.num_pose_candidates; ++k) {
                curr_j_pose_candidates.transform[k] = curr_tag_det.pose_candidates[k].transform;
                curr_j_pose_candidates.obj_err[k] = curr_tag_det.pose_candidates[k].obj_err;
            }
        }

        // ----- Pass 2: select candidate + smooth/hold global rotation -----
        // Pick the physically valid candidate, then smooth+hold each joint's GLOBAL (camera-frame)
        // rotation. Parent-before-child order (kJointsInfo) lets candidate selection use the parent's
        // already-resolved rotation. Holding the global rotation lets a child recompute its local_rot
        // from a briefly occluded parent, so occlusion doesn't cascade downward.
        for (const auto& curr_j_info : kJointsInfo)
        {
            const size_t curr_j_idx = index_of(curr_j_info.id);
            auto& curr_j_fstate = _ctx->filter_states[curr_j_idx];
            joint_state_t& curr_j_state = _ctx->last_frame_joint_states[curr_j_idx];
            const auto& curr_j_pose_candidates = _ctx->last_frame_tag_pose_candidates[curr_j_idx];

            // Locked pelvis: constant camera-frame orientation, no smoothing/hold. 
            // Children read this as their parent, so pelvis noise/flips/occlusion never reach them.
            if (is_locked_pelvis_joint(curr_j_info.id)) {
                curr_j_state.global_rot = rest_rotation_of(curr_j_info.id);
                if (curr_j_pose_candidates.n > 0) {
                    curr_j_state.view_pose = curr_j_pose_candidates.transform[0];
                    curr_j_state.selected_candidate = 0; // locked base draws the detector's min-error candidate
                    _ctx->last_frame_detection_flags[curr_j_idx] = true;
                }
                continue;
            }

            if (curr_j_pose_candidates.n <= 0) {
                // No fresh detection: hold the last smoothed rotation within the hold window.
                if (curr_j_fstate.last_out.has_value() && (t - curr_j_fstate.last_seen) <= _opt.max_hold) {
                    curr_j_state.global_rot = curr_j_fstate.last_out;
                }
                continue; // else lost -> global_rot stays nullopt
            }

            const std::optional<Eigen::Quaterniond> curr_j_rest_rot = rest_rotation_of(curr_j_info.id);
            const std::optional<Eigen::Quaterniond> curr_j_parent_global = is_root_joint(curr_j_info.id)
                ? std::nullopt
                : _ctx->last_frame_joint_states[index_of(curr_j_info.parent)].global_rot;

            // Candidate selection: default to the best geometric fit (candidates[0] = min obj_err).
            // Reject the planar-ambiguity flip (the second candidate, whose twist points the
            // opposite way). obj_err discriminates the true fit from the flip strongly (typically
            // 20-100x), so trust it unless the two are near-tied; only in a genuine near-tie (tag
            // near head-on) fall back to temporal continuity -- keep the candidate whose absolute
            // rotation stays closest to the last smoothed output, since the flip is always a large
            // discrete jump. swing is deliberately NOT used to select: it rewards the flip whenever
            // the leg twists off the hinge plane, which was the original opposite-leg bug.
            size_t selected_candidate_idx = 0;
            if (_opt.enable_hinge_constraint
                && curr_j_rest_rot.has_value()
                && (is_root_joint(curr_j_info.id) || curr_j_parent_global.has_value()))
            {
                if (curr_j_pose_candidates.n == 2 && curr_j_fstate.last_out.has_value()) {
                    constexpr double kObjErrDecisiveRatio = 8.0; // err1 >= 8*err0 => trust min-obj_err candidate#0
                    const double e0 = curr_j_pose_candidates.obj_err[0];
                    const double e1 = curr_j_pose_candidates.obj_err[1];
                    if (e1 < kObjErrDecisiveRatio * e0) {
                        // near-tie: keep the branch closest to the last smoothed absolute rotation
                        const Eigen::Quaterniond& prev = curr_j_fstate.last_out.value();
                        double best = std::numeric_limits<double>::max();
                        for (int k = 0; k < curr_j_pose_candidates.n; ++k) {
                            const double d = prev.angularDistance(rot_of(curr_j_pose_candidates.transform[k]));
                            if (d < best) { best = d; selected_candidate_idx = static_cast<size_t>(k); }
                        }
                    }
                }
            }

            // TODO(physical-gate): drop physically impossible candidates before selecting, once the
            // rig is the real robot. Selection has exactly one weak spot: the seed. Cold start, and
            // reseed after an occlusion longer than `reset_gap`, resolve to `candidates[0]` with no
            // track to check it against, so a near-tied flip on that single frame locks the track
            // until obj_err turns decisive again. A gate closes that hole, because the flip is then
            // absent from the candidate set no matter what the seed logic would have preferred.
            //
            // Deriving the limits from the `[hinge-diag]` lines below (run at debug level):
            //   1. Sweep each joint through its full mechanical range. The `flex` values logged for
            //      the selected candidate trace out that joint's real [flex_min, flex_max].
            //   2. Hold the joint still at several angles. The frame-to-frame spread of `flex`/`off`
            //      on the selected candidate is the measurement noise, i.e. the margin to add.
            //   3. Induce flips and compare the rejected candidate's `flex`/`off` to that envelope.
            //      A gate earns its place only if the flip lands clearly outside it.
            // Then reject a candidate whose `flex` leaves [flex_min - margin, flex_max + margin], or
            // whose `off` exceeds the largest `off` seen on a correct pose plus margin. Per-joint
            // limits belong in a `kJointsInfo` column, not in constants here.
            //
            // Keep the gate loose: it must never reject a real pose, only obvious impossibilities.
            // Never rank near-tied candidates by `off_axis_deg`. Real off-axis motion inflates it on
            // the true candidate, which is precisely what made an earlier selector prefer the flip.
            // Fine discrimination stays with obj_err and temporal continuity.
            //
            // NOTE: the current rig is a folded-paper mock with no mechanical stops. Its flex range
            // is whatever the operator's hand does and its `off` carries the paper's slop, so limits
            // measured on it do not transfer. Collect these on the robot.
            if (_opt.enable_hinge_constraint
                && !is_root_joint(curr_j_info.id)
                && curr_j_rest_rot.has_value()
                && curr_j_parent_global.has_value()
                && spdlog::should_log(spdlog::level::trace)) // per-frame, per-joint: trace, or it drowns the log
            {
                const auto a0 = hinge_angles_of(
                    rot_of(curr_j_pose_candidates.transform[0]),
                    curr_j_parent_global.value(),
                    curr_j_rest_rot.value()
                );

                if (curr_j_pose_candidates.n == 2) {
                    const auto a1 = hinge_angles_of(
                        rot_of(curr_j_pose_candidates.transform[1]),
                        curr_j_parent_global.value(),
                        curr_j_rest_rot.value()
                    );
                    const double e0 = curr_j_pose_candidates.obj_err[0];
                    const double e1 = curr_j_pose_candidates.obj_err[1];
                    spdlog::trace(
                        "[hinge-diag] {} sel=c{}"
                        " | c0 err={:.6f} flex={:+7.2f} off={:6.2f}"
                        " | c1 err={:.6f} flex={:+7.2f} off={:6.2f}"
                        " | err_ratio={:.2f}",
                        curr_j_info.name, selected_candidate_idx,
                        e0, a0.flex_deg, a0.off_axis_deg,
                        e1, a1.flex_deg, a1.off_axis_deg,
                        e0 > 0.0 ? e1 / e0 : std::numeric_limits<double>::infinity()
                    );
                } else {
                    spdlog::trace(
                        "[hinge-diag] {} sel=c0 | c0 err={:.6f} flex={:+7.2f} off={:6.2f} | c1 none",
                        curr_j_info.name,
                        curr_j_pose_candidates.obj_err[0], a0.flex_deg, a0.off_axis_deg
                    );
                }
            }

            // Flip warning: the annotated frame draws the detector's min-error candidate
            // (candidates[0]). When that candidate jumps far from this joint's stable track (the
            // previous smoothed rotation), the planar-ambiguity flip is showing in the annotated
            // view. Warn so the flip is visible + logged; the skeleton itself follows the selected
            // candidate, which temporal continuity keeps stable.
            if (curr_j_pose_candidates.n == 2 && 
                curr_j_fstate.last_out.has_value())
            {
                constexpr double kFlipWarnDeg = 60.0; // one-frame jump beyond any real joint motion
                const double min_err_jump_deg =
                    curr_j_fstate.last_out->angularDistance(rot_of(curr_j_pose_candidates.transform[0])) * kRadToDeg;
                
                if (min_err_jump_deg > kFlipWarnDeg)
                {
                    spdlog::warn(
                        "[flip] {}: min-error pose jumped {:.0f} deg from track "
                        "(planar-ambiguity flip; skeleton uses candidate#{})",
                        curr_j_info.name, min_err_jump_deg, selected_candidate_idx
                    );
                }
            }

            curr_j_state.view_pose = curr_j_pose_candidates.transform[selected_candidate_idx]; // reflect the selection in the raw (absolute) view
            curr_j_state.selected_candidate = static_cast<int>(selected_candidate_idx); // diagnostic: record which candidate won
            Eigen::Quaterniond q = rot_of(curr_j_pose_candidates.transform[selected_candidate_idx]);

            // A quaternion and its negation encode the same rotation (double-cover), so a
            // tag pose can report q on one frame and -q on the next while the joint barely
            // moved. That sign flip would surface as a large jump in the smoother, the
            // time-series plots, and the broadcast stream. Flip the sample into the same
            // hemisphere as the last emitted rotation to keep the stream sign-continuous.
            if (curr_j_fstate.last_out.has_value() && curr_j_fstate.last_out->dot(q) < 0.0) {
                q.coeffs() *= -1.0;
            }

            // Reseed on cold start, after a long gap, or when smoothing is disabled
            // (disabled -> raw passthrough while keeping the kernel synced for re-enable).
            // A reseed adopts the selected candidate with no prior track to validate it against,
            // which is the seed path the TODO(physical-gate) above is meant to protect.
            if (!_opt.enable_smoothing || !curr_j_fstate.last_out.has_value() || (t - curr_j_fstate.last_seen) > _opt.reset_gap) {
                curr_j_fstate.smoother->reset(q);
                curr_j_fstate.last_out = q;
            } else {
                const seconds_f64 dt = std::clamp(seconds_f64{ t - curr_j_fstate.t_prev }, _opt.dt_min, _opt.dt_max);
                curr_j_fstate.last_out = curr_j_fstate.smoother->filter(q, dt.count());
            }
            curr_j_fstate.t_prev = t;
            curr_j_fstate.last_seen = t;
            curr_j_state.global_rot = curr_j_fstate.last_out;
            _ctx->last_frame_detection_flags[curr_j_idx] = true;
        } // for

        // ----- Pass 3: relative + animation rotations + leg-hinge constraint -----
        // kJointsInfo is parent-before-child ordered, so a single forward pass suffices.
        for (const auto& curr_j_info : kJointsInfo)
        {
            const size_t curr_j_idx = index_of(curr_j_info.id);
            joint_state_t& curr_j_state = _ctx->last_frame_joint_states[curr_j_idx];
            if (!curr_j_state.global_rot.has_value()) { continue; }

            if (is_locked_pelvis_joint(curr_j_info.id)) {
                curr_j_state.local_rot = curr_j_state.global_rot; // constant camera-frame orientation
                // Locked pelvis is a fixed base: its animation contribution is identity.
                curr_j_state.local_anim_rot = Eigen::Quaterniond::Identity();
                continue;
            }

            if (is_root_joint(curr_j_info.id)) {
                curr_j_state.local_rot = curr_j_state.global_rot;
            } else {
                const joint_state_t& parent_j_state = _ctx->last_frame_joint_states[index_of(curr_j_info.parent)];
                if (parent_j_state.global_rot.has_value()) {
                    curr_j_state.local_rot = q_relative(
                        parent_j_state.global_rot.value(),
                        curr_j_state.global_rot.value()
                    ); // fresh or held
                }
                // parent lost beyond the hold window -> local_rot stays nullopt (cascade, documented)
            }

            if (!curr_j_state.local_rot.has_value()) {
                continue;
            }

            const std::optional<Eigen::Quaterniond> curr_j_rest_rot = rest_rotation_of(curr_j_info.id);
            curr_j_state.local_anim_rot = curr_j_state.local_rot.value(); // no rest captured == identity rest
            if (curr_j_rest_rot.has_value()) {
                curr_j_state.local_anim_rot = q_relative(
                    curr_j_rest_rot.value(), 
                    curr_j_state.local_rot.value()
                );
            }

            // Constrain the joint to its leg-hinge axis: 
            // the joint physically rotates only about this axis, 
            // so keep the rotation about it and drop the off-axis part, which is tag-pose error.
            if (_opt.enable_hinge_constraint && curr_j_rest_rot.has_value() && !is_root_joint(curr_j_info.id)) {
                curr_j_state.local_anim_rot = decompose_swing_twist_about_axis(curr_j_state.local_anim_rot.value(), kExoHingeLocalAxis).twist;
                curr_j_state.local_rot = curr_j_rest_rot.value() * curr_j_state.local_anim_rot.value(); // keep serialized rotations consistent
            }
        } // for
    }

    bool exo_pose_estimator::calibrate_rest_pose()
    {
        std::array<std::optional<Eigen::Quaterniond>, kNumJoints> new_rest_pose{};

        bool any = false;
        for (size_t i = 0; i < kNumJoints; ++i) {
            // Only latch freshly detected joints; a held joint's local_rot comes
            // from a frozen global rotation and would capture a stale rest reference.
            if (_ctx->last_frame_detection_flags[i]) {
                new_rest_pose[i] = _ctx->last_frame_joint_states[i].local_rot;
            }
            any = any || new_rest_pose[i].has_value();
        }

        if (any) { _ctx->rest_pose = new_rest_pose; }
        else { _ctx->rest_pose.reset(); }

        return any;
    }

    void exo_pose_estimator::clear_rest_pose()
    {
        _ctx->rest_pose.reset();
    }

} // namespace pose
