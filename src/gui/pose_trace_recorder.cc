#include "pose_trace_recorder.hh"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <fstream>
#include <utility>

namespace gui
{
    namespace
    {
        using json = nlohmann::json;

        // --- small serializers (quaternion as [w,x,y,z], vector as [..]) -----------------------

        json q_json(const Eigen::Quaterniond& q)
        {
            return json::array({ q.w(), q.x(), q.y(), q.z() });
        }

        json v3_json(const Eigen::Vector3d& v)
        {
            return json::array({ v.x(), v.y(), v.z() });
        }

        json v2_json(const Eigen::Vector2f& v)
        {
            return json::array({ v.x(), v.y() });
        }

        // std::optional<quat> -> [w,x,y,z] or null.
        json opt_q_json(const std::optional<Eigen::Quaterniond>& q)
        {
            return q.has_value() ? q_json(q.value()) : json(nullptr);
        }

    } // namespace

    // ------------------------------------------------------------------------------------------

    pose_trace_recorder::pose_trace_recorder(std::size_t capacity)
        : _capacity{ capacity == 0 ? 1 : capacity }
    {
    }

    void pose_trace_recorder::set_capacity(std::size_t capacity)
    {
        _capacity = (capacity == 0 ? 1 : capacity);
        while (_frames.size() > _capacity) { _frames.pop_front(); }
    }

    void pose_trace_recorder::capture(
        std::chrono::microseconds sensor_ts,
        std::span<const pose::tag_detection_t> detections,
        const pose::exo_pose_estimator& estimator)
    {
        const auto& opt = estimator.options();

        frame_rec_t f;
        f.seq = _next_seq++;
        f.t = sensor_ts;
        f.has_rest_pose = estimator.has_rest_pose();
        f.hinge_enabled = opt.enable_hinge_constraint;
        f.smoothing_enabled = opt.enable_smoothing;
        f.max_hold_ms = opt.max_hold.count();
        f.reset_gap_ms = opt.reset_gap.count();

        // --- raw detections (both pose candidates + the detector's chosen pose) ----------------
        f.detections.reserve(detections.size());
        for (const auto& det : detections)
        {
            detection_rec_t d;
            d.id = det.id;
            d.hamming = det.hamming;
            d.decision_margin = det.decision_margin;
            d.center = { det.center.x, det.center.y };
            for (std::size_t k = 0; k < 4; ++k) { d.corners[k] = { det.corners[k].x, det.corners[k].y }; }
            d.joint = pose::tag_to_joint(det.id);
            d.num_candidates = det.num_pose_candidates;

            const auto to_pose_rec = [](const pose::tag_pose_t& p) {
                pose_rec_t r;
                r.obj_err = p.obj_err;
                r.q = Eigen::Quaterniond{ p.transform.rotation() }.normalized();
                r.t = p.transform.translation();
                return r;
            };
            for (int k = 0; k < det.num_pose_candidates && k < 2; ++k) {
                d.candidates[k] = to_pose_rec(det.pose_candidates[k]);
            }
            if (det.pose.has_value()) { d.chosen = to_pose_rec(det.pose.value()); }

            f.detections.push_back(std::move(d));
        }

        // --- per-joint outputs + rest reference + detected/held/lost classification ------------
        // A joint is "detected" when a bound tag carried >=1 pose candidate this frame; from that
        // and whether a global rotation came out we separate a fresh solve from a held/lost one.
        std::array<bool, pose::kNumJoints> tag_present{};
        for (const auto& d : f.detections) {
            if (d.joint.has_value() && d.num_candidates > 0) {
                tag_present[static_cast<std::size_t>(d.joint.value())] = true;
            }
        }

        for (const auto& info : pose::kJointsInfo)
        {
            const std::size_t ji = static_cast<std::size_t>(info.id);
            const pose::joint_state_t& st = estimator.get_joint_state(info.id);
            joint_rec_t& jr = f.joints[ji];

            jr.selected_candidate = st.selected_candidate;
            jr.locked_pelvis = f.hinge_enabled && pose::is_root_joint(info.id)
                && estimator.rest_rotation(info.id).has_value();

            const bool has_global = st.global_rot.has_value();
            jr.detected = tag_present[ji] && has_global;
            jr.held = !tag_present[ji] && has_global;
            jr.lost = !has_global;

            if (st.view_pose.has_value()) {
                jr.view_q = Eigen::Quaterniond{ st.view_pose.value().rotation() }.normalized();
                jr.view_t = st.view_pose.value().translation();
            }
            jr.global_rot = st.global_rot;
            jr.local_rot = st.local_rot;
            jr.local_anim_rot = st.local_anim_rot;

            f.rest_rot[ji] = estimator.rest_rotation(info.id);
        }

        _frames.push_back(std::move(f));
        while (_frames.size() > _capacity) { _frames.pop_front(); }
    }

    bool pose_trace_recorder::write_json(
        const std::filesystem::path& path,
        const std::string& source_name,
        const Eigen::Vector2i& source_resolution,
        float source_fps,
        const std::optional<hw::intrinsic_t>& intrinsics) const
    {
        json root;
        root["schema"] = "exo-pose-trace/v1";
        root["notes"] = "Quaternions are [w,x,y,z]; translations [x,y,z] in meters; poses are tag->camera. "
                        "candidates[] are the raw orthogonal-iteration solutions, obj_err ascending "
                        "(index 1 is the planar-ambiguity flip when present).";

        // --- static source context ------------------------------------------------------------
        root["source"] = {
            { "name", source_name },
            { "width", source_resolution.x() },
            { "height", source_resolution.y() },
            { "fps", source_fps },
        };
        if (intrinsics.has_value()) {
            const auto& k = intrinsics.value();
            root["intrinsics"] = {
                { "fx", k.fx }, { "fy", k.fy }, { "cx", k.cx }, { "cy", k.cy },
                { "width", k.width }, { "height", k.height },
            };
        } else {
            root["intrinsics"] = nullptr;
        }

        // --- rig table (data-driven; mirrors kJointsInfo) -------------------------------------
        root["hinge_axis_local"] = v3_json(pose::kExoHingeLocalAxis);
        json rig = json::array();
        for (const auto& info : pose::kJointsInfo) {
            rig.push_back({
                { "index", static_cast<int>(info.id) },
                { "joint", std::string{ info.name } },
                { "tag_id", info.tag_id },
                { "parent", std::string{ pose::joint_info(info.parent).name } },
                { "is_root", pose::is_root_joint(info.id) },
            });
        }
        root["rig"] = std::move(rig);

        // --- per-frame trace ------------------------------------------------------------------
        const auto joint_name = [](std::size_t i) { return std::string{ pose::kJointsInfo[i].name }; };

        json frames = json::array();
        for (const auto& f : _frames)
        {
            json jf;
            jf["seq"] = f.seq;
            jf["t_us"] = f.t.count();
            jf["t_s"] = std::chrono::duration<double>{ f.t }.count();
            jf["gates"] = {
                { "has_rest_pose", f.has_rest_pose },
                { "hinge_enabled", f.hinge_enabled },
                { "smoothing_enabled", f.smoothing_enabled },
                { "max_hold_ms", f.max_hold_ms },
                { "reset_gap_ms", f.reset_gap_ms },
            };

            // rest reference in effect this frame (per joint; null where uncomputed)
            json rest = json::object();
            for (std::size_t i = 0; i < pose::kNumJoints; ++i) {
                rest[joint_name(i)] = opt_q_json(f.rest_rot[i]);
            }
            jf["rest_pose"] = std::move(rest);

            // raw detections
            json dets = json::array();
            for (const auto& d : f.detections)
            {
                json jd;
                jd["id"] = d.id;
                jd["hamming"] = d.hamming;
                jd["decision_margin"] = d.decision_margin;
                jd["center"] = v2_json(d.center);
                json corners = json::array();
                for (const auto& c : d.corners) { corners.push_back(v2_json(c)); }
                jd["corners"] = std::move(corners);
                jd["joint"] = d.joint.has_value() ? json(joint_name(static_cast<std::size_t>(d.joint.value())))
                                                  : json(nullptr);
                jd["num_candidates"] = d.num_candidates;

                json cands = json::array();
                for (int k = 0; k < d.num_candidates && k < 2; ++k) {
                    cands.push_back({
                        { "obj_err", d.candidates[k].obj_err },
                        { "q", q_json(d.candidates[k].q) },
                        { "t", v3_json(d.candidates[k].t) },
                    });
                }
                jd["candidates"] = std::move(cands);
                jd["chosen_pose"] = d.chosen.has_value()
                    ? json{ { "q", q_json(d.chosen->q) }, { "t", v3_json(d.chosen->t) } }
                    : json(nullptr);

                dets.push_back(std::move(jd));
            }
            jf["detections"] = std::move(dets);

            // per-joint decisions + outputs
            json joints = json::object();
            for (std::size_t i = 0; i < pose::kNumJoints; ++i)
            {
                const joint_rec_t& jr = f.joints[i];
                json jj;
                jj["status"] = jr.lost ? "lost" : (jr.held ? "held" : "detected");
                jj["locked_pelvis"] = jr.locked_pelvis;
                jj["selected_candidate"] = jr.selected_candidate;
                jj["view_q"] = opt_q_json(jr.view_q);
                jj["view_t"] = jr.view_t.has_value() ? v3_json(jr.view_t.value()) : json(nullptr);
                jj["global_rot"] = opt_q_json(jr.global_rot);
                jj["local_rot"] = opt_q_json(jr.local_rot);
                jj["local_anim_rot"] = opt_q_json(jr.local_anim_rot);
                joints[joint_name(i)] = std::move(jj);
            }
            jf["joints"] = std::move(joints);

            frames.push_back(std::move(jf));
        }
        root["frame_count"] = frames.size();
        root["frames"] = std::move(frames);

        try {
            std::ofstream os{ path, std::ios::binary | std::ios::trunc };
            if (!os) {
                spdlog::error("pose trace: cannot open '{}' for writing", path.string());
                return false;
            }
            os << root.dump(2);
            if (!os) {
                spdlog::error("pose trace: write to '{}' failed", path.string());
                return false;
            }
        }
        catch (const std::exception& e) {
            spdlog::error("pose trace: serialization failed: {}", e.what());
            return false;
        }

        spdlog::info("pose trace: wrote {} frames to '{}'", _frames.size(), path.string());
        return true;
    }

} // namespace gui
