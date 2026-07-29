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

        json q_json(const Eigen::Quaterniond& q) { return json::array({ q.w(), q.x(), q.y(), q.z() }); }
        json v3_json(const Eigen::Vector3d& v) { return json::array({ v.x(), v.y(), v.z() }); }
        json v2_json(const Eigen::Vector2f& v) { return json::array({ v.x(), v.y() }); }

        json opt_v3_json(const std::optional<Eigen::Vector3d>& v) { return v.has_value() ? v3_json(v.value()) : json(nullptr); }
        json opt_q_json(const std::optional<Eigen::Quaterniond>& q) { return q.has_value() ? q_json(q.value()) : json(nullptr); }

        // Chosen camera-space position of a detection: the selected pose translation, else candidate[0].
        std::optional<Eigen::Vector3d> detection_position(const pose::tag_detection_t& det)
        {
            if (det.pose.has_value()) { return det.pose->transform.translation(); }
            if (det.num_pose_candidates > 0) { return det.pose_candidates[0].transform.translation(); }
            return std::nullopt;
        }
    } // namespace

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
        f.smoothing_enabled = opt.enable_position_smoothing;
        f.hinge_enabled = opt.enable_hinge_constraint;
        f.max_hold_ms = opt.max_hold.count();
        f.reset_gap_ms = opt.reset_gap.count();
        f.hinge_axis = opt.hinge_axis_world;

        // --- detections (chosen camera-space position per tag) ---
        std::array<bool, pose::kNumJoints> tag_present{};
        f.detections.reserve(detections.size());
        for (const auto& det : detections)
        {
            detection_rec_t d;
            d.id = det.id;
            d.hamming = det.hamming;
            d.decision_margin = det.decision_margin;
            d.center = { det.center.x, det.center.y };
            for (std::size_t k = 0; k < 4; ++k) { d.corners[k] = { det.corners[k].x, det.corners[k].y }; }
            d.joint_id = pose::tag_to_joint(det.id);
            d.position = detection_position(det);
            if (d.joint_id.has_value() && d.position.has_value()) {
                tag_present[static_cast<std::size_t>(d.joint_id.value())] = true;
            }
            f.detections.push_back(std::move(d));
        }

        // --- per-joint positions + IK anim + detected/held/lost + rest position ---
        for (const auto& info : pose::kJointsInfo)
        {
            const std::size_t ji = static_cast<std::size_t>(info.id);
            const pose::joint_state_t& st = estimator.get_joint_state(info.id);
            joint_rec_t& jr = f.joints[ji];

            const bool has_position = st.position.has_value();
            jr.detected = tag_present[ji] && has_position;
            jr.held = !tag_present[ji] && has_position;
            jr.lost = !has_position;
            jr.raw_position = st.raw_position;
            jr.position = st.position;
            jr.local_anim_rot = st.local_anim_rot;
            f.rest_position[ji] = estimator.get_rest_position(info.id);
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
        root["schema"] = "exo-pose-trace/v3";
        root["notes"] = "Positions are [x,y,z] in meters, tag->camera (camera frame: X right, Y down, "
                        "Z forward/depth). Quaternions are [w,x,y,z]. local_anim_rot is the per-joint "
                        "IK animation rotation (parent-relative, vs the captured rest).";

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

        // rig table (data-driven; mirrors kJointsInfo)
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
                { "smoothing_enabled", f.smoothing_enabled },
                { "hinge_enabled", f.hinge_enabled },
                { "max_hold_ms", f.max_hold_ms },
                { "reset_gap_ms", f.reset_gap_ms },
                { "hinge_axis", v3_json(f.hinge_axis) },
            };

            json rest = json::object();
            for (std::size_t i = 0; i < pose::kNumJoints; ++i) { rest[joint_name(i)] = opt_v3_json(f.rest_position[i]); }
            jf["rest_position"] = std::move(rest);

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
                jd["joint"] = d.joint_id.has_value() ? json(joint_name(static_cast<std::size_t>(d.joint_id.value()))) : json(nullptr);
                jd["position"] = opt_v3_json(d.position);
                dets.push_back(std::move(jd));
            }
            jf["detections"] = std::move(dets);

            json joints = json::object();
            for (std::size_t i = 0; i < pose::kNumJoints; ++i)
            {
                const joint_rec_t& jr = f.joints[i];
                json jj;
                jj["status"] = jr.lost ? "lost" : (jr.held ? "held" : "detected");
                jj["raw_position"] = opt_v3_json(jr.raw_position);
                jj["position"] = opt_v3_json(jr.position);
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
