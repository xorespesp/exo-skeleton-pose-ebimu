#include "pose_plot_panel.hh"
#include "pose/hinge_angle.hh"

#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <limits>
#include <stdexcept>
#include <string>

namespace gui
{
    namespace
    {
        constexpr float kGridPlotWindowSec = 6.0f; // subplot-grid x-axis scroll span [s]
        constexpr float kViewComboWidth = 210.0f;  // toolbar picker width [px]

        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

        // Rig frame is X-right, Y-down, Z-depth; ImPlot3D's is X-right, Y-depth, Z-up. So
        // (x, y, z) -> (x, z, -y): a mirror-free rotation, rig-down becoming plot-up.
        Eigen::Vector3f rig_to_display(const Eigen::Vector3d& p)
        {
            return Eigen::Vector3f(static_cast<float>(p.x()), static_cast<float>(p.z()), static_cast<float>(-p.y()));
        }

        // A readable 3/4 front view. ImPlot3D projects with screen-up = (Rotation*p).y, so bringing
        // the plot's up axis (Z) onto screen-up needs the -90 deg turn about X.
        ImPlot3DQuat front_view_quat()
        {
            constexpr double pi = 3.14159265358979323846;
            const double d = pi / 180.0;
            return ImPlot3DQuat(15.0 * d, ImPlot3DPoint(1, 0, 0))   // pitch: look slightly down
                 * ImPlot3DQuat(-pi / 2, ImPlot3DPoint(1, 0, 0))    // base: up -> screen up
                 * ImPlot3DQuat(20.0 * d, ImPlot3DPoint(0, 0, 1));  // yaw about up: slight 3/4
        }

        // Fixed-length T-pose lower limb in rig space, what the rig view drives by `local_anim_rot`.
        // The robot faces the camera, so its right leg sits on camera-left (-X). The display rig
        // gives each hip its lateral offset, so a thigh hangs straight down from its own hip and a
        // hip rotation reads as a clean swing about that point.
        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> canonical_rest_layout()
        {
            constexpr double hip = 0.18, thigh = 0.45, shin = 0.42, foot = 0.16; // bone lengths [m]
            const auto ankle_def = pose::get_joint_def(pose::joint_id_t::r_ankle);
            const double ankle_rad = ankle_def ? ankle_def->sagittal_clinical_neutral_offset_rad : 0.0;
            const double foot_y = foot * std::cos(ankle_rad);
            const double foot_z = -foot * std::sin(ankle_rad);

            const auto at = [](pose::joint_id_t j) { return static_cast<std::size_t>(j); };
            std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> r{};
            r[at(pose::joint_id_t::pelvis)]  = Eigen::Vector3d{ 0.0, 0.0, 0.0 };
            r[at(pose::joint_id_t::r_hip)]   = Eigen::Vector3d{ -hip, 0.0, 0.0 };
            r[at(pose::joint_id_t::l_hip)]   = Eigen::Vector3d{ +hip, 0.0, 0.0 };
            r[at(pose::joint_id_t::r_knee)]  = Eigen::Vector3d{ -hip, thigh, 0.0 };
            r[at(pose::joint_id_t::l_knee)]  = Eigen::Vector3d{ +hip, thigh, 0.0 };
            r[at(pose::joint_id_t::r_ankle)] = Eigen::Vector3d{ -hip, thigh + shin, 0.0 };
            r[at(pose::joint_id_t::l_ankle)] = Eigen::Vector3d{ +hip, thigh + shin, 0.0 };
            r[at(pose::joint_id_t::r_foot)]  = Eigen::Vector3d{ -hip, thigh + shin + foot_y, foot_z };
            r[at(pose::joint_id_t::l_foot)]  = Eigen::Vector3d{ +hip, thigh + shin + foot_y, foot_z };
            return r;
        }

        // Forward-kinematics the rig, walking `get_joint_defs()` (parent precedes child):
        //   world_rot = parent_world_rot * anim
        //   world_pos = parent_world_pos + parent_world_rot * (rest[joint] - rest[parent])
        // A joint's rotation turns the bones leaving it, so the offset up to a joint is turned by
        // its parent's world rotation, which already carries the parent's own anim.
        // Missing anim keeps the rest orientation; missing rest (joint or ancestor) yields no position.
        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> rig_fk(
            const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>& rest,
            const std::array<std::optional<Eigen::Quaterniond>, pose::kNumJoints>& anim,
            const Eigen::Vector3d& root_anchor)
        {
            std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> pos{};
            std::array<Eigen::Quaterniond, pose::kNumJoints> world_rot{};
            for (const auto& def : pose::get_joint_defs())
            {
                const std::size_t j = static_cast<std::size_t>(def.joint_id);
                const Eigen::Quaterniond a = anim[j].value_or(Eigen::Quaterniond::Identity());
                if (pose::is_root_joint(def.joint_id))
                {
                    world_rot[j] = a.normalized();
                    pos[j] = root_anchor;
                    continue;
                }
                const std::size_t p = static_cast<std::size_t>(def.parent);
                if (!pos[p].has_value() || !rest[j].has_value() || !rest[p].has_value()) { continue; }
                world_rot[j] = (world_rot[p] * a).normalized();
                pos[j] = pos[p].value() + world_rot[p] * (rest[j].value() - rest[p].value());
            }
            return pos;
        }

        // One subplot, drawn zero-copy from the buffer's strided view over the newest `window`
        // seconds. `colors` / `names` are indexed by channel, so one buffer labels several sets of
        // channels in place. The caller wraps each subplot in PushID/PopID for unique ids.
        template <typename _Scalar>
        void draw_plot_lines(
            const char* title,
            const plot_buffer_view_t<_Scalar>& v,
            float window,
            float y_lo,
            float y_hi,
            ImPlotCond y_cond,
            bool sync,
            double* sy,
            const ImVec4* colors,
            const char* const* names,
            const ImVec2& size,
            std::size_t first_channel,
            std::size_t channel_count)
        {
            if (!ImPlot::BeginPlot(title, size, ImPlotFlags_None)) { return; }

            ImPlot::SetupAxes(nullptr, nullptr, 0, 0);
            ImPlot::SetupLegend(ImPlotLocation_NorthWest);
            if (sync) { ImPlot::SetupAxisLinks(ImAxis_Y1, &sy[0], &sy[1]); } // sync y only
            ImPlot::SetupAxisLimits(ImAxis_X1, v.t_hi - window, v.t_hi, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, y_lo, y_hi, y_cond);

            // An empty view has no channels, which leaves the plot drawn but blank.
            const std::size_t last_channel = std::min(first_channel + channel_count, v.ys.size());
            for (std::size_t k = first_channel; k < last_channel; ++k)
            {
                ImPlotSpec spec;
                spec.LineColor = colors[k];
                spec.LineWeight = 2.0f;
                spec.Offset = v.offset;
                spec.Stride = v.stride;
                ImPlot::PlotLine(names[k], v.xs, v.ys.data() + k, v.count, spec);
            }
            ImPlot::EndPlot();
        }

        // `g.reset` is a one-shot raised here and cleared once the grid has drawn with the default
        // range forced.
        void draw_grid_range_controls(grid_plot_ui_t& g)
        {
            ImGui::Checkbox("Lock", &g.lock);
            ImGui::SetItemTooltip("Hold every subplot at its default Y range (no mouse pan/zoom on Y).\n"
                                  "On: ranges stay put, live. Off: Y is mouse-adjustable.");
            ImGui::SameLine();
            if (ImGui::Checkbox("Sync", &g.sync)) { g.reset = true; }
            ImGui::SetItemTooltip("Share one Y range across all joint subplots so they compare directly.");
            ImGui::SameLine();
            if (ImGui::Button("Reset")) { g.reset = true; }
            ImGui::SetItemTooltip("Return every subplot to its default Y range now.");
        }

        void draw_grid_layout_controls(grid_plot_ui_t& g)
        {
            ImGui::Checkbox("Auto-size Plots", &g.autosize);
            ImGui::SetItemTooltip("On: pack the subplots to fill the panel. Off: use a fixed cell size.");
            if (!g.autosize) {
                ImGui::SliderFloat("Plots Size", &g.size_px, 80.0f, 400.0f, "%.0f px");
            }
        }

        // One square subplot per rig joint, packed to fill the panel or laid out at a fixed cell size.
        template <typename _GetLabel, typename _GetView>
        void draw_joint_plot_grid(
            grid_plot_ui_t& g,
            float dpi_scale,
            float y_lo, float y_hi,
            ImPlotCond y_cond,
            const ImVec4* colors,
            const char* const* names,
            std::size_t first_channel,
            std::size_t channel_count,
            _GetLabel&& get_label, // what each subplot shows above itself; its id stays the joint's
            _GetView&& get_view)
        {
            const int n = static_cast<int>(pose::kNumJoints);
            const float spacing = ImGui::GetStyle().ItemSpacing.x;
            const ImVec2 avail = ImGui::GetContentRegionAvail();

            int cols = 1;
            float cell = 1.0f;
            if (g.autosize) {
                for (int c = 1; c <= n; ++c) {
                    const int r = (n + c - 1) / c;
                    const float cw = (avail.x - spacing * (c - 1)) / c;
                    const float ch = (avail.y - spacing * (r - 1)) / r;
                    if (const float s = std::min(cw, ch); s > cell) { cell = s; cols = c; }
                }
            } else {
                cell = g.size_px * dpi_scale; // DPI-aware px
                cols = std::max(1, static_cast<int>((avail.x + spacing) / (cell + spacing)));
            }
            const ImVec2 cell_sz{ cell, cell };

            int col = 0;
            for (std::size_t i = 0; i < pose::kNumJoints; ++i)
            {
                const auto name = pose::get_joint_name(static_cast<pose::joint_id_t>(i));
                const std::string title = std::format("{}###{}", get_label(i), name);
                if (col != 0) { ImGui::SameLine(); }
                ImGui::PushID(static_cast<int>(i));
                ImGui::BeginGroup();
                draw_plot_lines(title.c_str(), get_view(i), kGridPlotWindowSec, y_lo, y_hi,
                           y_cond, g.sync, g.sync_y, colors, names, cell_sz,
                           first_channel, channel_count);
                ImGui::EndGroup();
                ImGui::PopID();
                if (++col >= cols) { col = 0; }
            }
        }

    } // namespace

    void pose_plot_panel::push(const pose::pose_estimator_base& est, double t_now)
    {
        _pos_plot_buffers.advance(t_now);
        _angle_plot_buffers.advance(t_now);

        // Smoothed+held when the estimator smooths, raw when it does not.
        const bool smoothed_positions = est.uses_smoothed_positions();

        // Running total of the rotation-derived flexion down each leg, which is the rig-frame
        // reading. `get_joint_defs()` lists a parent before its children, so one forward pass fills it.
        std::array<double, pose::kNumJoints> quat_angle_sum{};

        int ji = 0;
        for (const auto& def : pose::get_joint_defs())
        {
            const auto& st = est.get_joint_state(def.joint_id);
            const std::optional<Eigen::Vector3d> p = smoothed_positions ? st.position : st.raw_position;
            _raw_skel_positions[ji] = p;
            if (p.has_value()) { _pos_plot_buffers.push(ji, rig_to_display(p.value())); }

            const bool has_rot = st.local_anim_rot.has_value();
            const double quat_angle = has_rot
                ? pose::quat_hinge_angle(st.local_anim_rot.value(), pose::pose_estimator_base::kRigLateralAxis)
                : 0.0;
            quat_angle_sum[ji] = pose::is_root_joint(def.joint_id)
                ? quat_angle : quat_angle_sum[static_cast<std::size_t>(def.parent)] + quat_angle;

            // The measured angles and the rest-relative motion fill and empty independently (the
            // measurements need no rest pose), so a sample is taken whenever anything was
            // produced, and a channel the estimator left empty holds NaN, which the plot draws
            // as a gap.
            const bool has_any = st.sagittal_clinical_angle.has_value() || st.sagittal_segment_angle.has_value()
                || st.sagittal_clinical_angle_delta.has_value() || st.sagittal_segment_angle_delta.has_value()
                || has_rot;
            if (has_any)
            {
                constexpr float no_value = std::numeric_limits<float>::quiet_NaN();
                const auto deg_or_nan = [](const std::optional<double>& a) {
                    return a.has_value() ? static_cast<float>(a.value() * kRadToDeg) : no_value;
                };

                // The rest-relative bend read back out of the rotation, carried into the
                // clinical sign so the two lines overlay; joints without a clinical angle have
                // nothing to overlay. The chain total plays the same role for the segment delta.
                const std::optional<double> quat_clinical_delta = has_rot
                    ? pose::sagittal_clinical_angle_delta_from_rig_bend_delta(def.joint_id, quat_angle)
                    : std::nullopt;

                angle_sample_t v;
                v[0] = deg_or_nan(st.sagittal_clinical_angle);
                v[1] = deg_or_nan(st.sagittal_segment_angle);
                v[2] = deg_or_nan(st.sagittal_clinical_angle_delta);
                v[3] = deg_or_nan(quat_clinical_delta);
                v[4] = deg_or_nan(st.sagittal_segment_angle_delta);
                v[5] = has_rot
                    ? static_cast<float>(pose::sagittal_segment_angle_from_rig(quat_angle_sum[ji]) * kRadToDeg)
                    : no_value;
                v[6] = deg_or_nan(st.sagittal_included_angle);
                _angle_plot_buffers.push(ji, v);
                _latest_angles[ji] = v;
            }
            ++ji;
        }
    }

    void pose_plot_panel::reset()
    {
        _pos_plot_buffers.clear();
        _angle_plot_buffers.clear();
        _latest_angles = {};
        _raw_skel_positions = {};
        _autofit_frames = kAutofitFrames;
    }

    void pose_plot_panel::render(const pose::pose_estimator_base* est, float dpi_scale)
    {
        this->_render_toolbar();

        switch (_plot_type) {
        case plot_type_t::raw_skeleton:  this->_render_raw_skeleton_plot(est); break;
        case plot_type_t::rig_skeleton:  this->_render_rig_skeleton_plot(est); break;
        case plot_type_t::positions:     this->_render_positions_plot(dpi_scale); break;
        case plot_type_t::sagittal_angles:  this->_render_sagittal_angles_plot(dpi_scale); break;
        default: throw std::runtime_error{ "unknown plot type" };
        }
    }

    void pose_plot_panel::_render_toolbar()
    {
        constexpr std::array<const char*, 4> plot_types{ "Raw Skeleton", "Rig Skeleton", "Positions", "Sagittal Angles" };

        ImGui::SetNextItemWidth(kViewComboWidth);
        if (ImGui::BeginCombo("##plot_type", plot_types[static_cast<int>(_plot_type)])) {
            for (std::size_t i = 0; i < plot_types.size(); ++i) {
                const plot_type_t curr_plot_type = static_cast<plot_type_t>(i);
                const bool selected = (curr_plot_type == _plot_type);
                if (ImGui::Selectable(plot_types[i], selected) && !selected) {
                    _plot_type = curr_plot_type;
                    _autofit_frames = kAutofitFrames; // reframe the 3D box for the new view
                }
                if (selected) { ImGui::SetItemDefaultFocus(); }
            }
            ImGui::EndCombo();
        }
        ImGui::SetItemTooltip("Raw Skeleton: Measured raw skeleton (+ FK reconstruction overlay).\n"
                              "Rig Skeleton: Fixed-length T-pose leg rig driven by local_anim_rot.\n"
                              "Positions: Per-joint XYZ position channels as 2D line plots.\n"
                              "Sagittal Angles: Per-joint angles [deg] as 2D line plots, in the\n"
                              "                 conventions of docs/joint_angle_convention.md.");

        switch (_plot_type) {
        case plot_type_t::raw_skeleton:
        case plot_type_t::rig_skeleton:
            ImGui::SameLine();
            if (ImGui::Button("Fit view")) { _autofit_frames = kAutofitFrames; }
            ImGui::SetItemTooltip("Re-center/zoom the 3D view to the current skeleton.\n"
                                  "Zoom (wheel) / pan / rotate are otherwise free.");
            break;

        case plot_type_t::positions:
            ImGui::SameLine();
            draw_grid_range_controls(_pos_plot_grid);
            break;

        case plot_type_t::sagittal_angles:
        {
            constexpr std::array<const char*, 5> angle_views{
                "Clinical", "Segment", "Included", "Clinical vs rest", "Segment vs rest"
            };
            ImGui::SameLine();
            ImGui::SetNextItemWidth(kViewComboWidth);
            if (ImGui::BeginCombo("##angle_view", angle_views[static_cast<int>(_angle_view)])) {
                for (std::size_t i = 0; i < angle_views.size(); ++i) {
                    const auto view = static_cast<angle_view_t>(i);
                    const bool selected = (view == _angle_view);
                    if (ImGui::Selectable(angle_views[i], selected) && !selected) {
                        _angle_view = view;
                        _angle_plot_grid.reset = true; // reframe Y: the views sit in different bands
                    }
                    if (selected) { ImGui::SetItemDefaultFocus(); }
                }
                ImGui::EndCombo();
            }
            ImGui::SetItemTooltip("Which angle the traces draw, in the conventions of\n"
                                  "docs/joint_angle_convention.md. The first three are the protocol's\n"
                                  "three angle fields, shown so what goes on the wire can be checked\n"
                                  "by eye. Every channel is recorded, so switching changes the view\n"
                                  "and keeps each history.\n"
                                  "Clinical: the bend at the joint vs its parent bone\n"
                                  "    (sagittal_clinical_angle), 0 at neutral, flexion/dorsiflexion\n"
                                  "    positive; what a joint encoder reads. No rest pose involved.\n"
                                  "Segment: the bone's attitude from vertically down\n"
                                  "    (sagittal_segment_angle), anterior positive. No rest pose involved.\n"
                                  "Included: the inter-bone angle (sagittal_included_angle), 180 deg\n"
                                  "    when collinear, ankle neutral at 90 deg. No rest pose involved.\n"
                                  "Clinical vs rest: the bend's change since the captured rest, beside\n"
                                  "    the same turn read back out of local_anim_rot; the two split by\n"
                                  "    however far a free solve left the hinge plane.\n"
                                  "Segment vs rest: the attitude's change since the captured rest,\n"
                                  "    beside the chain total of the rotations.");
            ImGui::SameLine();
            draw_grid_range_controls(_angle_plot_grid);
            break;
        }

        default: throw std::runtime_error{ "unknown plot type" };
        }

        ImGui::SameLine();
        if (ImGui::Button("Style...")) { ImGui::OpenPopup("##plot_style"); }
        ImGui::SetItemTooltip("Subplot layout and the 3D skeleton's colours.");
        this->_render_style_popup();

        ImGui::Separator();
    }

    void pose_plot_panel::_render_style_popup()
    {
        if (!ImGui::BeginPopup("##plot_style")) { return; }

        switch (_plot_type) {
        case plot_type_t::raw_skeleton:
        case plot_type_t::rig_skeleton:
        {
            const bool is_raw = (_plot_type == plot_type_t::raw_skeleton);
            skeleton_plot_ui_t& style = is_raw ? _raw_skel : _rig_skel; // the active mode's own style

            constexpr ImGuiColorEditFlags col_flags = ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar;
            ImGui::DragFloat("Sphere size", &style.point_size, 0.1f, 1.0f, 20.0f, "%.1f px", ImGuiSliderFlags_AlwaysClamp);
            ImGui::ColorEdit4("Sphere color", style.point_color, col_flags);
            ImGui::ColorEdit4("Bone color", style.bone_color, col_flags);
            if (is_raw) {
                ImGui::ColorEdit4("FK overlay color", _raw_skel_fk_bone_color, col_flags);
            }
            break;
        }

        case plot_type_t::positions:        draw_grid_layout_controls(_pos_plot_grid); break;
        case plot_type_t::sagittal_angles:  draw_grid_layout_controls(_angle_plot_grid); break;

        default: throw std::runtime_error{ "unknown plot type" };
        }

        ImGui::EndPopup();
    }

    void pose_plot_panel::_render_raw_skeleton_plot(const pose::pose_estimator_base* est)
    {
        const auto to_display = [](const Eigen::Vector3d& p) -> Eigen::Vector3d { return rig_to_display(p).cast<double>(); };

        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> measured{};
        for (std::size_t i = 0; i < pose::kNumJoints; ++i) {
            if (_raw_skel_positions[i].has_value()) { measured[i] = to_display(_raw_skel_positions[i].value()); }
        }

        // Overlay: the anim rotations replayed on the captured rest geometry, anchored at the
        // measured pelvis and kept only where the joint was solved this frame. Where the two split,
        // the rotations no longer describe what was measured; a length-only split is scale or
        // perspective rather than a wrong angle, since the overlay keeps the rest bone lengths.
        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> overlay{};
        bool has_overlay = false;
        if (est && est->has_rest_pose())
        {
            std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> rest{};
            std::array<std::optional<Eigen::Quaterniond>, pose::kNumJoints> anim{};
            std::optional<Eigen::Vector3d> anchor;
            for (const auto& def : pose::get_joint_defs()) {
                const std::size_t k = static_cast<std::size_t>(def.joint_id);
                rest[k] = est->get_rest_position(def.joint_id);
                anim[k] = est->get_joint_state(def.joint_id).local_anim_rot;
                if (pose::is_root_joint(def.joint_id)) { anchor = est->get_joint_state(def.joint_id).position; }
            }
            if (anchor.has_value())
            {
                const auto fk = rig_fk(rest, anim, anchor.value());
                for (const auto& def : pose::get_joint_defs()) {
                    const std::size_t i = static_cast<std::size_t>(def.joint_id);
                    // A joint's overlay point is placed by its parent's rotation (a rotation turns
                    // the bones leaving its joint), so that is the anim whose presence says the
                    // point was solved this frame; the root answers for itself.
                    const std::size_t solver = pose::is_root_joint(def.joint_id)
                        ? i : static_cast<std::size_t>(def.parent);
                    if (fk[i].has_value() && anim[solver].has_value()) {
                        overlay[i] = to_display(fk[i].value());
                        has_overlay = true;
                    }
                }
            }
        }

        const ImVec4 bone_col(_raw_skel.bone_color[0], _raw_skel.bone_color[1], _raw_skel.bone_color[2], _raw_skel.bone_color[3]);
        const ImVec4 point_col(_raw_skel.point_color[0], _raw_skel.point_color[1], _raw_skel.point_color[2], _raw_skel.point_color[3]);
        const ImVec4 fk_col(_raw_skel_fk_bone_color[0], _raw_skel_fk_bone_color[1], _raw_skel_fk_bone_color[2], _raw_skel_fk_bone_color[3]);
        this->_render_skeleton_3d(
            "Raw skeleton (measured positions + FK overlay)",
            measured, bone_col, point_col,
            _raw_skel.point_size,
            has_overlay ? &overlay : nullptr,
            fk_col,
            /*hint*/nullptr
        );
    }

    void pose_plot_panel::_render_rig_skeleton_plot(const pose::pose_estimator_base* est)
    {
        const auto to_display = [](const Eigen::Vector3d& p) -> Eigen::Vector3d { return rig_to_display(p).cast<double>(); };

        // With no rest pose captured every anim rotation is absent, so the rig holds its T-pose.
        const auto rest = canonical_rest_layout();
        std::array<std::optional<Eigen::Quaterniond>, pose::kNumJoints> anim{};
        if (est) {
            for (const auto& def : pose::get_joint_defs()) {
                anim[static_cast<std::size_t>(def.joint_id)] = est->get_joint_state(def.joint_id).local_anim_rot;
            }
        }
        const Eigen::Vector3d anchor =
            rest[static_cast<std::size_t>(pose::get_root_joint())].value_or(Eigen::Vector3d::Zero());
        const auto world = rig_fk(rest, anim, anchor);

        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> disp{};
        for (std::size_t i = 0; i < pose::kNumJoints; ++i) {
            if (world[i].has_value()) { disp[i] = to_display(world[i].value()); }
        }

        // An undriven joint holds the canonical layout, which the hint names.
        // End sites never carry a rotation, so they sit out the count.
        bool any_driven = false, any_still = false;
        for (const auto& def : pose::get_joint_defs()) {
            if (pose::is_root_joint(def.joint_id)) { continue; }
            if (!pose::get_child_joint(def.joint_id).has_value()) { continue; }
            if (anim[static_cast<std::size_t>(def.joint_id)].has_value()) { any_driven = true; }
            else { any_still = true; }
        }

        const char* hint = nullptr;
        if (!est || !est->has_rest_pose()) { hint = "calibrate a rest pose to animate"; }
        else if (any_driven && any_still) { hint = "only the measured leg is driven"; }

        const ImVec4 bone_col(_rig_skel.bone_color[0], _rig_skel.bone_color[1], _rig_skel.bone_color[2], _rig_skel.bone_color[3]);
        const ImVec4 point_col(_rig_skel.point_color[0], _rig_skel.point_color[1], _rig_skel.point_color[2], _rig_skel.point_color[3]);
        this->_render_skeleton_3d(
            "Rig skeleton (local_anim_rot on a fixed-length rig)",
            disp, bone_col, point_col,
            _rig_skel.point_size,
            /*overlay*/nullptr,
            /*overlay_color*/ImVec4{},
            hint
        );
    }

    void pose_plot_panel::_render_positions_plot(float dpi_scale)
    {
        constexpr float kYLo = -1.2f, kYHi = 1.2f; // default position range [m], display space
        const ImPlotCond y_cond = (_pos_plot_grid.lock || _pos_plot_grid.reset) ? ImPlotCond_Always : ImPlotCond_Once;

        // Plot space: rig X, rig Z, and -rig Y, which the 3D views label right / depth / up.
        const ImVec4 axis_col[3]{ { 0.95f, 0.35f, 0.35f, 1 }, { 0.45f, 0.85f, 0.45f, 1 }, { 0.45f, 0.55f, 0.95f, 1 } };
        const char* const axis_nm[3]{ "x", "y", "z" };

        draw_joint_plot_grid(
            _pos_plot_grid, dpi_scale, kYLo, kYHi, y_cond,
            axis_col, axis_nm, /*first_channel*/0, /*channel_count*/3,
            // A position is the joint's own, so the name alone says what the subplot holds.
            [](std::size_t i) { return std::string{ pose::get_joint_name(static_cast<pose::joint_id_t>(i)) }; },
            [this](std::size_t i) { return _pos_plot_buffers.view(i); }
        );

        _pos_plot_grid.reset = false;
    }

    void pose_plot_panel::_render_sagittal_angles_plot(float dpi_scale)
    {
        const ImPlotCond y_cond = (_angle_plot_grid.lock || _angle_plot_grid.reset) ? ImPlotCond_Always : ImPlotCond_Once;

        // On the vs-rest views, "delta" is the estimator's rest-relative reading and "quat" the
        // same motion recovered from `local_anim_rot` (carried into the same convention); the
        // two separate by however far the joint's solved rotation axis sits off the hinge axis.
        // The measured views hold one channel each, being quantities of their own.
        const ImVec4 ch_col[7]{
            { 0.95f, 0.65f, 0.25f, 1 }, { 0.55f, 0.85f, 0.45f, 1 },
            { 0.95f, 0.65f, 0.25f, 1 }, { 0.35f, 0.75f, 0.90f, 1 },
            { 0.55f, 0.85f, 0.45f, 1 }, { 0.35f, 0.75f, 0.90f, 1 },
            { 0.75f, 0.55f, 0.95f, 1 },
        };
        const char* const ch_nm[7]{ "clinical", "segment", "delta", "quat", "delta", "quat", "included" };

        std::size_t first_channel = 0;
        std::size_t channel_count = 1;
        switch (_angle_view) {
        case angle_view_t::clinical:         first_channel = 0; channel_count = 1; break;
        case angle_view_t::segment:          first_channel = 1; channel_count = 1; break;
        case angle_view_t::included:         first_channel = 6; channel_count = 1; break;
        case angle_view_t::clinical_vs_rest: first_channel = 2; channel_count = 2; break;
        case angle_view_t::segment_vs_rest:  first_channel = 4; channel_count = 2; break;
        }

        // Default Y range [deg]. The flexion-style readings swing about 0 and a walk stays
        // within +/-90; the Included Angle rides around its neutral instead (180 at hip/knee,
        // 90 at the ankle), so that view frames its own band.
        const bool included_view = (_angle_view == angle_view_t::included);
        const float y_lo = included_view ? 0.0f : -90.0f;
        const float y_hi = included_view ? 225.0f : 90.0f;

        // Each subplot is one joint's articulation, which the joint's own name states (a joint's
        // angles describe the bend at it), so the title is the name plus the newest reading. A
        // NaN channel is one the estimator left empty this frame, which the title reads out as
        // n/a; the feet, being marker end sites, stay that way.
        const auto label = [this, first_channel](std::size_t i) {
            std::string s{ pose::get_joint_name(static_cast<pose::joint_id_t>(i)) };

            const bool has_value = _latest_angles[i].has_value()
                && !std::isnan((*_latest_angles[i])[first_channel]);
            s += has_value
                ? std::format("\n{:+.1f} deg", (*_latest_angles[i])[first_channel])
                : std::string{ "\nn/a" };
            return s;
        };

        draw_joint_plot_grid(
            _angle_plot_grid, dpi_scale, y_lo, y_hi, y_cond,
            ch_col, ch_nm, first_channel, channel_count,
            label,
            [this](std::size_t i) { return _angle_plot_buffers.view(i); }
        );

        _angle_plot_grid.reset = false;
    }

    void pose_plot_panel::_render_skeleton_3d(
        const char* title,
        const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>& disp,
        ImVec4 bone_color, ImVec4 point_color,
        float point_size,
        const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>* overlay,
        ImVec4 overlay_color,
        const char* hint)
    {
        const ImVec2 avail = ImGui::GetContentRegionAvail();

        // Fit the box to `disp` alone. If the captured rest is off, the overlay can land far away,
        // and letting it drive the box would shrink the real skeleton to a dot.
        Eigen::Vector3d bb_min = Eigen::Vector3d::Zero(), bb_max = Eigen::Vector3d::Zero();
        int npts = 0;
        for (const auto& v : disp) {
            if (!v.has_value()) { continue; }
            if (npts == 0) { bb_min = bb_max = v.value(); }
            else { bb_min = bb_min.cwiseMin(v.value()); bb_max = bb_max.cwiseMax(v.value()); }
            ++npts;
        }

        const bool do_fit = _autofit_frames > 0;

        const ImPlot3DFlags f3d = ImPlot3DFlags_Equal | ImPlot3DFlags_NoClip | ImPlot3DFlags_NoLegend;
        if (!ImPlot3D::BeginPlot(title, avail, f3d)) { return; }
        ImPlot3D::SetupAxes("right [m]", "depth [m]", "up [m]");
        {
            const ImPlot3DQuat r = front_view_quat();
            ImPlot3D::SetupBoxInitialRotation(r); // double-click reset returns to front
            ImPlot3D::SetupBoxRotation(r, false, ImPlot3DCond_Once); // open facing front
        }
        // Fitted for the auto-fit window, then the range is left free so wheel zoom / pan / rotate work.
        const Eigen::Vector3d center = (npts > 0) ? Eigen::Vector3d{ 0.5 * (bb_min + bb_max) }
                                                  : Eigen::Vector3d{ 0.0, 0.0, -0.4 };
        const double half = (npts > 0) ? std::max(0.5 * (bb_max - bb_min).maxCoeff() * 1.3, 0.15) : 0.6;
        ImPlot3D::SetupAxesLimits(
            center.x() - half, center.x() + half,
            center.y() - half, center.y() + half,
            center.z() - half, center.z() + half,
            do_fit ? ImPlot3DCond_Always : ImPlot3DCond_Once
        );
        if (do_fit && npts > 0) { --_autofit_frames; }

        // The overlay shares one label so its bones don't collide with the primary per-bone ids.
        const auto draw_bones = [](
            const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>& a,
            ImVec4 color, float weight, const char* label)
        {
            ImPlot3DSpec bone;
            bone.LineWeight = weight;
            bone.LineColor = color;
            for (const auto& def : pose::get_joint_defs()) {
                if (pose::is_root_joint(def.joint_id)) { continue; }
                const std::size_t c = static_cast<std::size_t>(def.joint_id);
                const std::size_t p = static_cast<std::size_t>(def.parent);
                if (!a[c].has_value() || !a[p].has_value()) { continue; }
                const double bx[2]{ a[p]->x(), a[c]->x() };
                const double by[2]{ a[p]->y(), a[c]->y() };
                const double bz[2]{ a[p]->z(), a[c]->z() };
                ImPlot3D::PlotLine(label != nullptr ? label : def.name.data(), bx, by, bz, 2, bone);
            }
        };

        draw_bones(disp, bone_color, 3.0f, /*label*/ nullptr);

        std::array<double, pose::kNumJoints> jx{}, jy{}, jz{};
        int jn = 0;
        for (const auto& def : pose::get_joint_defs()) {
            const std::size_t i = static_cast<std::size_t>(def.joint_id);
            if (!disp[i].has_value()) { continue; }
            jx[jn] = disp[i]->x(); jy[jn] = disp[i]->y(); jz[jn] = disp[i]->z(); ++jn;

            // A joint sitting on its parent's point (the hips ride the pelvis marker) skips its
            // label, which would print over the parent's.
            const std::size_t p = static_cast<std::size_t>(def.parent);
            const bool co_sited = !pose::is_root_joint(def.joint_id)
                && disp[p].has_value()
                && (disp[i].value() - disp[p].value()).squaredNorm() < 1e-10;
            if (co_sited) { continue; }
            ImPlot3D::PlotText(pose::get_joint_name(def.joint_id).data(),
                disp[i]->x(), disp[i]->y(), disp[i]->z());
        }
        ImPlot3DSpec pt_spec;
        pt_spec.Marker = ImPlot3DMarker_Circle;
        pt_spec.MarkerSize = point_size;
        pt_spec.MarkerFillColor = point_color;
        ImPlot3D::PlotScatter("joints", jx.data(), jy.data(), jz.data(), jn, pt_spec);

        if (overlay != nullptr) { draw_bones(*overlay, overlay_color, 2.0f, "fk"); }

        if (hint != nullptr) { ImPlot3D::PlotText(hint, 0.0, 0.0, 0.15); }
        ImPlot3D::EndPlot();
    }

} // namespace gui
