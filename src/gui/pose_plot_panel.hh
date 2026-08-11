#pragma once
#include "plot_buffer.hh"

#include "pose/joints_def.hh"
#include "pose/pose_estimator_base.hh"

#include <imgui.h>
#include <Eigen/Geometry>

#include <array>
#include <optional>

namespace gui
{
    enum class plot_type_t {
        raw_skeleton,     // measured 3D positions + bones, with the forward-kinematics overlay
        rig_skeleton,     // fixed-length T-pose leg rig driven by the per-joint `local_anim_rot`
        positions,        // per-joint position channels over time (2D subplot grid)
        sagittal_angles,  // per-joint flexion over time (2D subplot grid)
    };

    // One grid mode's whole state, so adding a knob reaches both grid modes by one line.
    struct grid_plot_ui_t
    {
        bool autosize{ true };    // pack subplots to fill the panel
        float size_px{ 150.0f };  // manual subplot cell size [px], DPI-scaled at use
        bool lock{ false };       // force the default range (live), else mouse-adjustable
        bool sync{ true };        // share one Y range across all subplots

        bool reset{ false };          // one-shot: force the default range on the next frame
        double sync_y[2]{ 0.0, 0.0 }; // the range `sync` shares across the subplots
    };

    struct skeleton_plot_ui_t
    {
        float point_size{ 7.5f };                          // joint sphere size [px]
        float point_color[4]{ 0.95f, 0.45f, 0.20f, 1.0f }; // joint sphere (orange)
        float bone_color[4]{ 0.55f, 0.75f, 0.95f, 1.0f };  // bone line (blue)
    };

    // The debugger's plot pane: one of four views of the solved joint state, over a toolbar that
    // picks the view. Everything here describes how that state is looked at and never how it is
    // produced, so nothing the panel holds reaches a config file.
    class pose_plot_panel
    {
    public:
        // Records one solved frame. `t_now` is the estimator's frame time, so the histories scroll
        // with the source rather than with the window's frame rate.
        void push(const pose::pose_estimator_base& est, double t_now);

        // Drops every history and reframes the 3D box, for a source that does not continue the
        // samples already plotted.
        void reset();

        // `est` is null until a source is open and solving, which leaves the rig view on its
        // neutral T-pose and the rest blank.
        void render(const pose::pose_estimator_base* est, float dpi_scale);

    private:
        // The toolbar carries what is reached while watching a view (framing, range locks); what is
        // settled once sits behind `Style...`, so the row stays one line and the plot keeps the height.
        void _render_toolbar();
        void _render_style_popup();

        void _render_raw_skeleton_plot(const pose::pose_estimator_base* est);
        void _render_rig_skeleton_plot(const pose::pose_estimator_base* est);
        void _render_positions_plot(float dpi_scale);
        void _render_sagittal_angles_plot(float dpi_scale);

        // Front-facing, equal-scaled, data-fitted 3D plot: `disp` as bones + spheres + labels, with
        // an optional second skeleton in `overlay_color` and an optional centered hint.
        void _render_skeleton_3d(
            const char* title,
            const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>& disp,
            ImVec4 bone_color, ImVec4 point_color,
            float point_size,
            const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>* overlay,
            ImVec4 overlay_color,
            const char* hint
        );

    private:
        static constexpr int kAutofitFrames = 30; // frames the 3D box is fitted for, then the range is the mouse's

        plot_type_t _plot_type{ plot_type_t::raw_skeleton };

        grid_plot_ui_t _pos_grid{};   // positions
        grid_plot_ui_t _angle_grid{}; // sagittal_angles
        bool _angle_relative{ true }; // draw each joint's turn from its parent bone, else its turn in the rig frame

        skeleton_plot_ui_t _raw_skel{};
        skeleton_plot_ui_t _rig_skel{};
        float _raw_skel_fk_bone_color[4]{ 0.95f, 0.85f, 0.20f, 1.0f }; // only the raw view draws a second skeleton

        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> _raw_skel_positions{};
        plot_buffer<Eigen::Vector3f, pose::kNumJoints> _pos_bufs; // display-space positions
        // Flexion [deg], the pair (estimator's measured angle, the angle read back out of
        // `local_anim_rot`) twice over: channels 0-1 from the parent bone, 2-3 in the rig frame.
        plot_buffer<Eigen::Vector4f, pose::kNumJoints> _angle_bufs;

        int _autofit_frames{ kAutofitFrames };
    };

} // namespace gui
