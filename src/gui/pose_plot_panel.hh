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
        sagittal_angles,  // per-joint sagittal angles over time (2D subplot grid)
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

        // Which angle the grid draws, one quantity per view (conventions per
        // docs/joint_angle_convention.md). The first three are the wire's three angle fields, so
        // what the protocol broadcasts can be checked here by eye. Every channel is recorded
        // regardless, so switching changes the view and keeps each history.
        enum class angle_view_t {
            clinical,         // Clinical Joint Angle: bend vs the parent bone, flexion +
            segment,          // Segment Angle: bone attitude from vertical, anterior +
            included,         // Included Angle: signed inter-bone angle, pi when collinear
            // NOTE: the two vs-rest views are diagnostic only, drawn from `joint_state_t`'s delta fields.
            clinical_vs_rest, // clinical change since rest, beside the same turn read from `local_anim_rot`
            segment_vs_rest,  // segment change since rest, beside the rotations' chain total
        };

        // One angle-grid sample [deg]. Channels: 0 = sagittal_clinical_angle, 1 = sagittal_segment_angle,
        // 2 = sagittal_clinical_angle_delta, 3 = the delta recovered from `local_anim_rot` (carried into
        // the clinical sign), 4 = sagittal_segment_angle_delta, 5 = the segment delta recovered from the
        // rotations' chain total, 6 = sagittal_included_angle. A channel the estimator left empty
        // holds NaN, which plots as a gap, so one sample carries whichever quantities this frame
        // produced.
        using angle_sample_t = Eigen::Matrix<float, 7, 1>;

        plot_type_t _plot_type{ plot_type_t::raw_skeleton };

        grid_plot_ui_t _pos_plot_grid{};   // positions
        grid_plot_ui_t _angle_plot_grid{}; // sagittal_angles
        angle_view_t _angle_view{ angle_view_t::clinical };

        skeleton_plot_ui_t _raw_skel{};
        skeleton_plot_ui_t _rig_skel{};
        float _raw_skel_fk_bone_color[4]{ 0.95f, 0.85f, 0.20f, 1.0f }; // only the raw view draws a second skeleton

        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> _raw_skel_positions{};
        plot_buffer<Eigen::Vector3f, pose::kNumJoints> _pos_plot_buffers; // display-space positions
        plot_buffer<angle_sample_t, pose::kNumJoints> _angle_plot_buffers;
        // Newest sample of each, for the subplot titles:
        // the buffer's view is strided for plotting and does not hand a single value back.
        std::array<std::optional<angle_sample_t>, pose::kNumJoints> _latest_angles{};

        int _autofit_frames{ kAutofitFrames };
    };

} // namespace gui
