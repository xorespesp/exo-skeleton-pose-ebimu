#pragma once
#include "app_base.hh"
#include "app_renderer_sdl3.hh"
#include "frame_texture.hh"
#include "log_console.hh"
#include "pose_trace_recorder.hh"
#include "app_config.hh"

#include "io/recording_writer.hh"
#include "pose/marker_tracker.hh"
#include "pose/frontal_pose_estimator.hh"
#include "pose/sagittal_pose_estimator.hh"
#include "pose/hinge_angle.hh"
#include "pose/view_plane.hh"
#include "plot_buffer.hh"

#include <imfilebrowser.h>
#include <imgui.h>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace net { class exo_pose_server; }

namespace gui
{
    enum class plot_type_t {
        raw_skeleton,  // measured per-joint 3D positions + bones, with the forward-kinematics overlay
        rig_skeleton,  // fixed-length T-pose leg rig driven purely by the per-joint `local_anim_rot`
        positions,     // per-joint position channels over time, in plot space (2D line subplot grid)
        sagittal_angles,  // per-joint flexion over time: the measured angle against the one `local_anim_rot` carries
    };

    // One grid mode's whole state: the modes that lay one subplot out per joint (`positions`,
    // `sagittal_angles`) each keep one of these, so adding a knob reaches both by one line.
    struct grid_plot_ui_t
    {
        bool autosize{ true };    // pack subplots to fill the panel
        float size_px{ 150.0f };  // manual subplot cell size [px], DPI-scaled at use
        bool lock{ false };       // force the default range (live), else mouse-adjustable
        bool sync{ true };        // share one Y range across all subplots

        bool reset{ false };          // one-shot: force the default range on the next frame
        double sync_y[2]{ 0.0, 0.0 }; // the range `sync` shares across the subplots
    };

    // Style of one 3D skeleton plot.
    struct skeleton_plot_ui_t
    {
        float point_size{ 7.5f };                          // joint sphere size [px]
        float point_color[4]{ 0.95f, 0.45f, 0.20f, 1.0f }; // joint sphere color (orange)
        float bone_color[4]{ 0.55f, 0.75f, 0.95f, 1.0f };  // bone line color (blue)
    };

    enum class source_kind_t { k4a_device, vz_device, recording };

    // Debugger GUI for the pose server: starts/stops the WebSocket listener and drives the pose
    // pipeline (source open/close, rest-pose calibration) while visualizing the annotated frame
    // and the per-joint 3D positions / reconstructed skeleton. Owns the server; the listener starts stopped.
    class debugger_app final : public app_base<app_renderer_sdl3>
    {
    public:
        explicit debugger_app(const app::app_config_t& config);
        ~debugger_app();

        int run(); // create window, loop, destroy

    public:
        void render_ui() override;

    private:
        // Hands the address to the pipeline and restarts the plots for the new source.
        void _open_source(const app::source_address& address, pose::view_plane_t view_plane);

        void _do_open_source();
        void _do_close_source();

        // Reads `path` over the settings this window edits, discarding what the control panel has
        // been tuned to since the last save. It fills the open dialog and reopens an open source,
        // so what streams is what the settings say. A file that fails to load changes nothing.
        void _do_load_config(const std::filesystem::path& path);
        void _do_save_config(const std::filesystem::path& path);

        // Fills the open dialog from the settings the server holds.
        void _seed_open_dialog();
        void _do_start_recording();
        void _do_stop_recording();
        void _update_pose_frame();
        void _dump_pose_trace(); // write the trace ring to a timestamped .json under dumps/

        void _render_menu_bar();
        void _render_control_panel();

        // Estimator tuning, one function per viewing plane.
        void _render_frontal_estimator_control(pose::frontal_pose_estimator::options_t& opt);
        void _render_sagittal_estimator_control(pose::sagittal_pose_estimator::options_t& opt);

        // Blob filters, joint assignment, and what the last frame found, plus the sampling that
        // measures this installation's colour.
        void _render_color_marker_control(pose::color_marker_tracker& tracker);

        // Sampling and fitting the colour itself, which is what the rest of that panel is tuned
        // against. Nothing here saves: the fit lands on the running tracker and reaches the file
        // through the profile, like every other control on this panel.
        void _render_color_model_section(pose::color_marker_tracker& tracker);

        // Feeds the pixels under the cursor to the colour sampler, over the camera view drawn
        // between `img_min` and `img_max`.
        void _handle_color_sample_click(const ImVec2& img_min, const ImVec2& img_max);

        // Fits the collected samples and installs the result on the running tracker, so the next
        // frame is classified by what was just measured.
        void _do_fit_color_model(pose::color_marker_tracker& tracker);
        void _render_recording_status(); // live counters while a recording is being written
        void _render_plot_panel();
        void _render_raw_skeleton_plot();  // raw_skeleton: measured 3D positions + bones (+ FK overlay)
        void _render_rig_skeleton_plot();  // rig_skeleton: fixed-length T-pose rig driven by `local_anim_rot`
        void _render_positions_plot();     // positions: per-joint positions over time (2D subplot grid)
        void _render_sagittal_angles_plot();  // sagittal_angles: per-joint flexion over time (2D subplot grid)

        // Shared 3D skeleton renderer: one front-facing, equal-scaled, data-fitted plot drawing the
        // per-joint display-space positions as bones + spheres + labels, with an optional second
        // skeleton overlaid (in `overlay_color`) and an optional centered hint. Used by both skeleton modes.
        void _render_skeleton_3d(
            const char* title,
            const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>& disp,
            ImVec4 bone_color, ImVec4 point_color,
            float point_size,
            const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>* overlay,
            ImVec4 overlay_color,
            const char* hint
        );

        void _render_open_dialog();
        void _render_record_dialog();
        void _render_log_panel();  // bottom dock: resize grip + log console child
        float _log_split_height(); // clamps `_ui.log_panel_height`; returns the main content height above the panel

    private:
        // gui control states
        struct ui_state_t
        {
            // window layout (splitter-adjustable panels)
            float side_panel_width{ 460.0f };  // right control panel width [px]
            bool show_log{ false };            // spdlog output console: bottom dock panel
            float log_panel_height{ 200.0f };  // bottom log panel height [px]

            // view / visualization
            bool camera_fullscreen{ false };
            plot_type_t plot_type{ plot_type_t::raw_skeleton };

            // 2D subplot-grid controls, one set per grid mode so each keeps its own framing
            grid_plot_ui_t pos_grid{};   // positions
            grid_plot_ui_t angle_grid{}; // sagittal_angles
            // Which flexion the sagittal-angles grid draws: each joint's turn from its parent bone, or
            // its own bone's turn in the rig frame. Both are recorded, so this only picks the view.
            bool angle_plot_relative{ true };

            // 3D skeleton plot styles, one per mode so each keeps its own
            skeleton_plot_ui_t raw_skel{}; // raw_skeleton
            skeleton_plot_ui_t rig_skel{}; // rig_skeleton
            // Only the raw plot draws a second skeleton over the first, so this is not part of the
            // style both modes share.
            float raw_skel_fk_bone_color[4]{ 0.95f, 0.85f, 0.20f, 1.0f };

            // open-source dialog
            bool open_dlg_show{ false };
            source_kind_t open_dlg_kind{ source_kind_t::k4a_device };
            pose::view_plane_t open_dlg_view_plane{ pose::view_plane_t::frontal };
            int open_dlg_device{ 0 }; // index within whichever camera backend is selected
            bool open_dlg_manual_exposure{ false };
            int open_dlg_exposure{ 8000 };
            bool open_dlg_manual_gain{ false };
            int open_dlg_gain{ 0 };
            std::string open_dlg_recording;
            std::string open_dlg_intrinsics; // calibration file for a camera that reports none
            app::marker_kind_t open_dlg_marker_kind{ app::marker_kind_t::apriltag };

            // colour sampling (the panel that measures this installation's colour)
            bool color_sampling{ false }; // clicking the camera view feeds the sampler
            int color_sample_radius{ 6 }; // pixels collected around each click [px]
            double color_max_distance{ 3.0 }; // how far into the fitted ellipse still counts
            int color_backdrop{ 0 };      // 0 camera, 1 mask, 2 membership score

            // record dialog
            bool record_dlg_show{ false };
            int record_dlg_codec{ 0 }; // index into `io::kImageCodecs`
            int record_dlg_jpeg_quality{ 90 };
            std::string record_dlg_path;

            // diagnostic pose trace (rolling ring dumped to JSON on demand)
            bool trace_enabled{ true }; // capture each pose frame into the ring
            int trace_capacity{ 600 }; // ring length [frames]
        };

        // Owns the installation settings this window edits; `_server->config()` is the one copy,
        // so a client command and this window open a source with the same thing.
        std::unique_ptr<net::exo_pose_server> _server;

        std::optional<frame_texture> _frame_texture;
        ImGui::FileBrowser _file_dialog;
        ImGui::FileBrowser _save_dialog{ ImGuiFileBrowserFlags_EnterNewFilename | ImGuiFileBrowserFlags_CreateNewDir };
        ImGui::FileBrowser _config_dialog{ ImGuiFileBrowserFlags_EnterNewFilename | ImGuiFileBrowserFlags_CreateNewDir };
        ImGui::FileBrowser _config_open_dialog; // picks an existing profile, so no new-filename flag
        ImGui::FileBrowser _intrinsics_dialog;
        log_console _log_console;

        // last annotated frame pulled from the pipeline, the same capture undrawn, and its
        // sequence number. The undrawn one is what a colour sample is taken from.
        cv::Mat _last_frame;
        cv::Mat _last_source_frame;
        uint64_t _last_seq{ 0 };

        // Samples this installation's colour, accumulated across clicks and frames. What it holds
        // is a measurement in progress, discarded whenever the operator says so and no part of
        // detecting anything, so it belongs to the panel that collects it.
        pose::color_sampler _color_sampler;

        pose_trace_recorder _trace; // rolling per-frame diagnostic trace (dumped to JSON on demand)

        ui_state_t _ui;

        // latest per-joint rig-space 3D position for the `raw_skeleton` plot (smoothed or raw)
        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> _raw_skel_positions{};
        // per-joint position history (display space) for the positions plot
        plot_buffer<Eigen::Vector3f, pose::kNumJoints> _pos_plot_bufs;
        // per-joint flexion history [deg] for the sagittal-angles plot, two pairs of (estimator's
        // measured angle, the angle read back out of `local_anim_rot`): 
        // channels 0-1 hold the turn from the parent bone, 
        // channels 2-3 the bone's own turn in the rig frame
        plot_buffer<Eigen::Vector4f, pose::kNumJoints> _angle_plot_bufs;
        int _skel_plot_autofit_frames{ 30 }; // frames left to auto-fit the 3D box (after a source/view change), then free zoom
    };

} // namespace gui
