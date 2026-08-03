#pragma once
#include "app_base.hh"
#include "app_renderer_sdl3.hh"
#include "frame_texture.hh"
#include "log_console.hh"
#include "pose_trace_recorder.hh"
#include "app_config.hh"

#include "io/recording_writer.hh"
#include "pose/tag_detector.hh"
#include "pose/frontal_pose_estimator.hh"
#include "pose/sagittal_pose_estimator.hh"
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

        void _do_save_config(const std::filesystem::path& path);
        void _do_start_recording();
        void _do_stop_recording();
        void _update_pose_frame();
        void _dump_pose_trace(); // write the trace ring to a timestamped .json under dumps/

        void _render_menu_bar();
        void _render_control_panel();

        // Estimator tuning, one function per viewing plane.
        void _render_frontal_estimator_control(pose::frontal_pose_estimator::options_t& opt);
        void _render_sagittal_estimator_control(pose::sagittal_pose_estimator::options_t& opt);
        void _render_recording_status(); // live counters while a recording is being written
        void _render_plot_panel();
        void _render_raw_skeleton_plot();  // raw_skeleton: measured 3D positions + bones (+ FK overlay)
        void _render_rig_skeleton_plot();  // rig_skeleton: fixed-length T-pose rig driven by `local_anim_rot`
        void _render_positions_plot();     // positions: per-joint positions over time (2D subplot grid)

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

            // positions plot (2D subplot grid) controls
            bool pos_plot_autosize{ true };   // pack subplots to fill the panel
            float pos_plot_size_px{ 150.0f }; // manual subplot cell size [px], DPI-scaled at use
            bool pos_plot_lock{ false };      // force the default range (live), else mouse-adjustable
            bool pos_plot_sync{ true };       // share one Y range across all subplots

            // raw_skeleton plot style
            float raw_skel_point_size{ 7.5f };                          // joint sphere size [px]
            float raw_skel_point_color[4]{ 0.95f, 0.45f, 0.20f, 1.0f }; // joint sphere color (orange)
            float raw_skel_bone_color[4]{ 0.55f, 0.75f, 0.95f, 1.0f };  // bone line color (blue)
            float raw_skel_fk_bone_color[4]{ 0.95f, 0.85f, 0.20f, 1.0f }; // FK overlay bone color

            // rig_skeleton plot style
            float rig_skel_point_size{ 7.5f };                          // joint sphere size [px]
            float rig_skel_point_color[4]{ 0.95f, 0.45f, 0.20f, 1.0f }; // joint sphere color (orange)
            float rig_skel_bone_color[4]{ 0.55f, 0.75f, 0.95f, 1.0f };  // bone line color (blue)

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

            // record dialog
            bool record_dlg_show{ false };
            int record_dlg_codec{ 0 }; // index into `io::kImageCodecs`
            int record_dlg_jpeg_quality{ 90 };
            std::string record_dlg_path;

            // diagnostic pose trace (rolling ring dumped to JSON on demand)
            bool trace_enabled{ true }; // capture each pose frame into the ring
            int trace_capacity{ 600 }; // ring length [frames]
        };

        app::app_config_t _config;
        std::unique_ptr<net::exo_pose_server> _server;

        std::optional<frame_texture> _frame_texture;
        ImGui::FileBrowser _file_dialog;
        ImGui::FileBrowser _save_dialog{ ImGuiFileBrowserFlags_EnterNewFilename | ImGuiFileBrowserFlags_CreateNewDir };
        ImGui::FileBrowser _config_dialog{ ImGuiFileBrowserFlags_EnterNewFilename | ImGuiFileBrowserFlags_CreateNewDir };
        ImGui::FileBrowser _intrinsics_dialog;
        log_console _log_console;

        // last frame pulled from the pipeline: annotated image, its tag detections, and its sequence number
        cv::Mat _last_frame;
        std::vector<pose::tag_detection_t> _last_tag_detections;
        uint64_t _last_seq{ 0 };

        pose_trace_recorder _trace; // rolling per-frame diagnostic trace (dumped to JSON on demand)

        ui_state_t _ui;

        // latest per-joint rig-space 3D position for the `raw_skeleton` plot (smoothed or raw)
        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> _raw_skel_positions{};
        // per-joint position history (display space) for the positions plot
        plot_buffer<Eigen::Vector3f, pose::kNumJoints> _pos_plot_bufs;
        int _skel_plot_autofit_frames{ 30 }; // frames left to auto-fit the 3D box (after a source/view change), then free zoom
        // positions plot (2D subplot grid) range controls
        bool _pos_plot_reset{ false };   // one-shot: force the default range on the next frame
        double _pos_plot_sync_y[2]{ 0.0, 0.0 }; // shared Y range link for "Sync Plots"
    };

} // namespace gui
