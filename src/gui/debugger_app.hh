#pragma once
#include "app_base.hh"
#include "app_renderer_sdl3.hh"
#include "frame_texture.hh"
#include "log_console.hh"
#include "pose_trace_recorder.hh"
#include "cli_options.hh"

#include "io/recording_writer.hh"
#include "pose/tag_detector.hh"
#include "pose/exo_pose_estimator.hh"
#include "plot_buffer.hh"

#include <imfilebrowser.h>
#include <imgui.h>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace net { class exo_pose_server; }

namespace gui
{
    enum class plot_type_t { axis_frame, euler_line, quat_line };

    enum class source_kind_t { device, recording };

    // Debugger GUI for the pose server: starts/stops the WebSocket listener and drives the pose
    // pipeline (source open/close, rest-pose calibration) while visualizing the annotated frame
    // and per-joint rotations. Owns the server; the listener starts stopped.
    class debugger_app final : public app_base<app_renderer_sdl3>
    {
    public:
        debugger_app(const app::source_options& opt, uint16_t port);
        ~debugger_app();

        int run(); // create window, loop, destroy

    public:
        void render_ui() override;

    private:
        void _open_device(uint32_t index);
        void _open_recording(const std::string& path);

        void _do_open_source();
        void _do_close_source();
        void _do_start_recording();
        void _do_stop_recording();
        void _update_pose_frame();

        void _render_menu_bar();
        void _render_control_panel();
        void _render_recording_status(); // live counters while a recording is being written
        void _dump_pose_trace(); // write the trace ring to a timestamped .json under dumps/
        void _render_plot_panel();
        void _render_open_dialog();
        void _render_record_dialog();
        void _render_log_panel();  // bottom dock: resize grip + log console child
        float _log_split_height(); // clamps `_ui.log_h`; returns the main content height above the panel
        void _sync_axis_frame(); // read-back rotation/range sync + reset for the current implot3d plot

    private:
        // gui control states
        struct ui_state_t
        {
            // view / visualization
            bool  camera_fullscreen{ false };
            bool  relative_rot{ true }; // true = local (vs parent), false = global (camera frame)
            int   euler_order{ 0 };     // index into kEulerOrders for the euler readout
            plot_type_t plot_type{ plot_type_t::axis_frame };
            bool  autosize_plots{ true }; // pack subplots to fill the panel
            float plot_size_px{ 150.0f }; // manual subplot cell size [px], DPI-scaled at use
            bool  lock_plots{ false }; // true = force default ranges (live); false = mouse-adjustable
            bool  sync_plots{ true };  // share one range across all subplots
            float side_w{ 460.0f };    // right control panel width [px], splitter-adjustable
            bool  show_log{ false };   // spdlog output console: bottom dock panel
            float log_h{ 200.0f };     // bottom log panel height [px], splitter-adjustable

            // open-source dialog
            bool  show_open{ false };
            source_kind_t open_kind{ source_kind_t::device };
            int   device{ 0 };
            bool  manual_exposure{ false };
            int   exposure{ 8000 };
            bool  manual_gain{ false };
            int   gain{ 0 };
            std::string recording;

            // record dialog
            bool  show_record{ false };
            int   record_codec{ 0 }; // index into io::kImageCodecs
            int   jpeg_quality{ 90 };
            std::string record_path;

            // diagnostic pose trace (rolling ring dumped to JSON on demand)
            bool  trace_enabled{ true }; // capture each pose frame into the ring
            int   trace_capacity{ 600 }; // ring length [frames]
        };

        app::source_options _opt;
        std::unique_ptr<net::exo_pose_server> _server;

        std::optional<frame_texture> _texture;
        ImGui::FileBrowser _file_dialog;
        ImGui::FileBrowser _save_dialog{ ImGuiFileBrowserFlags_EnterNewFilename | ImGuiFileBrowserFlags_CreateNewDir };
        log_console _log_console;

        cv::Mat _frame;
        std::vector<pose::tag_detection_t> _detections;
        uint64_t _last_seq{ 0 };

        pose_trace_recorder _trace; // rolling per-frame diagnostic trace (dumped to JSON on demand)

        ui_state_t _ui;

        // scrolling plot buffer per joint (euler xyz / quat xyzw)
        plot_buffer<Eigen::Vector3f, pose::kNumJoints> _euler_bufs;
        plot_buffer<Eigen::Quaternionf, pose::kNumJoints> _quat_bufs;

        // plot limit controls (one-shot reset, shared by line + axis-frame plots)
        bool _reset_plots{ false };
        // line plots: shared y-range link vars for "sync" (x always auto-scrolls)
        double _sync_y[2]{ 0.0, 0.0 };
        // axis-frame plots: shared rotation (quat xyzw) + per-axis range, captured from the hovered plot
        double _sync_rot[4]{ 0.0, 0.0, 0.0, 1.0 };
        double _sync_range[3][2]{ { -1.2, 1.2 }, { -1.2, 1.2 }, { -1.2, 1.2 } };
        bool _sync_init{ false };
    };

} // namespace gui
