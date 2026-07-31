#include "debugger_app.hh"

#include "net/exo_pose_server.hh"
#include "net/exo_pose_pipeline.hh"

#include <spdlog/spdlog.h>

#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <format>
#include <vector>

namespace gui
{
    namespace
    {
        // Codec picker entries, one per `io::kImageCodecs` row and in the same order.
        constexpr std::array<const char*, io::kImageCodecs.size()> kCodecLabels{
            "JPEG (compressed)",
            "Raw BGR8 (lossless)",
        };

        // A recording is named after the moment it started, so successive takes never
        // collide and are orderable by name.
        std::string default_recording_name()
        {
            const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
            try {
                const std::chrono::zoned_time local{ std::chrono::current_zone(), now };
                return std::format("recording_{:%Y%m%d_%H%M%S}.mcap", local);
            }
            catch (const std::exception&) {
                return std::format("recording_{:%Y%m%d_%H%M%S}Z.mcap", now); // no time zone database
            }
        }

        // A pose-trace dump is named after the moment it was written (same convention as recordings).
        std::string default_trace_name()
        {
            const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
            try {
                const std::chrono::zoned_time local{ std::chrono::current_zone(), now };
                return std::format("pose_trace_{:%Y%m%d_%H%M%S}.json", local);
            }
            catch (const std::exception&) {
                return std::format("pose_trace_{:%Y%m%d_%H%M%S}Z.json", now); // no time zone database
            }
        }

        // Map a rig-space position into the ImPlot3D plot frame, shared by every 3D view (raw + rig).
        //   Rig frame:      X-right, Y-down,  Z-depth   (see skeleton.hh).
        //   ImPlot3D frame: X-right, Y-depth, Z-up      (ImPlot3D treats Z as vertical).
        // So (x, y, z) -> (x, z, -y): a mirror-free rotation, rig-down (-Y) becoming plot-up.
        Eigen::Vector3f rig_to_display(const Eigen::Vector3d& p)
        {
            return Eigen::Vector3f(static_cast<float>(p.x()), static_cast<float>(p.z()), static_cast<float>(-p.y()));
        }

        constexpr float kPositionsWindowSec = 6.0f; // positions-plot x-axis scroll span [s]
        constexpr int kNumAutofitFrames = 30; // auto-fit the 3D box for this many frames after a source/view change

        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

        // Small double-DragScalar helper (estimator options are double; avoids float temporaries).
        bool option_drag(const char* label, double& v, double lo, double hi, double step, const char* fmt)
        {
            return ImGui::DragScalar(label, ImGuiDataType_Double, &v, static_cast<float>(step),
                &lo, &hi, fmt, ImGuiSliderFlags_AlwaysClamp);
        }

        // Initial box rotation: a readable 3/4 front view. ImPlot3D projects with screen-up = (Rotation*p).y, 
        // so bringing the plot's up axis (Z) onto screen-up needs a -90 deg turn about X (== `FromElAz(0,0)`); 
        // a small pitch+yaw tilt it off a flat face-on.
        ImPlot3DQuat front_view_quat()
        {
            constexpr double pi = 3.14159265358979323846;
            const double d = pi / 180.0;
            return ImPlot3DQuat(15.0 * d, ImPlot3DPoint(1, 0, 0))   // pitch: look slightly down
                 * ImPlot3DQuat(-pi / 2, ImPlot3DPoint(1, 0, 0))    // base: up -> screen up, look along depth
                 * ImPlot3DQuat(20.0 * d, ImPlot3DPoint(0, 0, 1));  // yaw about up: slight 3/4
        }

        // Fixed-length T-pose lower-limb rest layout in rig space (X-right, Y-down, Z-depth), indexed to `kJointsInfo`.
        // The rig-skeleton plot drives it by each joint's `local_anim_rot` to eyeball the joint rotations without a rig client.
        // The robot faces the camera, so its right leg sits on camera-left (-X), its left leg on camera-right (+X).
        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> canonical_rest_layout()
        {
            constexpr double hip = 0.10, thigh = 0.45, shin = 0.42, foot = 0.16; // bone lengths [m]
            const auto at = [](pose::joint_id_t j) { return static_cast<std::size_t>(j); };
            std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> r{};
            r[at(pose::joint_id_t::pelvis)]  = Eigen::Vector3d{ 0.0, 0.0, 0.0 };
            r[at(pose::joint_id_t::r_knee)]  = Eigen::Vector3d{ -hip, thigh, 0.0 };
            r[at(pose::joint_id_t::l_knee)]  = Eigen::Vector3d{ +hip, thigh, 0.0 };
            r[at(pose::joint_id_t::r_ankle)] = Eigen::Vector3d{ -hip, thigh + shin, 0.0 };
            r[at(pose::joint_id_t::l_ankle)] = Eigen::Vector3d{ +hip, thigh + shin, 0.0 };
            // Foot points down and forward from the ankle (-Z), matching the ankle->foot marker bone direction.
            r[at(pose::joint_id_t::r_foot)]  = Eigen::Vector3d{ -hip, thigh + shin + 0.75 * foot, -0.66 * foot };
            r[at(pose::joint_id_t::l_foot)]  = Eigen::Vector3d{ +hip, thigh + shin + 0.75 * foot, -0.66 * foot };
            return r;
        }

        // Forward-kinematics the rig from per-joint local rotations, data driven over `kJointsInfo`(parent precedes child). 
        // Each joint, from its parent:
        //   world_rot = parent_world_rot * anim
        //   world_pos = parent_world_pos + world_rot * (rest[joint] - rest[parent])
        // Missing anim -> joint keeps its rest orientation; 
        // Missing rest (joint or an ancestor) -> no position. 
        // Output shares the frame of `rest` / `root_anchor`.
        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> rig_fk(
            const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>& rest,
            const std::array<std::optional<Eigen::Quaterniond>, pose::kNumJoints>& anim,
            const Eigen::Vector3d& root_anchor)
        {
            std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> pos{};
            std::array<Eigen::Quaterniond, pose::kNumJoints> world_rot{};
            for (const auto& info : pose::kJointsInfo)
            {
                const std::size_t j = static_cast<std::size_t>(info.id);
                const Eigen::Quaterniond a = anim[j].value_or(Eigen::Quaterniond::Identity());
                if (pose::is_root_joint(info.id))
                {
                    world_rot[j] = a.normalized();
                    pos[j] = root_anchor;
                    continue;
                }
                const std::size_t p = static_cast<std::size_t>(info.parent);
                if (!pos[p].has_value() || !rest[j].has_value() || !rest[p].has_value()) { continue; }
                world_rot[j] = (world_rot[p] * a).normalized();
                pos[j] = pos[p].value() + world_rot[j] * (rest[j].value() - rest[p].value());
            }
            return pos;
        }

        // Splitter grip thickness [px]. It doubles as the inter-panel gap: surrounding
        // ItemSpacing is zeroed so the visible border-to-border gap equals this on both
        // axes, and the whole gap is the drag hit-target (same width for v/h splitters).
        constexpr float kSplitHit = 6.0f;
        constexpr float kLogMinH = 60.0f;  // min height for both the content and log panes [px]
        constexpr float kPlotMinW = 200.0f; // min width for the plots pane [px]
        constexpr float kSideMinW = 200.0f; // min width for the control pane [px]

        // One subplot: a buffer's channels drawn zero-copy from its strided view over the newest
        // `window` seconds. x auto-scrolls; y obeys `y_cond` (Always locks, Once mouse-free) and
        // `sync` (links y to the shared `sy` so all subplots share one range).
        template <typename _Scalar>
        void draw_plot_lines(
            const char* title,
            const plot_buffer_view<_Scalar>& v,
            float window,
            float y_lo,
            float y_hi,
            ImPlotCond y_cond,
            bool sync,
            double* sy,
            const ImVec4* colors,
            const char* const* names,
            const ImVec2& size)
        {
            // legend shown (short names); the caller wraps each subplot in PushID/PopID for unique ids.
            if (!ImPlot::BeginPlot(title, size, ImPlotFlags_None)) { return; }

            ImPlot::SetupAxes(nullptr, nullptr, 0, 0);
            ImPlot::SetupLegend(ImPlotLocation_NorthWest);
            if (sync) { ImPlot::SetupAxisLinks(ImAxis_Y1, &sy[0], &sy[1]); } // sync y only
            ImPlot::SetupAxisLimits(ImAxis_X1, v.t_hi - window, v.t_hi, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, y_lo, y_hi, y_cond);

            for (std::size_t k = 0; k < v.ys.size(); ++k)
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

    } // namespace

    debugger_app::debugger_app(const app::source_options& opt, uint16_t port)
        : _opt{ opt }
        , _server{ std::make_unique<net::exo_pose_server>(port, opt, /*annotate_frames*/ true) }
    {
        if (opt.exposure_us.has_value()) { _ui.open_dlg_manual_exposure = true; _ui.open_dlg_exposure = opt.exposure_us.value(); }
        if (opt.gain.has_value()) { _ui.open_dlg_manual_gain = true; _ui.open_dlg_gain = opt.gain.value(); }
        _ui.open_dlg_view_plane = opt.view_plane; // command-line viewing plane seeds the dialog

        // The dialog holds a value per kind so toggling does not discard what was typed;
        // only fill the one the command line named.
        if (opt.source_addr.has_value())
        {
            if (opt.source_addr->is_device()) {
                _ui.open_dlg_device = static_cast<int>(opt.source_addr->device_index());
                _ui.open_dlg_kind = source_kind_t::device;
            } else {
                _ui.open_dlg_recording = opt.source_addr->recording_path().string();
                _ui.open_dlg_kind = source_kind_t::recording;
            }
        }
        _ui.show_log = true; // surface the log console by default

        _file_dialog.SetTitle("Open recording file");
        _file_dialog.SetTypeFilters({ ".mcap" });

        _save_dialog.SetTitle("Save recording as");
        _save_dialog.SetTypeFilters({ ".mcap" });

        // Mirror spdlog output into the in-GUI console. Registered on the main thread before any
        // capture worker exists, so appending to the sink list is race-free. Captures every severity;
        // the console's own toggles filter the view, and re-enabling a level can reveal what it missed.
        _log_console.sink()->set_level(spdlog::level::trace);
        spdlog::default_logger()->sinks().push_back(_log_console.sink());
    }

    debugger_app::~debugger_app() = default;

    int debugger_app::run()
    {
        if (!this->create("exo-skeleton-pose debugger", 1440, 900))
        {
            spdlog::error("failed to create debugger window");
            return -1;
        }

        spdlog::info("debugger ready: open a source from File > Open..., start the listener from Server > Start Server");

        this->app_base::run();

        spdlog::info("debugger shutting down");
        this->destroy();
        return 0;
    }

    void debugger_app::render_ui()
    {
        if (!_frame_texture.has_value()) { _frame_texture.emplace(this->renderer().sdl_renderer()); }

        // Advance the server one tick: services the listener when up, and always pumps the
        // pipeline so device/algorithm testing works whether or not it's running.
        _server->poll();
        this->_update_pose_frame();

        if (ImGui::IsKeyPressed(ImGuiKey_F11, false)) { _ui.camera_fullscreen = !_ui.camera_fullscreen; }
        this->_render_menu_bar();

        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        constexpr ImGuiWindowFlags host_flags =
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

        // Drop the host's rounded corners and outer border.
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::Begin("##host", nullptr, host_flags);
        ImGui::PopStyleVar(2);

        // plots, control, and log are all direct host siblings (no wrapper child), so
        // every inter-panel gap is the same kSplitHit-wide grip drawn over the same host
        // background: the log/plots gap matches the control/plots gap exactly.
        // `row_h` is the height of the main content row above the (optional) log panel.
        const bool show_log = _ui.show_log && !_ui.camera_fullscreen;
        const float row_h = show_log ? this->_log_split_height() : ImGui::GetContentRegionAvail().y;

        if (!_server->pipeline().is_source_open())
        {
            // No source: centered call-to-action, bounded to the content row.
            ImGui::BeginChild("content", ImVec2(0, row_h), ImGuiChildFlags_None);
            const char* msg = "Open a source to start.   (File > Open...)";
            const ImVec2 avail = ImGui::GetContentRegionAvail();
            const ImVec2 sz = ImGui::CalcTextSize(msg);
            const ImVec2 cur = ImGui::GetCursorPos();
            ImGui::SetCursorPos(ImVec2{ cur.x + (avail.x - sz.x) * 0.5f, cur.y + (avail.y - sz.y) * 0.5f });
            ImGui::TextDisabled("%s", msg);
            ImGui::EndChild();
        }
        else if (_ui.camera_fullscreen)
        {
            // Fullscreen: sensor frame scaled to fit, centered (log panel is hidden here).
            if (!_frame_texture.value().valid()) { ImGui::TextUnformatted("Waiting for frames...  (F11 to exit)"); }
            else
            {
                const ImVec2 avail = ImGui::GetContentRegionAvail();
                const float tw = static_cast<float>(_frame_texture.value().width());
                const float th = static_cast<float>(_frame_texture.value().height());
                const float scale = std::min(avail.x / tw, avail.y / th);
                const ImVec2 sz{ tw * scale, th * scale };
                const ImVec2 cur = ImGui::GetCursorPos();
                ImGui::SetCursorPos(ImVec2{ cur.x + (avail.x - sz.x) * 0.5f, cur.y + (avail.y - sz.y) * 0.5f });
                ImGui::Image(_frame_texture.value().id(), sz);
            }
        }
        else
        {
            // Normal: plot panel (left) + control panel (right), split by a grip whose
            // width equals the log splitter's so every gap looks identical.
            const float avail_x = ImGui::GetContentRegionAvail().x;
            const float max_side = std::max(kSideMinW, avail_x - kSplitHit - kPlotMinW);
            _ui.side_panel_width = std::clamp(_ui.side_panel_width, kSideMinW, max_side);
            const float plots_w = avail_x - _ui.side_panel_width - kSplitHit;

            ImGui::BeginChild("plots", ImVec2(plots_w, row_h), ImGuiChildFlags_Borders);
            this->_render_plot_panel();
            ImGui::EndChild();

            // Vertical resize grip (no visible line): flush to both panes (zero spacing),
            // so the whole inter-panel gap is grabbable. Drag left to grow the control pane.
            ImGui::SameLine(0.0f, 0.0f);
            ImGui::InvisibleButton("##side_split", ImVec2(kSplitHit, row_h));
            if (ImGui::IsItemActive()) { _ui.side_panel_width -= ImGui::GetIO().MouseDelta.x; }
            if (ImGui::IsItemHovered() || ImGui::IsItemActive()) { ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW); }
            ImGui::SameLine(0.0f, 0.0f);

            ImGui::BeginChild("side", ImVec2(0, row_h), ImGuiChildFlags_Borders);
            this->_render_control_panel();
            ImGui::EndChild();
        }

        if (show_log) { this->_render_log_panel(); }

        ImGui::End();

        this->_render_open_dialog();
        this->_render_record_dialog();

        _file_dialog.Display();
        if (_file_dialog.HasSelected())
        {
            _ui.open_dlg_recording = _file_dialog.GetSelected().string();
            _ui.open_dlg_kind = source_kind_t::recording;
            _file_dialog.ClearSelected();
        }

        _save_dialog.Display();
        if (_save_dialog.HasSelected())
        {
            _ui.record_dlg_path = _save_dialog.GetSelected().string();
            _save_dialog.ClearSelected();
        }
    }

    void debugger_app::_open_device(uint32_t index, pose::view_plane_t view_plane)
    {
        _server->pipeline().open_source(
            app::source_address::device(index), 
            view_plane, 
            _opt.tag_size_m, 
            _opt.exposure_us, _opt.gain
        );
        _last_seq = 0;
        _pos_plot_bufs.clear(); // restart the positions-plot timeline for the new source
        _raw_skel_positions = {};
        _skel_plot_autofit_frames = kNumAutofitFrames; // re-fit the 3D box over the next frames of the new source
    }

    void debugger_app::_open_recording(const std::string& path, pose::view_plane_t view_plane)
    {
        _server->pipeline().open_source(
            app::source_address::recording(path), 
            view_plane, 
            _opt.tag_size_m, 
            std::nullopt, std::nullopt
        );
        _last_seq = 0;
        _pos_plot_bufs.clear(); // restart the positions-plot timeline for the new source
        _raw_skel_positions = {};
        _skel_plot_autofit_frames = kNumAutofitFrames; // re-fit the 3D box over the next frames of the new source
    }

    void debugger_app::_do_open_source()
    {
        _opt.view_plane = _ui.open_dlg_view_plane; // the dialog's choice becomes the session default

        if (_ui.open_dlg_kind == source_kind_t::recording)
        {
            if (_ui.open_dlg_recording.empty()) { spdlog::warn("no recording file selected"); return; }
            _opt.exposure_us.reset();
            _opt.gain.reset();
            this->_open_recording(_ui.open_dlg_recording, _ui.open_dlg_view_plane);
        }
        else
        {
            _opt.exposure_us = _ui.open_dlg_manual_exposure ? std::optional<int32_t>{ _ui.open_dlg_exposure } : std::nullopt;
            _opt.gain = _ui.open_dlg_manual_gain ? std::optional<int32_t>{ _ui.open_dlg_gain } : std::nullopt;
            this->_open_device(static_cast<uint32_t>(_ui.open_dlg_device), _ui.open_dlg_view_plane);
        }
        _ui.open_dlg_show = false;
    }

    void debugger_app::_do_close_source()
    {
        _server->pipeline().close_source();
        _last_seq = 0;
        _pos_plot_bufs.clear(); // restart the positions-plot timeline for the new source
        _raw_skel_positions = {};
        _skel_plot_autofit_frames = kNumAutofitFrames; // re-fit the 3D box over the next frames of the new source
    }

    void debugger_app::_update_pose_frame()
    {
        // Pull the server's latest annotated frame + detections. The server owns and updates the
        // estimator; nothing to do until a new frame arrives.
        std::chrono::microseconds ts{ 0 };
        if (!_server->pipeline().try_get_annotated_frame(_last_frame, _last_tag_detections, ts, _last_seq)) { return; }

        _frame_texture.value().update(_last_frame);

        const double t_now = std::chrono::duration<double>{ ts }.count();
        net::exo_pose_pipeline& pipe = _server->pipeline();
        const pose::pose_estimator_base* est = pipe.estimator();
        if (!est) { return; } // a frame arrived, so a source is open and so is its estimator

        // Capture the full per-frame trace into the rolling ring so a glitch can be dumped with its
        // lead-up right after it is seen on screen. Uses the same detections behind the plots below.
        // The gates are estimator specific, so they are read off whichever options exist.
        if (_ui.trace_enabled)
        {
            trace_gates_t gates;
            if (const auto* o = pipe.frontal_options()) {
                gates.max_hold_ms = o->max_hold.count();
                gates.reset_gap_ms = o->reset_gap.count();
                if (o->enable_hinge_constraint) { gates.hinge_axis = o->hinge_axis_world; }
            }
            else if (const auto* o = pipe.sagittal_options()) {
                gates.max_hold_ms = o->max_hold.count();
                gates.reset_gap_ms = o->reset_gap.count();
            }
            _trace.capture(ts, _last_tag_detections, *est, gates);
        }

        // Advance the positions-plot timeline.
        _pos_plot_bufs.advance(t_now);

        // Position source follows the smoothing switch: smoothed+held when on, raw when off.
        const bool smoothed_positions = est->uses_smoothed_positions();

        int ji = 0;
        for (const auto& info : pose::kJointsInfo)
        {
            const auto& st = est->get_joint_state(info.id);
            const std::optional<Eigen::Vector3d> p = smoothed_positions ? st.position : st.raw_position;
            _raw_skel_positions[ji] = p; // latest rig-space position for the skeleton plot
            if (p.has_value()) { _pos_plot_bufs.push(ji, rig_to_display(p.value())); } // display-space history
            ++ji;
        }
    }

    void debugger_app::_render_menu_bar()
    {
        if (!ImGui::BeginMainMenuBar()) { return; }
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open...")) { _ui.open_dlg_show = true; }
            if (ImGui::MenuItem("Close", nullptr, false, _server->pipeline().is_source_open())) { this->_do_close_source(); }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) { SDL_Event e{}; e.type = SDL_EVENT_QUIT; ::SDL_PushEvent(&e); }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Fullscreen", "F11", &_ui.camera_fullscreen);
            ImGui::MenuItem("Log Panel", nullptr, &_ui.show_log);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Record"))
        {
            net::exo_pose_pipeline& pipe = _server->pipeline();
            const bool recording = pipe.is_recording();
            // Only a live camera can be recorded; a playback source is already a recording.
            const bool can_record = pipe.is_source_open() && !pipe.is_source_recording() && !recording;

            if (ImGui::MenuItem("Start Recording...", nullptr, false, can_record))
            {
                if (_ui.record_dlg_path.empty()) { _ui.record_dlg_path = default_recording_name(); }
                _ui.record_dlg_show = true;
            }
            if (ImGui::MenuItem("Stop Recording", nullptr, false, recording)) { this->_do_stop_recording(); }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Server"))
        {
            const bool running = _server->is_listening();
            if (ImGui::MenuItem("Start Server", nullptr, false, !running)) { _server->start(); }
            if (ImGui::MenuItem("Stop Server", nullptr, false, running)) { _server->stop(); }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    void debugger_app::_render_control_panel()
    {
        net::exo_pose_pipeline& pipe = _server->pipeline();

        // Sensor info section
        if (ImGui::CollapsingHeader("Sensor Info", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (pipe.is_source_open())
            {
                // annotated sensor frame (texture) at the top of the section
                if (_frame_texture.value().valid())
                {
                    const float scale = ImGui::GetContentRegionAvail().x / _frame_texture.value().width();
                    ImGui::Image(_frame_texture.value().id(), ImVec2{ _frame_texture.value().width() * scale, _frame_texture.value().height() * scale });
                }
                else
                {
                    ImGui::TextUnformatted("Waiting for sensor frames...");
                }

                ImGui::TextUnformatted(std::format("Source : {}", pipe.source_name()).c_str());
                const auto res = pipe.source_resolution();
                ImGui::TextUnformatted(std::format("Resolution : {}x{}", res.x(), res.y()).c_str());
                ImGui::TextUnformatted(std::format("FPS : {:.1f}", pipe.source_fps()).c_str());

                if (pipe.is_source_recording())
                {
                    if (ImGui::Button(pipe.is_source_paused() ? " >" : "||")) {
                        pipe.set_source_paused(!pipe.is_source_paused());
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("|<")) { pipe.seek_to_begin(); }
                    ImGui::SameLine();
                    if (ImGui::Button(">|")) { pipe.seek_to_end(); }
                }

                this->_render_recording_status();
            }
            else
            {
                ImGui::TextUnformatted("No source opened. (File > Open...)");
            }
        }

        // Visualization section
        if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen))
        {
            constexpr std::array<const char*, 3> plot_types{ "Raw Skeleton", "Rig Skeleton", "Positions" };
            if (ImGui::BeginCombo("Plot Type", plot_types[static_cast<int>(_ui.plot_type)])) {
                for (size_t i = 0; i < plot_types.size(); ++i) {
                    const plot_type_t curr_plot_type = static_cast<plot_type_t>(i);
                    const bool selected = (curr_plot_type == _ui.plot_type);
                    if (ImGui::Selectable(plot_types[i], selected) && !selected) {
                        _ui.plot_type = curr_plot_type;
                        _skel_plot_autofit_frames = kNumAutofitFrames; // reframe the 3D box for the new view
                    }
                    if (selected) { ImGui::SetItemDefaultFocus(); }
                }
                ImGui::EndCombo();
            }
            ImGui::SetItemTooltip("Raw Skeleton: Measured raw skeleton (+ FK reconstruction overlay).\n"
                                  "Rig Skeleton: Fixed-length T-pose leg rig driven by local_anim_rot.\n"
                                  "Positions: Per-joint XYZ position channels as 2D line plots.");

            // Plot controls, matched to the selected type: the 3D skeletons get a box re-fit; the
            // positions grid gets the range lock/sync/reset and cell-size controls.
            if (_ui.plot_type == plot_type_t::positions)
            {
                ImGui::Checkbox("Lock Plots", &_ui.pos_plot_lock);
                ImGui::SetItemTooltip("Hold every subplot at its default Y range (no mouse pan/zoom on Y).\n"
                                      "On: ranges stay put, live. Off: Y is mouse-adjustable.");
                ImGui::SameLine();
                if (ImGui::Checkbox("Sync Plots", &_ui.pos_plot_sync)) { _pos_plot_reset = true; }
                ImGui::SetItemTooltip("Share one Y range across all joint subplots so they compare directly.");
                ImGui::SameLine();
                if (ImGui::Button("Reset Plots")) { _pos_plot_reset = true; }
                ImGui::SetItemTooltip("Return every subplot to its default Y range now.");

                ImGui::Checkbox("Auto-size Plots", &_ui.pos_plot_autosize);
                ImGui::SetItemTooltip("On: pack the subplots to fill the panel. Off: use a fixed cell size.");
                if (!_ui.pos_plot_autosize) {
                    ImGui::SliderFloat("Plots Size", &_ui.pos_plot_size_px, 80.0f, 400.0f, "%.0f px");
                }
            }
            else // raw/rig_skeleton (3D)
            {
                if (ImGui::Button("Fit view")) { _skel_plot_autofit_frames = kNumAutofitFrames; } // re-frame the 3D box
                ImGui::SetItemTooltip("Re-center/zoom the 3D view to the current skeleton.\n"
                                      "Zoom (wheel) / pan / rotate are otherwise free.");

                // Bind the style controls to the active skeleton mode's own fields.
                const bool is_raw = (_ui.plot_type == plot_type_t::raw_skeleton);
                float* point_size  = is_raw ? &_ui.raw_skel_point_size : &_ui.rig_skel_point_size;
                float* point_color = is_raw ? _ui.raw_skel_point_color : _ui.rig_skel_point_color;
                float* bone_color  = is_raw ? _ui.raw_skel_bone_color  : _ui.rig_skel_bone_color;

                constexpr ImGuiColorEditFlags col_flags = ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar;
                ImGui::DragFloat("Sphere size", point_size, 0.1f, 1.0f, 20.0f, "%.1f px", ImGuiSliderFlags_AlwaysClamp);
                ImGui::ColorEdit4("Sphere color", point_color, col_flags);
                ImGui::ColorEdit4("Bone color", bone_color, col_flags);
                if (is_raw) {
                    ImGui::ColorEdit4("FK overlay color", _ui.raw_skel_fk_bone_color, col_flags);
                }
            }
        }

        // Control section
        if (ImGui::CollapsingHeader("Control", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // ----- AprilTag detection tuning (live; the worker rebuilds the detector on change) -----
            ImGui::SeparatorText("AprilTag Detection");
            {
                const bool solves_tag_pose = (pipe.view_plane() == pose::view_plane_t::frontal);

                // Edit a copy of the current tuning, push it back only when something changed.
                pose::tag_tuning_t t = pipe.tag_tuning();
                bool changed = false;

                const double tag_min = 0.005, tag_max = 1.0;
                changed |= ImGui::DragScalar("Tag size [m]", ImGuiDataType_Double, &t.tag_size_m,
                    0.001f, &tag_min, &tag_max, "%.3f", ImGuiSliderFlags_AlwaysClamp);
                ImGui::SetItemTooltip("Real black-square edge length of the printed tag [m].\n"
                                      "Fixes the metric scale of every estimated 3D position; must match the tag.\n"
                                      "Higher: estimated depth and the whole skeleton scale up.\n"
                                      "Lower: they scale down.");

                // A sagittal run works off 2D tag centers, so the detector never solves a tag pose
                // and these two knobs have nothing to act on.
                ImGui::BeginDisabled(!solves_tag_pose);
                const char* const methods[] = { "Orthogonal iteration", "Homography (closed form)" };
                int mi = (t.pose_method == pose::tag_detector::pose_method_t::homography) ? 1 : 0;
                if (ImGui::Combo("Pose method", &mi, methods, IM_ARRAYSIZE(methods))) {
                    t.pose_method = (mi == 1) ? pose::tag_detector::pose_method_t::homography
                                              : pose::tag_detector::pose_method_t::orthogonal_iteration;
                    changed = true;
                }
                ImGui::SetItemTooltip("How tag->camera pose (hence the 3D position) is solved.\n"
                                      "OI: iterative; most accurate rotation, two candidates, costlier.\n"
                                      "Homography: closed-form; cheaper, translation/depth comparable.");
                ImGui::EndDisabled();

                changed |= ImGui::SliderFloat("quad_decimate", &t.quad_decimate, 1.0f, 4.0f, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                ImGui::SetItemTooltip("Image downsample factor before quad detection (1.0 = full res).\n"
                                      "The biggest detection CPU knob.\n"
                                      "Higher: much faster, but coarser corners (worse pose/depth) and\n"
                                      "        misses small/distant tags.\n"
                                      "Lower: slower, best corner accuracy.");

                changed |= ImGui::SliderFloat("quad_sigma", &t.quad_sigma, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
                ImGui::SetItemTooltip("Gaussian blur applied before detection (0 = none).\n"
                                      "Higher: smooths sensor noise (helps low-res/noisy), but erases small tags.\n"
                                      "Lower: sharper corners, no denoising.");

                changed |= ImGui::Checkbox("refine_edges", &t.refine_edges);
                ImGui::SetItemTooltip("Snap quad edges to image gradients for sub-pixel corners.\n"
                                      "On: better pose/depth accuracy, small extra cost.\n"
                                      "Off: faster, coarser corners (fine when decimating hard).");

                ImGui::BeginDisabled(!solves_tag_pose
                    || t.pose_method != pose::tag_detector::pose_method_t::orthogonal_iteration);
                changed |= ImGui::SliderInt("num_iters", &t.num_iters, 1, 100, "%d");
                ImGui::SetItemTooltip("Orthogonal-iteration steps for pose refinement (OI only).\n"
                                      "Higher: more accurate rotation, diminishing returns past ~50.\n"
                                      "Lower: faster, coarser pose.");
                ImGui::EndDisabled();

                changed |= ImGui::SliderInt("num_threads", &t.num_threads, 1, 16, "%d");
                ImGui::SetItemTooltip("Detector worker threads (no effect on accuracy).\n"
                                      "Higher: faster detection on multi-core CPUs.\n"
                                      "Lower: fewer cores used.");

                if (changed) { pipe.set_tag_tuning(t); } // worker rebuilds the detector next frame

                ImGui::TextDisabled("Applies live to an open source; rebuilds the detector.");
            }

            // ----- Rest Pose calibration options -----
            ImGui::SeparatorText("Rest Pose");
            {
                ImGui::TextUnformatted(pipe.has_rest_pose() ? "Rest Pose: calibrated" : "Rest Pose: N/A");
                ImGui::SameLine();
                if (ImGui::Button("Calibrate")) { pipe.calibrate_rest_pose(); } // the pipeline logs the outcome
                ImGui::SameLine();
                if (ImGui::Button("Clear")) { pipe.clear_rest_pose(); }
            }

            // ----- Estimator tuning (the two estimators expose different knobs) -----
            if (auto* frontal = pipe.frontal_options()) { this->_render_frontal_estimator_control(*frontal); }
            else if (auto* sagittal = pipe.sagittal_options()) { this->_render_sagittal_estimator_control(*sagittal); }

            // ----- Diagnostic pose trace -----
            // Rolling ring of full per-frame traces (detections + 3D positions + joint rotations). See a
            // glitch on screen, hit Dump, and the recent history lands in dumps/*.json for analysis.
            ImGui::SeparatorText("Diagnostics");
            {
                ImGui::Checkbox("Capture pose trace", &_ui.trace_enabled);
                ImGui::SetItemTooltip("Record each frame (tag detections + chosen 3D positions, per-joint\n"
                                      "raw/smoothed positions and animation rotation) into a rolling ring buffer.");

                if (ImGui::SliderInt("Trace length", &_ui.trace_capacity, 30, 3000, "%d frames")) {
                    _trace.set_capacity(static_cast<std::size_t>(_ui.trace_capacity));
                }

                ImGui::Text("Buffered: %zu / %d frames", _trace.size(), _ui.trace_capacity);
                if (ImGui::Button("Dump Trace")) { this->_dump_pose_trace(); }
                ImGui::SetItemTooltip("Write the buffered frames to dumps/pose_trace_*.json");
                ImGui::SameLine();
                if (ImGui::Button("Clear Trace")) { _trace.clear(); }
            }
        }
    }

    void debugger_app::_render_frontal_estimator_control(pose::frontal_pose_estimator::options_t& opt)
    {
        // ----- Leg hinge (1-DOF) -----
        ImGui::SeparatorText("Leg Hinge (1-DOF)");
        {
            ImGui::Checkbox("Constrain leg joints to 1-DOF hinge", &opt.enable_hinge_constraint);
            ImGui::SetItemTooltip("Every exo leg joint (hip/knee/ankle) is a forward/back hinge. Keep\n"
                                  "only the rotation about the lateral axis; drop off-hinge components\n"
                                  "as tag-position error. Needs a captured rest pose.\n"
                                  "On: clean 1-DOF swing per joint.\n"
                                  "Off: free minimal-swing (also picks up lateral wobble).");

            ImGui::BeginDisabled(!opt.enable_hinge_constraint);
            float axis[3]{ static_cast<float>(opt.hinge_axis_world.x()),
                           static_cast<float>(opt.hinge_axis_world.y()),
                           static_cast<float>(opt.hinge_axis_world.z()) };
            if (ImGui::DragFloat3("Hinge axis (cam)", axis, 0.01f, -1.0f, 1.0f, "%.2f")) {
                opt.hinge_axis_world = Eigen::Vector3d{ axis[0], axis[1], axis[2] };
            }
            ImGui::SetItemTooltip("Lateral hinge axis in the rig frame, shared by all leg joints.\n"
                                  "~(1,0,0) for a frontal view, ~(0,0,1) for a sagittal (side) view.\n"
                                  "It is normalized internally; direction matters, length does not.");
            ImGui::EndDisabled();
        }

        // ----- Position pipeline (rig-space 3D position track) -----
        ImGui::SeparatorText("Position (3D positions)");
        {
            ImGui::Checkbox("Enable position smoothing", &opt.enable_position_smoothing);
            ImGui::SetItemTooltip("Low-pass the 3D positions (One Euro per axis) and hold them briefly\n"
                                  "through occlusion. Also selects what the plots draw.\n"
                                  "On: steadier smoothed+held positions, some lag.\n"
                                  "Off: raw per-frame positions (noisier, no lag).");

            ImGui::BeginDisabled(!opt.enable_position_smoothing);
            option_drag("Min cutoff [Hz]", opt.position_filter.min_cutoff_hz, 0.01, 10.0, 0.01, "%.2f");
            ImGui::SetItemTooltip("Baseline low-pass cutoff for the 3D position while it is still.\n"
                                  "Higher: more responsive position, but more jitter.\n"
                                  "Lower: steadier position at rest, but adds lag.");
            option_drag("Beta", opt.position_filter.beta, 0.0, 1.0, 0.001, "%.3f");
            ImGui::SetItemTooltip("Speed coefficient: how much position motion raises the cutoff.\n"
                                  "Higher: less lag when the joint moves, more jitter.\n"
                                  "Lower: smoother in motion, more lag (0 = plain low-pass).");
            option_drag("Deriv cutoff [Hz]", opt.position_filter.dcutoff_hz, 0.01, 10.0, 0.01, "%.2f");
            ImGui::SetItemTooltip("Cutoff for the internal speed-estimate low-pass.\n"
                                  "Higher: speed reacts faster (beta engages sooner), a bit noisier.\n"
                                  "Lower: steadier speed estimate.");
            ImGui::EndDisabled();

            // Occlusion hold (independent of the smoothing on/off switch).
            double hold_ms = opt.max_hold.count();
            if (option_drag("Max hold [ms]", hold_ms, 0.0, 1000.0, 1.0, "%.0f")) { opt.max_hold = pose::millis_f64{ hold_ms }; }
            ImGui::SetItemTooltip("How long a lost joint keeps its last position before dropping out.\n"
                                  "Higher: rides through longer occlusions, but shows staler positions.\n"
                                  "Lower: drops a lost joint sooner (fresher, but blinks out more).");
            double reset_ms = opt.reset_gap.count();
            if (option_drag("Reset gap [ms]", reset_ms, 0.0, 2000.0, 1.0, "%.0f")) { opt.reset_gap = pose::millis_f64{ reset_ms }; }
            ImGui::SetItemTooltip("Gap after which the filter reseeds to the raw position instead of smoothing.\n"
                                  "Higher: keeps smoothing across longer pauses (may lurch on return).\n"
                                  "Lower: reseeds sooner after a pause (snappier, less overshoot).");
        }
    }

    void debugger_app::_render_sagittal_estimator_control(pose::sagittal_pose_estimator::options_t& opt)
    {
        // ----- Readout: which leg is being tracked and what it currently measures -----
        // The side is inferred from the detected tag ids, so showing it is the only way to catch a
        // camera placed on the wrong side (or a leg whose tags are not being seen at all).
        ImGui::SeparatorText("Tracked Leg");
        {
            const pose::sagittal_pose_estimator* est = _server->pipeline().sagittal_estimator();
            const auto knee = est ? est->tracked_leg_knee() : std::nullopt;

            if (knee.has_value()) {
                const auto& info = pose::joint_info(knee.value());
                ImGui::TextUnformatted(std::format("Near leg: {} (tag {})", info.name, info.tag_id).c_str());
            } else {
                ImGui::TextDisabled("Near leg: undecided (waiting for tags)");
            }
            ImGui::SetItemTooltip("Read off the detected tag ids: only one leg carries tags.\n"
                                  "It also tells which side the camera stands on, hence which way\n"
                                  "the legs swing.");

            const auto angles = est ? est->leg_angles() : std::nullopt;
            if (angles.has_value()) {
                ImGui::TextUnformatted(std::format("Hip   {:+7.2f} deg", angles->hip * kRadToDeg).c_str());
                ImGui::TextUnformatted(std::format("Knee  {:+7.2f} deg", angles->knee * kRadToDeg).c_str());
                ImGui::TextUnformatted(std::format("Ankle {:+7.2f} deg", angles->ankle * kRadToDeg).c_str());
            } else {
                ImGui::TextDisabled("Angles: n/a (needs a rest pose and the full chain visible)");
            }
        }

        // ----- Position track (image-plane points; angles are read off them) -----
        ImGui::SeparatorText("Position (image plane)");
        {
            ImGui::Checkbox("Enable position smoothing", &opt.enable_position_smoothing);
            ImGui::SetItemTooltip("Low-pass the tag centers (One Euro per image axis) and hold them briefly\n"
                                  "through occlusion. The angles are measured on these points, so this\n"
                                  "smooths the angles too. Also selects what the plots draw.\n"
                                  "On: steadier angles, some lag.\n"
                                  "Off: raw per-frame centers (noisier, no lag).");

            ImGui::BeginDisabled(!opt.enable_position_smoothing);
            option_drag("Min cutoff [Hz]", opt.position_filter.min_cutoff_hz, 0.01, 10.0, 0.01, "%.2f");
            ImGui::SetItemTooltip("Baseline low-pass cutoff for a tag center while it is still.\n"
                                  "Higher: more responsive angles, but more jitter.\n"
                                  "Lower: steadier angles at rest, but adds lag.");
            option_drag("Beta", opt.position_filter.beta, 0.0, 1.0, 0.001, "%.3f");
            ImGui::SetItemTooltip("Speed coefficient: how much marker motion raises the cutoff.\n"
                                  "Higher: less lag when the leg swings, more jitter.\n"
                                  "Lower: smoother in motion, more lag (0 = plain low-pass).");
            option_drag("Deriv cutoff [Hz]", opt.position_filter.dcutoff_hz, 0.01, 10.0, 0.01, "%.2f");
            ImGui::SetItemTooltip("Cutoff for the internal speed-estimate low-pass.\n"
                                  "Higher: speed reacts faster (beta engages sooner), a bit noisier.\n"
                                  "Lower: steadier speed estimate.");
            ImGui::EndDisabled();

            // Occlusion hold (independent of the smoothing on/off switch).
            double hold_ms = opt.max_hold.count();
            if (option_drag("Max hold [ms]", hold_ms, 0.0, 1000.0, 1.0, "%.0f")) { opt.max_hold = pose::millis_f64{ hold_ms }; }
            ImGui::SetItemTooltip("How long a lost joint keeps its last center before dropping out. The far\n"
                                  "leg swinging past the near one is the usual occlusion here.\n"
                                  "Higher: rides through longer occlusions, but holds staler angles.\n"
                                  "Lower: drops a lost joint sooner (fresher, but the chain breaks more).");
            double reset_ms = opt.reset_gap.count();
            if (option_drag("Reset gap [ms]", reset_ms, 0.0, 2000.0, 1.0, "%.0f")) { opt.reset_gap = pose::millis_f64{ reset_ms }; }
            ImGui::SetItemTooltip("Gap after which the filter reseeds to the raw center instead of smoothing.\n"
                                  "Higher: keeps smoothing across longer pauses (may lurch on return).\n"
                                  "Lower: reseeds sooner after a pause (snappier, less overshoot).");
        }
    }

    void debugger_app::_render_plot_panel()
    {
        switch (_ui.plot_type) {
        case plot_type_t::raw_skeleton:  this->_render_raw_skeleton_plot(); break;
        case plot_type_t::rig_skeleton:  this->_render_rig_skeleton_plot(); break;
        case plot_type_t::positions:     this->_render_positions_plot(); break;
        default: throw std::runtime_error{ "unknown plot type" };
        }
    }

    void debugger_app::_render_raw_skeleton_plot()
    {
        const auto to_display = [](const Eigen::Vector3d& p) -> Eigen::Vector3d { return rig_to_display(p).cast<double>(); };
        const pose::pose_estimator_base* est = _server->pipeline().estimator();

        // Measured per-joint positions (display space), smoothed or raw per the smoothing switch.
        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> measured{};
        for (std::size_t i = 0; i < pose::kNumJoints; ++i) {
            if (_raw_skel_positions[i].has_value()) { measured[i] = to_display(_raw_skel_positions[i].value()); }
        }

        // Forward-kinematics overlay: the per-joint anim rotations replayed on the captured rest
        // geometry, anchored at the measured pelvis, kept only where the joint was actually solved
        // this frame (anim present). Where it sits on the measured skeleton, the rotations agree
        // with the points they came from; where the two split, the rotations no longer describe what
        // was measured (the overlay keeps the rest bone lengths, so a length-only split is scale or
        // perspective rather than a wrong angle).
        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> overlay{};
        bool has_overlay = false;
        if (est && est->has_rest_pose())
        {
            std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> rest{};
            std::array<std::optional<Eigen::Quaterniond>, pose::kNumJoints> anim{};
            std::optional<Eigen::Vector3d> anchor; // measured position of the root
            for (const auto& info : pose::kJointsInfo) {
                const std::size_t k = static_cast<std::size_t>(info.id);
                rest[k] = est->get_rest_position(info.id);
                anim[k] = est->get_joint_state(info.id).local_anim_rot;
                if (pose::is_root_joint(info.id)) { anchor = est->get_joint_state(info.id).position; }
            }
            if (anchor.has_value())
            {
                const auto fk = rig_fk(rest, anim, anchor.value());
                for (std::size_t i = 0; i < pose::kNumJoints; ++i) {
                    if (fk[i].has_value() && anim[i].has_value()) { overlay[i] = to_display(fk[i].value()); has_overlay = true; }
                }
            }
        }

        const ImVec4 bone_col(_ui.raw_skel_bone_color[0], _ui.raw_skel_bone_color[1], _ui.raw_skel_bone_color[2], _ui.raw_skel_bone_color[3]);
        const ImVec4 point_col(_ui.raw_skel_point_color[0], _ui.raw_skel_point_color[1], _ui.raw_skel_point_color[2], _ui.raw_skel_point_color[3]);
        const ImVec4 fk_col(_ui.raw_skel_fk_bone_color[0], _ui.raw_skel_fk_bone_color[1], _ui.raw_skel_fk_bone_color[2], _ui.raw_skel_fk_bone_color[3]);
        this->_render_skeleton_3d(
            "Raw skeleton (measured positions + FK overlay)",
            measured, bone_col, point_col,
            _ui.raw_skel_point_size,
            has_overlay ? &overlay : nullptr,
            fk_col,
            /*hint*/nullptr
        );
    }

    void debugger_app::_render_rig_skeleton_plot()
    {
        const auto to_display = [](const Eigen::Vector3d& p) -> Eigen::Vector3d { return rig_to_display(p).cast<double>(); };
        const pose::pose_estimator_base* est = _server->pipeline().estimator();

        // Fixed-length clean rig (xbot-like T-pose) driven by the live per-joint rotations; FK
        // anchored at the rig root. With no rest pose captured every anim rotation is absent, so the rig
        // holds its neutral T-pose.
        const auto rest = canonical_rest_layout();
        std::array<std::optional<Eigen::Quaterniond>, pose::kNumJoints> anim{};
        if (est) {
            for (const auto& info : pose::kJointsInfo) {
                anim[static_cast<std::size_t>(info.id)] = est->get_joint_state(info.id).local_anim_rot;
            }
        }
        const Eigen::Vector3d anchor =
            rest[static_cast<std::size_t>(pose::kJointsInfo[0].id)].value_or(Eigen::Vector3d::Zero());
        const auto world = rig_fk(rest, anim, anchor);

        std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints> disp{};
        for (std::size_t i = 0; i < pose::kNumJoints; ++i) {
            if (world[i].has_value()) { disp[i] = to_display(world[i].value()); }
        }

        const ImVec4 bone_col(_ui.rig_skel_bone_color[0], _ui.rig_skel_bone_color[1], _ui.rig_skel_bone_color[2], _ui.rig_skel_bone_color[3]);
        const ImVec4 point_col(_ui.rig_skel_point_color[0], _ui.rig_skel_point_color[1], _ui.rig_skel_point_color[2], _ui.rig_skel_point_color[3]);
        this->_render_skeleton_3d(
            "Rig skeleton (local_anim_rot on a fixed-length rig)",
            disp, bone_col, point_col,
            _ui.rig_skel_point_size,
            /*overlay*/nullptr,
            /*overlay_color*/ImVec4{},
            _server->pipeline().has_rest_pose() ? nullptr : "calibrate a rest pose to animate"
        );
    }

    void debugger_app::_render_positions_plot()
    {
        const int n = static_cast<int>(pose::kNumJoints);
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        const ImVec2 avail = ImGui::GetContentRegionAvail();

        // Subplot cell size: pack to fill the panel, or a fixed DPI-scaled size.
        int cols = 1;
        float cell = 1.0f;
        if (_ui.pos_plot_autosize) {
            for (int c = 1; c <= n; ++c) {
                const int r = (n + c - 1) / c;
                const float cw = (avail.x - spacing * (c - 1)) / c;
                const float ch = (avail.y - spacing * (r - 1)) / r;
                if (const float s = std::min(cw, ch); s > cell) { cell = s; cols = c; }
            }
        } else {
            cell = _ui.pos_plot_size_px * this->renderer().dpi_scale(); // DPI-aware px
            cols = std::max(1, static_cast<int>((avail.x + spacing) / (cell + spacing)));
        }
        const ImVec2 cell_sz{ cell, cell };

        // Y range: Lock (or a one-shot Reset) forces the default; otherwise it is mouse-adjustable
        // (set once). Sync links one Y range across every subplot. X always scrolls the newest window.
        constexpr float kYLo = -1.2f, kYHi = 1.2f; // default position range [m], display space
        const ImPlotCond y_cond = (_ui.pos_plot_lock || _pos_plot_reset) ? ImPlotCond_Always : ImPlotCond_Once;

        // Channels are plot space: rig X, rig Z, and -rig Y (the 3D views label the same three axes
        // right / depth / up).
        const ImVec4 axis_col[3]{ { 0.95f, 0.35f, 0.35f, 1 }, { 0.45f, 0.85f, 0.45f, 1 }, { 0.45f, 0.55f, 0.95f, 1 } };
        const char* const axis_nm[3]{ "x", "y", "z" };

        int col = 0;
        for (std::size_t i = 0; i < pose::kNumJoints; ++i)
        {
            const std::string title = std::format("{}###{}", pose::kJointsInfo[i].name, pose::kJointsInfo[i].name);
            if (col != 0) { ImGui::SameLine(); }
            ImGui::PushID(static_cast<int>(i));
            ImGui::BeginGroup();
            draw_plot_lines(title.c_str(), _pos_plot_bufs.view(i), kPositionsWindowSec, kYLo, kYHi,
                       y_cond, _ui.pos_plot_sync, _pos_plot_sync_y, axis_col, axis_nm, cell_sz);
            ImGui::EndGroup();
            ImGui::PopID();
            if (++col >= cols) { col = 0; }
        }

        _pos_plot_reset = false; // one-shot: the ranges were forced this frame
    }

    void debugger_app::_render_skeleton_3d(
        const char* title,
        const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>& disp,
        ImVec4 bone_color, ImVec4 point_color, 
        float point_size,
        const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>* overlay,
        ImVec4 overlay_color,
        const char* hint)
    {
        const ImVec2 avail = ImGui::GetContentRegionAvail();

        // Fit the box to the PRIMARY skeleton's positions only. The overlay is drawn but deliberately
        // left out of the fit: if the captured rest is off, the reconstructed overlay can land far
        // away, and letting it drive the box would shrink the real skeleton to a dot.
        Eigen::Vector3d bb_min = Eigen::Vector3d::Zero(), bb_max = Eigen::Vector3d::Zero();
        int npts = 0;
        for (const auto& v : disp) {
            if (!v.has_value()) { continue; }
            if (npts == 0) { bb_min = bb_max = v.value(); }
            else { bb_min = bb_min.cwiseMin(v.value()); bb_max = bb_max.cwiseMax(v.value()); }
            ++npts;
        }

        const bool do_fit = _skel_plot_autofit_frames > 0;

        const ImPlot3DFlags f3d = ImPlot3DFlags_Equal | ImPlot3DFlags_NoClip | ImPlot3DFlags_NoLegend;
        if (!ImPlot3D::BeginPlot(title, avail, f3d)) { return; }
        ImPlot3D::SetupAxes("right [m]", "depth [m]", "up [m]"); // plot X=rig X, Y=rig Z (depth), Z=-rig Y (up)
        {
            const ImPlot3DQuat r = front_view_quat();
            ImPlot3D::SetupBoxInitialRotation(r); // double-click reset returns to front
            ImPlot3D::SetupBoxRotation(r, false, ImPlot3DCond_Once); // open facing front
        }
        // Fit the equal-scaled cube to the data for the auto-fit window (after a source/view change),
        // then leave the range to the user so wheel zoom / pan / rotate work freely.
        const Eigen::Vector3d center = (npts > 0) ? Eigen::Vector3d{ 0.5 * (bb_min + bb_max) }
                                                  : Eigen::Vector3d{ 0.0, 0.0, -0.4 };
        const double half = (npts > 0) ? std::max(0.5 * (bb_max - bb_min).maxCoeff() * 1.3, 0.15) : 0.6;
        ImPlot3D::SetupAxesLimits(
            center.x() - half, center.x() + half,
            center.y() - half, center.y() + half,
            center.z() - half, center.z() + half,
            do_fit ? ImPlot3DCond_Always : ImPlot3DCond_Once
        );
        if (do_fit && npts > 0) { --_skel_plot_autofit_frames; }

        // parent->child bone segments for a skeleton.
        // (overlay shares one label so its bones don't collide with the primary per-bone ids)
        const auto draw_bones = [](
            const std::array<std::optional<Eigen::Vector3d>, pose::kNumJoints>& a,
            ImVec4 color, float weight, const char* label)
        {
            ImPlot3DSpec bone;
            bone.LineWeight = weight;
            bone.LineColor = color;
            for (const auto& info : pose::kJointsInfo) {
                if (pose::is_root_joint(info.id)) { continue; }
                const std::size_t c = static_cast<std::size_t>(info.id);
                const std::size_t p = static_cast<std::size_t>(info.parent);
                if (!a[c].has_value() || !a[p].has_value()) { continue; }
                const double bx[2]{ a[p]->x(), a[c]->x() };
                const double by[2]{ a[p]->y(), a[c]->y() };
                const double bz[2]{ a[p]->z(), a[c]->z() };
                ImPlot3D::PlotLine(label != nullptr ? label : info.name.data(), bx, by, bz, 2, bone);
            }
        };

        draw_bones(disp, bone_color, 3.0f, /*label*/ nullptr);

        // Joints: one scatter of every present primary position, plus a text label per joint.
        std::array<double, pose::kNumJoints> jx{}, jy{}, jz{};
        int jn = 0;
        for (std::size_t i = 0; i < pose::kNumJoints; ++i) {
            if (!disp[i].has_value()) { continue; }
            jx[jn] = disp[i]->x(); jy[jn] = disp[i]->y(); jz[jn] = disp[i]->z(); ++jn;
            ImPlot3D::PlotText(pose::kJointsInfo[i].name.data(), disp[i]->x(), disp[i]->y(), disp[i]->z());
        }
        ImPlot3DSpec pt_spec;
        pt_spec.Marker = ImPlot3DMarker_Circle;
        pt_spec.MarkerSize = point_size;
        pt_spec.MarkerFillColor = point_color;
        ImPlot3D::PlotScatter("joints", jx.data(), jy.data(), jz.data(), jn, pt_spec);

        // Optional second skeleton (the forward-kinematics reconstruction), thinner and in `overlay_color`.
        if (overlay != nullptr) { draw_bones(*overlay, overlay_color, 2.0f, "fk"); }

        if (hint != nullptr) { ImPlot3D::PlotText(hint, 0.0, 0.0, 0.15); }
        ImPlot3D::EndPlot();
    }

    float debugger_app::_log_split_height()
    {
        const float avail_y = ImGui::GetContentRegionAvail().y;
        const float max_log = std::max(kLogMinH, avail_y - kSplitHit - kLogMinH);
        _ui.log_panel_height = std::clamp(_ui.log_panel_height, kLogMinH, max_log);
        return avail_y - _ui.log_panel_height - kSplitHit;
    }

    void debugger_app::_render_log_panel()
    {
        // Starting a new line after the content row already advanced the cursor by one
        // ItemSpacing.y; undo it so the grip sits flush against the row. Without this the
        // vertical gap would be ItemSpacing.y wider than the (SameLine-flush) side splitter.
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() - ImGui::GetStyle().ItemSpacing.y);

        // Horizontal resize grip (no visible line): zero spacing keeps it flush to both
        // panes, so the inter-panel gap matches the vertical splitter's width and the
        // whole gap is grabbable. Drag up to grow the panel, down to shrink it.
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
        ImGui::InvisibleButton("##log_split", ImVec2(-1.0f, kSplitHit));
        if (ImGui::IsItemActive()) { _ui.log_panel_height -= ImGui::GetIO().MouseDelta.y; }
        if (ImGui::IsItemHovered() || ImGui::IsItemActive()) { ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS); }

        ImGui::BeginChild("logpanel", ImVec2(0.0f, _ui.log_panel_height), ImGuiChildFlags_Borders);
        ImGui::PopStyleVar(); // restore spacing for the console's own contents
        _log_console.draw();
        ImGui::EndChild();
    }

    void debugger_app::_do_start_recording()
    {
        std::filesystem::path path{ _ui.record_dlg_path };
        if (path.empty()) { return; }

        // The browser lets a name through without one, but the reader finds recordings by
        // extension and so does the user.
        if (path.extension() != ".mcap") { path.replace_extension(".mcap"); }

        const size_t index = std::clamp<size_t>(
            static_cast<size_t>(_ui.record_dlg_codec), 0, io::kImageCodecs.size() - 1);
        const io::recording_options options{
            .codec = io::kImageCodecs[index].codec,
            .encode = { .jpeg_quality = _ui.record_dlg_jpeg_quality },
        };

        if (_server->pipeline().start_recording(path, options))
        {
            _ui.record_dlg_path.clear(); // the next take gets a fresh timestamped name
            _ui.record_dlg_show = false;
        }
    }

    void debugger_app::_do_stop_recording()
    {
        _server->pipeline().stop_recording();
    }

    void debugger_app::_dump_pose_trace()
    {
        if (_trace.empty()) {
            spdlog::warn("pose trace: nothing captured yet (enable capture and let a source run)");
            return;
        }

        const net::exo_pose_pipeline& pipe = _server->pipeline();

        std::error_code ec;
        const std::filesystem::path dir{ "dumps" };
        std::filesystem::create_directories(dir, ec); // best-effort; write_json reports a real failure
        const std::filesystem::path path = dir / default_trace_name();

        _trace.write_json(
            path,
            pipe.source_name(),
            pipe.source_resolution(),
            pipe.source_fps(),
            pipe.intrinsics(),
            pipe.view_plane()
        );
    }

    void debugger_app::_render_recording_status()
    {
        const net::exo_pose_pipeline& pipe = _server->pipeline();
        if (!pipe.is_recording()) { return; }

        const io::recording_stats stats = pipe.recording_stats();
        const double seconds = std::chrono::duration<double>{ stats.duration }.count();
        const double megabytes = static_cast<double>(stats.file_bytes) / (1024.0 * 1024.0);

        ImGui::Separator();
        ImGui::TextColored(ImVec4{ 0.90f, 0.30f, 0.30f, 1.0f }, "%s",
            std::format("REC  {}", pipe.recording_path().filename().string()).c_str());
        ImGui::TextUnformatted(std::format("Elapsed: {:.1f} s ({} frames)", seconds, stats.frames_written).c_str());
        ImGui::TextUnformatted(std::format("File   : {:.1f} MB ({:.1f} MB/s)",
            megabytes, seconds > 0.0 ? megabytes / seconds : 0.0).c_str());

        // Dropped frames mean the disk or the encoder fell behind; the recording is still
        // valid, just missing frames, so say so rather than failing silently.
        if (stats.frames_dropped > 0)
        {
            ImGui::TextColored(ImVec4{ 0.90f, 0.60f, 0.20f, 1.0f }, "%s",
                std::format("Dropped: {} frame(s)", stats.frames_dropped).c_str());
        }

        if (ImGui::Button("Stop Recording")) { this->_do_stop_recording(); }
    }

    void debugger_app::_render_record_dialog()
    {
        if (!_ui.record_dlg_show) { return; }

        ImGui::SetNextWindowSize(ImVec2(460, 0), ImGuiCond_Appearing);
        if (ImGui::Begin("Start Recording", &_ui.record_dlg_show, ImGuiWindowFlags_NoCollapse))
        {
            if (ImGui::Button("Browse...")) { _save_dialog.Open(); }
            ImGui::SameLine();
            ImGui::TextUnformatted(_ui.record_dlg_path.empty() ? "(no file selected)" : _ui.record_dlg_path.c_str());

            ImGui::Combo("Codec", &_ui.record_dlg_codec, kCodecLabels.data(), static_cast<int>(kCodecLabels.size()));

            const bool is_jpeg = (io::kImageCodecs[static_cast<size_t>(_ui.record_dlg_codec)].codec
                == io::image_codec::jpeg);

            ImGui::BeginDisabled(!is_jpeg);
            ImGui::SliderInt("JPEG quality", &_ui.record_dlg_jpeg_quality, 1, 100);
            ImGui::EndDisabled();

            ImGui::TextWrapped("%s", is_jpeg
                ? "Lossy. Roughly 5-15 MB/s at 1080p30."
                : "Lossless pixels, compressed per chunk. Much larger; meant for short clips.");

            ImGui::Separator();
            ImGui::BeginDisabled(_ui.record_dlg_path.empty());
            if (ImGui::Button("Start", ImVec2(90, 0))) { this->_do_start_recording(); }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(90, 0))) { _ui.record_dlg_show = false; }
        }
        ImGui::End();
    }

    void debugger_app::_render_open_dialog()
    {
        if (!_ui.open_dlg_show) { return; }
        ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_Appearing);
        if (ImGui::Begin("Open Source", &_ui.open_dlg_show, ImGuiWindowFlags_NoCollapse))
        {
            // The viewing plane decides which estimator runs, and swapping it mid-stream would
            // invalidate the rest pose and the tracking state, so it is picked alongside the source.
            const auto view_plane_radio = [this](const char* label, pose::view_plane_t val) {
                if (ImGui::RadioButton(label, _ui.open_dlg_view_plane == val)) { _ui.open_dlg_view_plane = val; }
            };
            ImGui::TextUnformatted("Viewing plane");
            view_plane_radio("Frontal", pose::view_plane_t::frontal);
            ImGui::SameLine();
            view_plane_radio("Sagittal", pose::view_plane_t::sagittal);
            ImGui::SetItemTooltip("Frontal: camera faces the exo; both legs tagged, rig solved in 3D.\n"
                                  "Sagittal: camera at the side; only the near leg is tagged and its\n"
                                  "          angles are read off the image plane (no tag pose solve).");
            ImGui::Separator();

            const auto kind_radio = [this](const char* label, source_kind_t val) {
                if (ImGui::RadioButton(label, _ui.open_dlg_kind == val)) { _ui.open_dlg_kind = val; }
            };
            kind_radio("Device", source_kind_t::device);
            ImGui::SameLine();
            kind_radio("Recording", source_kind_t::recording);
            ImGui::Separator();

            if (_ui.open_dlg_kind == source_kind_t::device)
            {
                ImGui::InputInt("Device index", &_ui.open_dlg_device);
                if (_ui.open_dlg_device < 0) { _ui.open_dlg_device = 0; }
                ImGui::Checkbox("Manual exposure [us]", &_ui.open_dlg_manual_exposure);
                if (_ui.open_dlg_manual_exposure)
                {
                    ImGui::SameLine(); ImGui::SetNextItemWidth(120);
                    ImGui::InputInt("##exposure", &_ui.open_dlg_exposure);
                }
                ImGui::Checkbox("Manual gain", &_ui.open_dlg_manual_gain);
                if (_ui.open_dlg_manual_gain)
                {
                    ImGui::SameLine(); ImGui::SetNextItemWidth(120);
                    ImGui::InputInt("##gain", &_ui.open_dlg_gain);
                }
            }
            else
            {
                if (ImGui::Button("Browse...")) { _file_dialog.Open(); }
                ImGui::SameLine();
                ImGui::TextUnformatted(_ui.open_dlg_recording.empty() ? "(no file selected)" : _ui.open_dlg_recording.c_str());
            }

            ImGui::Separator();
            if (ImGui::Button("Open", ImVec2(90, 0))) { this->_do_open_source(); }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(90, 0))) { _ui.open_dlg_show = false; }
        }
        ImGui::End();
    }

} // namespace gui
