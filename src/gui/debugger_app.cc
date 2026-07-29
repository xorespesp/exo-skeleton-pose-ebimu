#include "debugger_app.hh"

#include "net/exo_pose_server.hh"
#include "net/exo_pose_pipeline.hh"

#include <spdlog/spdlog.h>

#include <imgui.h>
#include <implot.h>
#include <implot3d.h>
#include <implot3d_internal.h> // GetCurrentPlot / ImPlot3DPlot for axis-frame sync

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <format>
#include <numbers>

namespace gui
{
    namespace
    {
        // Codec picker entries, one per io::kImageCodecs row and in the same order.
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

        // Selectable euler decomposition order (Eigen axis indices: 0=X, 1=Y, 2=Z).
        struct euler_order_t { const char* name; int a0, a1, a2; };
        constexpr std::array<euler_order_t, 6> kEulerOrders{ {
            { "XYZ", 0, 1, 2 }, { "XZY", 0, 2, 1 }, { "YXZ", 1, 0, 2 },
            { "YZX", 1, 2, 0 }, { "ZXY", 2, 0, 1 }, { "ZYX", 2, 1, 0 },
        } };

        Eigen::Vector3d to_euler_deg(const Eigen::Quaterniond& q, const euler_order_t& order)
        {
            return q.toRotationMatrix().eulerAngles(order.a0, order.a1, order.a2) * (180.0 / std::numbers::pi);
        }

        std::optional<Eigen::Quaterniond> try_get_joint_rot(const pose::joint_state_t& st, bool relative)
        {
            if (relative) { return st.local_anim_rot; }
            if (st.view_pose.has_value()) { return Eigen::Quaterniond{ st.view_pose.value().rotation() }.normalized(); }
            return std::nullopt;
        }

        // RGB axis triad (X=red, Y=green, Z=blue) for `q`, into the current ImPlot3D plot.
        void draw_axes(const Eigen::Quaterniond& q, float thickness)
        {
            const Eigen::Vector3d ex = q * Eigen::Vector3d::UnitX();
            const Eigen::Vector3d ey = q * Eigen::Vector3d::UnitY();
            const Eigen::Vector3d ez = q * Eigen::Vector3d::UnitZ();

            const double xx[2]{ 0.0, ex.x() }, xy[2]{ 0.0, ex.y() }, xz[2]{ 0.0, ex.z() };
            const double yx[2]{ 0.0, ey.x() }, yy[2]{ 0.0, ey.y() }, yz[2]{ 0.0, ey.z() };
            const double zx[2]{ 0.0, ez.x() }, zy[2]{ 0.0, ez.y() }, zz[2]{ 0.0, ez.z() };

            // NOTE: Use short legend names; the caller wraps each subplot in PushID/PopID so ids stay unique.
            ImPlot3DSpec spec;
            spec.LineWeight = thickness;
            spec.LineColor = ImVec4(1, 0, 0, 1);
            ImPlot3D::PlotLine("X", xx, xy, xz, 2, spec);
            spec.LineColor = ImVec4(0, 1, 0, 1);
            ImPlot3D::PlotLine("Y", yx, yy, yz, 2, spec);
            spec.LineColor = ImVec4(0, 0, 1, 1);
            ImPlot3D::PlotLine("Z", zx, zy, zz, 2, spec);
        }

        // Splitter grip thickness [px]. It doubles as the inter-panel gap: surrounding
        // ItemSpacing is zeroed so the visible border-to-border gap equals this on both
        // axes, and the whole gap is the drag hit-target (same width for v/h splitters).
        constexpr float kSplitHit = 6.0f;
        constexpr float kLogMinH = 60.0f;  // min height for both the content and log panes [px]
        constexpr float kPlotMinW = 200.0f; // min width for the plots pane [px]
        constexpr float kSideMinW = 200.0f; // min width for the control pane [px]

        constexpr float kWindowSec = 10.0f; // scrolling line-plot window [s]
        constexpr float kEulerYLo = -190.0f, kEulerYHi = 190.0f; // euler deg range (all subplots)
        constexpr float kQuatYLo = -1.1f, kQuatYHi = 1.1f; // quaternion range (all subplots)

        // Scrolling line plot of a buffer's channels over the newest `window` seconds.
        // x is the device time (`v.xs`); channel k is `v.ys.data() + k`, both strided by `v.stride`.
        // x always auto-scrolls; only y obeys `y_cond` (Always locks, Once leaves it mouse-free) and
        // `sync` (links y to the shared `sy` so all subplots share one y range).
        template <typename _Scalar>
        void draw_lines(
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
            // x always tracks the newest `window` seconds; y follows lock/sync.
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
        if (opt.exposure_us.has_value()) { _ui.manual_exposure = true; _ui.exposure = opt.exposure_us.value(); }
        if (opt.gain.has_value()) { _ui.manual_gain = true; _ui.gain = opt.gain.value(); }

        // The dialog holds a value per kind so toggling does not discard what was typed;
        // only fill the one the command line named.
        if (opt.source_addr.has_value())
        {
            if (opt.source_addr->is_device()) {
                _ui.device = static_cast<int>(opt.source_addr->device_index());
                _ui.open_kind = source_kind_t::device;
            } else {
                _ui.recording = opt.source_addr->recording_path().string();
                _ui.open_kind = source_kind_t::recording;
            }
        }
        _ui.show_log = true; // surface the log console by default

        _file_dialog.SetTitle("Open recording file");
        _file_dialog.SetTypeFilters({ ".mcap" });

        _save_dialog.SetTitle("Save recording as");
        _save_dialog.SetTypeFilters({ ".mcap" });

        // Mirror spdlog output into the in-GUI log console. Registered here (main thread,
        // before any capture worker exists) so appending to the sink list is race-free.
        //
        // Records every severity, unlike the terminal sink: the console's own toggles filter the
        // view, and a level turned back on must be able to reveal what it already missed.
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
        if (!_texture.has_value()) { _texture.emplace(this->renderer().sdl_renderer()); }

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
            if (!_texture.value().valid()) { ImGui::TextUnformatted("Waiting for frames...  (F11 to exit)"); }
            else
            {
                const ImVec2 avail = ImGui::GetContentRegionAvail();
                const float tw = static_cast<float>(_texture.value().width());
                const float th = static_cast<float>(_texture.value().height());
                const float scale = std::min(avail.x / tw, avail.y / th);
                const ImVec2 sz{ tw * scale, th * scale };
                const ImVec2 cur = ImGui::GetCursorPos();
                ImGui::SetCursorPos(ImVec2{ cur.x + (avail.x - sz.x) * 0.5f, cur.y + (avail.y - sz.y) * 0.5f });
                ImGui::Image(_texture.value().id(), sz);
            }
        }
        else
        {
            // Normal: plot panel (left) + control panel (right), split by a grip whose
            // width equals the log splitter's so every gap looks identical.
            const float avail_x = ImGui::GetContentRegionAvail().x;
            const float max_side = std::max(kSideMinW, avail_x - kSplitHit - kPlotMinW);
            _ui.side_w = std::clamp(_ui.side_w, kSideMinW, max_side);
            const float plots_w = avail_x - _ui.side_w - kSplitHit;

            ImGui::BeginChild("plots", ImVec2(plots_w, row_h), ImGuiChildFlags_Borders);
            this->_render_plot_panel();
            ImGui::EndChild();

            // Vertical resize grip (no visible line): flush to both panes (zero spacing),
            // so the whole inter-panel gap is grabbable. Drag left to grow the control pane.
            ImGui::SameLine(0.0f, 0.0f);
            ImGui::InvisibleButton("##side_split", ImVec2(kSplitHit, row_h));
            if (ImGui::IsItemActive()) { _ui.side_w -= ImGui::GetIO().MouseDelta.x; }
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
            _ui.recording = _file_dialog.GetSelected().string();
            _ui.open_kind = source_kind_t::recording;
            _file_dialog.ClearSelected();
        }

        _save_dialog.Display();
        if (_save_dialog.HasSelected())
        {
            _ui.record_path = _save_dialog.GetSelected().string();
            _save_dialog.ClearSelected();
        }
    }

    void debugger_app::_open_device(uint32_t index)
    {
        _server->pipeline().open_device(index, _opt.exposure_us, _opt.gain);
        _last_seq = 0;
        _euler_bufs.clear();
        _quat_bufs.clear();
    }

    void debugger_app::_open_recording(const std::string& path)
    {
        _server->pipeline().open_recording(path);
        _last_seq = 0;
        _euler_bufs.clear();
        _quat_bufs.clear();
    }

    void debugger_app::_do_open_source()
    {
        if (_ui.open_kind == source_kind_t::recording)
        {
            if (_ui.recording.empty()) { spdlog::warn("no recording file selected"); return; }
            _opt.exposure_us.reset();
            _opt.gain.reset();
            this->_open_recording(_ui.recording);
        }
        else
        {
            _opt.exposure_us = _ui.manual_exposure ? std::optional<int32_t>{ _ui.exposure } : std::nullopt;
            _opt.gain = _ui.manual_gain ? std::optional<int32_t>{ _ui.gain } : std::nullopt;
            this->_open_device(static_cast<uint32_t>(_ui.device));
        }
        _ui.show_open = false;
    }

    void debugger_app::_do_close_source()
    {
        _server->pipeline().close_source();
        _last_seq = 0;
        _euler_bufs.clear();
        _quat_bufs.clear();
    }

    void debugger_app::_update_pose_frame()
    {
        // Pull the server's latest annotated frame + detections. The server owns and updates the
        // estimator; nothing to do until a new frame arrives.
        std::chrono::microseconds ts{ 0 };
        if (!_server->pipeline().try_get_annotated_frame(_frame, _detections, ts, _last_seq)) { return; }

        _texture.value().update(_frame);

        // Append the current per-joint euler/quat samples, stamped with the device frame time.
        const double ts_sec = std::chrono::duration<double>{ ts }.count();
        _euler_bufs.advance(ts_sec);
        _quat_bufs.advance(ts_sec);

        const pose::exo_pose_estimator& est = _server->pipeline().estimator();
        int ji = 0;
        for (const auto& info : pose::kJointsInfo)
        {
            const auto& st = est.get_joint_state(info.id);
            const auto rot = try_get_joint_rot(st, _ui.relative_rot);
            if (rot.has_value())
            {
                const Eigen::Vector3d e = to_euler_deg(rot.value(), kEulerOrders[_ui.euler_order]);
                _euler_bufs.push(ji, e.cast<float>());
                _quat_bufs.push(ji, rot.value().cast<float>());
            }
            else // gap while the joint has no rotation this frame; NaN breaks the line
            {
                Eigen::Quaternionf qn;
                qn.coeffs().setConstant(std::nanf(""));
                _euler_bufs.push(ji, Eigen::Vector3f::Constant(std::nanf("")));
                _quat_bufs.push(ji, qn);
            }
            ++ji;
        }
    }

    void debugger_app::_render_menu_bar()
    {
        if (!ImGui::BeginMainMenuBar()) { return; }
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open...")) { _ui.show_open = true; }
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
                if (_ui.record_path.empty()) { _ui.record_path = default_recording_name(); }
                _ui.show_record = true;
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
                if (_texture.value().valid())
                {
                    const float scale = ImGui::GetContentRegionAvail().x / _texture.value().width();
                    ImGui::Image(_texture.value().id(), ImVec2{ _texture.value().width() * scale, _texture.value().height() * scale });
                }
                else
                {
                    ImGui::TextUnformatted("Waiting for sensor frames...");
                }

                ImGui::TextUnformatted(std::format("Source : {}", pipe.source_name()).c_str());
                const auto res = pipe.source_resolution();
                ImGui::TextUnformatted(std::format("Color  : {}x{}", res.x(), res.y()).c_str());
                ImGui::TextUnformatted(std::format("FPS    : {:.1f}", pipe.source_fps()).c_str());

                if (pipe.is_source_recording())
                {
                    if (ImGui::Button(pipe.is_source_paused() ? "Play" : "Pause")) {
                        pipe.set_source_paused(!pipe.is_source_paused());
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("|< Begin")) { pipe.seek_to_begin(); }
                    ImGui::SameLine();
                    if (ImGui::Button("End >|")) { pipe.seek_to_end(); }
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
            if (ImGui::Checkbox("Relative Rotation", &_ui.relative_rot)) {
                // A rotation-basis change invalidates both histories
                _euler_bufs.clear();
                _quat_bufs.clear();
            }

            if (ImGui::BeginCombo("Euler Order", kEulerOrders[_ui.euler_order].name)) {
                for (int i = 0; i < static_cast<int>(kEulerOrders.size()); ++i) {
                    const bool selected = (i == _ui.euler_order);
                    if (ImGui::Selectable(kEulerOrders[i].name, selected) && i != _ui.euler_order) {
                        _ui.euler_order = i;
                        _euler_bufs.clear(); // quats are order-independent; keep their history
                    }
                    if (selected) { ImGui::SetItemDefaultFocus(); }
                }
                ImGui::EndCombo();
            }

            constexpr std::array<const char*, 3> plot_types{ "Axis Frame", "Euler Angles", "Quaternion" };
            if (ImGui::BeginCombo("Plot Type", plot_types[static_cast<int>(_ui.plot_type)])) {
                for (size_t i = 0; i < plot_types.size(); ++i) {
                    const plot_type_t curr_plot_type = static_cast<plot_type_t>(i);
                    const bool selected = (curr_plot_type == _ui.plot_type);
                    if (ImGui::Selectable(plot_types[i], selected) && !selected) {
                        _ui.plot_type = curr_plot_type;
                        _reset_plots = true;
                    }
                    if (selected) { ImGui::SetItemDefaultFocus(); }
                }
                ImGui::EndCombo();
            }

            ImGui::Checkbox("Lock Plots", &_ui.lock_plots);
            ImGui::SameLine();
            if (ImGui::Checkbox("Sync Plots", &_ui.sync_plots)) { _reset_plots = true; }
            ImGui::SameLine();
            if (ImGui::Button("Reset Plots")) { _reset_plots = true; }

            ImGui::Checkbox("Auto-size Plots", &_ui.autosize_plots);
            if (!_ui.autosize_plots) {
                ImGui::SliderFloat("Plots Size", &_ui.plot_size_px, 80.0f, 400.0f, "%.0f px");
            }
        }

        // Control section
        if (ImGui::CollapsingHeader("Control", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // ----- Rest Pose calibration options -----
            ImGui::SeparatorText("Rest Pose");
            {
                ImGui::TextUnformatted(pipe.estimator().has_rest_pose() ? "Rest Pose: calibrated" : "Rest Pose: N/A");
                ImGui::SameLine();
                if (ImGui::Button("Calibrate")) { pipe.calibrate_rest_pose(); } // the pipeline logs the outcome
                ImGui::SameLine();
                if (ImGui::Button("Clear")) { pipe.clear_rest_pose(); }
            }

            // ----- Rotation filter options -----
            ImGui::SeparatorText("Rotation Filter");
            {
                constexpr auto kFlags = ImGuiSliderFlags_AlwaysClamp;
                // Small double-DragScalar helper (params are double; avoids float temporaries).
                const auto drag = [](const char* label, double& v, double lo, double hi, double step, const char* fmt) {
                    return ImGui::DragScalar(label, ImGuiDataType_Double, &v, static_cast<float>(step), &lo, &hi, fmt, kFlags);
                };

                auto& opt = pipe.estimator().options();

                ImGui::Checkbox("Enable smoothing", &opt.enable_smoothing);

                // Kernel selector
                const char* const kernel_kinds[] = { "One Euro" };
                int curr_kind = static_cast<int>(opt.filter.kind);
                if (ImGui::Combo("Kernel", &curr_kind, kernel_kinds, IM_ARRAYSIZE(kernel_kinds))) {
                    opt.filter.kind = static_cast<pose::rotation_filter_kind>(curr_kind);
                }

                if (opt.filter.kind == pose::rotation_filter_kind::one_euro)
                {
                    ImGui::BeginDisabled(!opt.enable_smoothing);
                    auto& oe = opt.filter.one_euro;
                    drag("Min cutoff [Hz]", oe.min_cutoff_hz, 0.01, 10.0, 0.01, "%.2f");
                    drag("Beta", oe.beta, 0.0, 1.0, 0.001, "%.3f");
                    drag("Deriv cutoff [Hz]", oe.dcutoff_hz, 0.01, 10.0, 0.01, "%.2f");
                    ImGui::EndDisabled();
                }

                // Occlusion hold (independent of the smoothing on/off switch).
                double hold_ms = opt.max_hold.count();
                if (drag("Max hold [ms]", hold_ms, 0.0, 1000.0, 1.0, "%.0f")) { opt.max_hold = pose::millis_f64{ hold_ms }; }
                double reset_ms = opt.reset_gap.count();
                if (drag("Reset gap [ms]", reset_ms, 0.0, 2000.0, 1.0, "%.0f")) { opt.reset_gap = pose::millis_f64{ reset_ms }; }
            }

            // ----- Leg-hinge constraint options -----
            ImGui::SeparatorText("Hinge Constraint");
            {
                auto& opt = pipe.estimator().options();
                ImGui::Checkbox("Enable hinge constraint", &opt.enable_hinge_constraint);
                ImGui::SetItemTooltip("Reject the planar-ambiguity flip and constrain each joint to its\n"
                                      "1-DOF hinge axis. Needs a captured rest pose.");
            }
        }
    }

    // Axis-frame sync/reset via the internal ImPlot3DPlot (implot3d has no public links).
    // Called inside each BeginPlot/EndPlot, after SetupAxesLimits.
    void debugger_app::_sync_axis_frame()
    {
        ImPlot3DPlot* plot = ImPlot3D::GetCurrentPlot();
        if (!plot) { return; }

        // One-shot reset: return to the home rotation and default ranges.
        if (_reset_plots)
        {
            plot->Rotation = plot->InitialRotation;
            for (int a = 0; a < 3; ++a) { plot->Axes[a].SetRange(-1.2, 1.2); }
        }

        if (!_ui.sync_plots) { return; } // subplots independent

        // The hovered/held plot (unless locked) is the master; the first sync frame and any reset
        // (re)seed the shared reference from it. Everyone else follows the shared reference.
        const bool master = !_sync_init || _reset_plots
            || (!_ui.lock_plots && (plot->Hovered || plot->Held));
        if (master)
        {
            _sync_rot[0] = plot->Rotation.x; _sync_rot[1] = plot->Rotation.y;
            _sync_rot[2] = plot->Rotation.z; _sync_rot[3] = plot->Rotation.w;
            for (int a = 0; a < 3; ++a)
            {
                _sync_range[a][0] = plot->Axes[a].Range.Min;
                _sync_range[a][1] = plot->Axes[a].Range.Max;
            }
            _sync_init = true;
        }
        else
        {
            plot->Rotation = ImPlot3DQuat{ _sync_rot[0], _sync_rot[1], _sync_rot[2], _sync_rot[3] };
            for (int a = 0; a < 3; ++a) { plot->Axes[a].SetRange(_sync_range[a][0], _sync_range[a][1]); }
        }
    }

    void debugger_app::_render_plot_panel()
    {
        const int n = static_cast<int>(pose::kNumJoints);
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        const ImVec2 avail = ImGui::GetContentRegionAvail();

        int cols = 1;
        float cell_sz = 1.0f;
        if (_ui.autosize_plots)
        {
            // Pick the column count whose square cell best fills the panel area.
            for (int c = 1; c <= n; ++c)
            {
                const int r = (n + c - 1) / c;
                const float cw = (avail.x - spacing * (c - 1)) / c;
                const float ch = (avail.y - spacing * (r - 1)) / r;
                if (const float s = std::min(cw, ch); s > cell_sz) { cell_sz = s; cols = c; }
            }
        }
        else
        {
            cell_sz = _ui.plot_size_px * this->renderer().dpi_scale(); // DPI-aware px
            cols = std::max(1, static_cast<int>((avail.x + spacing) / (cell_sz + spacing)));
        }

        const ImVec2 plot_sz{ cell_sz, cell_sz };
        const char* order = kEulerOrders[_ui.euler_order].name;

        // Lines: x always auto-scrolls; Lock (or a one-shot reset) forces the default y range,
        // otherwise y is mouse-adjustable. Axis frame: rotation/range sync handled in _sync_axis_frame().
        const ImPlotCond y_cond = (_ui.lock_plots || _reset_plots) ? ImPlotCond_Always : ImPlotCond_Once;

        const pose::exo_pose_estimator& est = _server->pipeline().estimator();
        int col = 0;
        int ji = 0;
        for (const auto& info : pose::kJointsInfo)
        {
            const auto& st = est.get_joint_state(info.id);
            const char* ref = (!_ui.relative_rot || pose::is_root_joint(info.id))
                ? "camera" : pose::joint_info(info.parent).name.data();
            const auto rot = try_get_joint_rot(st, _ui.relative_rot);
            const std::optional<Eigen::Vector3d> e = rot.has_value()
                ? std::optional{ to_euler_deg(rot.value(), kEulerOrders[_ui.euler_order]) }
                : std::nullopt;

            // Second title line: euler for the euler-line plot, else the quaternion.
            std::string readout;
            if (_ui.plot_type == plot_type_t::euler_line)
            {
                readout = e.has_value()
                    ? std::format("Euler{}: {:.1f}, {:.1f}, {:.1f}", order, e->x(), e->y(), e->z())
                    : std::format("Euler{}: -", order);
            }
            else
            {
                readout = rot.has_value()
                    ? std::format("Q: {:.3f}, {:.3f}, {:.3f}, {:.3f}", rot->x(), rot->y(), rot->z(), rot->w())
                    : "Q: -";
            }
            // `###name` = stable id so the per-frame readout doesn't reset the plot's zoom/rotation.
            const std::string title = std::format("{} (ref: {})\n{}###{}", info.name, ref, readout, info.name);

            if (col != 0) { ImGui::SameLine(); }
            // Scope every id in this subplot (plot, legend, context menus) so implot3d/implot
            // items don't clash across subplots. Lets the plot items keep short, plain labels.
            ImGui::PushID(ji);
            ImGui::BeginGroup();

            if (_ui.plot_type == plot_type_t::axis_frame)
            {
                // Axis-frame view: RGB triad. Limits fixed; Lock/Sync/Reset act on the view rotation.
                ImPlot3DFlags f3d = ImPlot3DFlags_Equal | ImPlot3DFlags_NoClip;
                if (_ui.lock_plots) { f3d |= ImPlot3DFlags_NoRotate | ImPlot3DFlags_NoPan | ImPlot3DFlags_NoZoom; }
                if (ImPlot3D::BeginPlot(title.c_str(), plot_sz, f3d))
                {
                    ImPlot3D::SetupAxesLimits(-1.2, 1.2, -1.2, 1.2, -1.2, 1.2, ImPlot3DCond_Once);
                    ImPlot3D::SetupLegend(ImPlot3DLocation_West);
                    this->_sync_axis_frame(); // read-back rotation/range sync across subplots + reset
                    if (rot.has_value()) { draw_axes(rot.value(), 3.0f); }
                    ImPlot3D::EndPlot();
                }
            }
            else if (_ui.plot_type == plot_type_t::euler_line)
            {
                // Rolling history of the three euler angles (matches the triad colors).
                const ImVec4 col[3]{ { 1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 0, 0, 1, 1 } };
                const char* const nm[3]{ "X", "Y", "Z" };
                draw_lines(title.c_str(), _euler_bufs.view(ji), kWindowSec, kEulerYLo, kEulerYHi,
                    y_cond, _ui.sync_plots, _sync_y, col, nm, plot_sz);
            }
            else
            {
                // Rolling history of the quaternion components.
                const ImVec4 col[4]{ { 1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 0, 0, 1, 1 }, { 0.85f, 0.85f, 0.2f, 1 } };
                const char* const nm[4]{ "X", "Y", "Z", "W" };
                draw_lines(title.c_str(), _quat_bufs.view(ji), kWindowSec, kQuatYLo, kQuatYHi,
                    y_cond, _ui.sync_plots, _sync_y, col, nm, plot_sz);
            }

            ImGui::EndGroup();
            ImGui::PopID();

            if (++col >= cols) { col = 0; }
            ++ji;
        }

        _reset_plots = false; // one-shot: the ranges were forced this frame
    }

    float debugger_app::_log_split_height()
    {
        const float avail_y = ImGui::GetContentRegionAvail().y;
        const float max_log = std::max(kLogMinH, avail_y - kSplitHit - kLogMinH);
        _ui.log_h = std::clamp(_ui.log_h, kLogMinH, max_log);
        return avail_y - _ui.log_h - kSplitHit;
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
        if (ImGui::IsItemActive()) { _ui.log_h -= ImGui::GetIO().MouseDelta.y; }
        if (ImGui::IsItemHovered() || ImGui::IsItemActive()) { ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS); }

        ImGui::BeginChild("logpanel", ImVec2(0.0f, _ui.log_h), ImGuiChildFlags_Borders);
        ImGui::PopStyleVar(); // restore spacing for the console's own contents
        _log_console.draw();
        ImGui::EndChild();
    }

    void debugger_app::_do_start_recording()
    {
        std::filesystem::path path{ _ui.record_path };
        if (path.empty()) { return; }

        // The browser lets a name through without one, but the reader finds recordings by
        // extension and so does the user.
        if (path.extension() != ".mcap") { path.replace_extension(".mcap"); }

        const size_t index = std::clamp<size_t>(
            static_cast<size_t>(_ui.record_codec), 0, io::kImageCodecs.size() - 1);
        const io::recording_options options{
            .codec = io::kImageCodecs[index].codec,
            .encode = { .jpeg_quality = _ui.jpeg_quality },
        };

        if (_server->pipeline().start_recording(path, options))
        {
            _ui.record_path.clear(); // the next take gets a fresh timestamped name
            _ui.show_record = false;
        }
    }

    void debugger_app::_do_stop_recording()
    {
        _server->pipeline().stop_recording();
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
        if (!_ui.show_record) { return; }

        ImGui::SetNextWindowSize(ImVec2(460, 0), ImGuiCond_Appearing);
        if (ImGui::Begin("Start Recording", &_ui.show_record, ImGuiWindowFlags_NoCollapse))
        {
            if (ImGui::Button("Browse...")) { _save_dialog.Open(); }
            ImGui::SameLine();
            ImGui::TextUnformatted(_ui.record_path.empty() ? "(no file selected)" : _ui.record_path.c_str());

            ImGui::Combo("Codec", &_ui.record_codec, kCodecLabels.data(), static_cast<int>(kCodecLabels.size()));

            const bool is_jpeg = (io::kImageCodecs[static_cast<size_t>(_ui.record_codec)].codec
                == io::image_codec::jpeg);

            ImGui::BeginDisabled(!is_jpeg);
            ImGui::SliderInt("JPEG quality", &_ui.jpeg_quality, 1, 100);
            ImGui::EndDisabled();

            ImGui::TextWrapped("%s", is_jpeg
                ? "Lossy. Roughly 5-15 MB/s at 1080p30."
                : "Lossless pixels, compressed per chunk. Much larger; meant for short clips.");

            ImGui::Separator();
            ImGui::BeginDisabled(_ui.record_path.empty());
            if (ImGui::Button("Start", ImVec2(90, 0))) { this->_do_start_recording(); }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(90, 0))) { _ui.show_record = false; }
        }
        ImGui::End();
    }

    void debugger_app::_render_open_dialog()
    {
        if (!_ui.show_open) { return; }
        ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_Appearing);
        if (ImGui::Begin("Open Source", &_ui.show_open, ImGuiWindowFlags_NoCollapse))
        {
            const auto kind_radio = [this](const char* label, source_kind_t val) {
                if (ImGui::RadioButton(label, _ui.open_kind == val)) { _ui.open_kind = val; }
            };
            kind_radio("Device", source_kind_t::device);
            ImGui::SameLine();
            kind_radio("Recording", source_kind_t::recording);
            ImGui::Separator();

            if (_ui.open_kind == source_kind_t::device)
            {
                ImGui::InputInt("Device index", &_ui.device);
                if (_ui.device < 0) { _ui.device = 0; }
                ImGui::Checkbox("Manual exposure [us]", &_ui.manual_exposure);
                if (_ui.manual_exposure)
                {
                    ImGui::SameLine(); ImGui::SetNextItemWidth(120);
                    ImGui::InputInt("##exposure", &_ui.exposure);
                }
                ImGui::Checkbox("Manual gain", &_ui.manual_gain);
                if (_ui.manual_gain)
                {
                    ImGui::SameLine(); ImGui::SetNextItemWidth(120);
                    ImGui::InputInt("##gain", &_ui.gain);
                }
            }
            else
            {
                if (ImGui::Button("Browse...")) { _file_dialog.Open(); }
                ImGui::SameLine();
                ImGui::TextUnformatted(_ui.recording.empty() ? "(no file selected)" : _ui.recording.c_str());
            }

            ImGui::Separator();
            if (ImGui::Button("Open", ImVec2(90, 0))) { this->_do_open_source(); }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(90, 0))) { _ui.show_open = false; }
        }
        ImGui::End();
    }

} // namespace gui
