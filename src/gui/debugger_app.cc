#include "debugger_app.hh"

#include "net/exo_pose_server.hh"
#include "net/exo_pose_pipeline.hh"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <format>
#include <string>
#include <vector>

namespace gui
{
    namespace
    {
        // Codec picker entries, one per `io::kImageCodecs` row and in the same order.
        constexpr std::array<const char*, io::kImageCodecs.size()> kCodecLabels{
            "JPEG (compressed)",
            "Raw (lossless)",
        };

        // Local wall clock string for filename
        std::string local_stamp()
        {
            const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
            try {
                const std::chrono::zoned_time local{ std::chrono::current_zone(), now };
                return std::format("{:%y%m%d%H%M%S}", local);
            }
            catch (const std::exception&) {
                return std::format("{:%y%m%d%H%M%S}", now); // no time zone database; UTC instead
            }
        }

        std::string default_recording_name(
            const hw::source_backend_t backend,
            const pose::view_plane_t view_plane)
        {
            return std::format("capture-{}-{}-{}.mcap"
                , hw::source_backend_to_str(backend)
                , pose::view_plane_name(view_plane)
                , local_stamp()
            );
        }

        std::string default_trace_name()
        {
            return std::format("pose_trace_{}.json", local_stamp());
        }

        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

        // Small double-DragScalar helper (estimator options are double; avoids float temporaries).
        bool option_drag(const char* label, double& v, double lo, double hi, double step, const char* fmt)
        {
            return ImGui::DragScalar(label, ImGuiDataType_Double, &v, static_cast<float>(step),
                &lo, &hi, fmt, ImGuiSliderFlags_AlwaysClamp);
        }

        // Splitter grip thickness [px]. It doubles as the inter-panel gap: surrounding
        // ItemSpacing is zeroed so the visible border-to-border gap equals this on both
        // axes, and the whole gap is the drag hit-target (same width for v/h splitters).
        constexpr float kSplitHit = 6.0f;
        constexpr float kLogMinH = 60.0f;  // min height for both the content and log panes [px]
        constexpr float kPlotMinW = 200.0f; // min width for the plots pane [px]
        constexpr float kSideMinW = 200.0f; // min width for the control pane [px]

    } // namespace

    debugger_app::debugger_app(const app::app_config_t& config)
        : _server{ std::make_unique<net::exo_pose_server>(config, /*annotate_frames*/true) }
    {
        _open_dialog.fill(_server->config());
        _ui.show_log = true; // surface the log console by default

        _recording_save_browser.SetTitle("Save recording as");
        _recording_save_browser.SetTypeFilters({ ".mcap" });

        _config_save_browser.SetTitle("Save config as");
        _config_save_browser.SetTypeFilters({ ".json" });

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
                this->_handle_color_sample_click(ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
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
            _plot_panel.render(_server->pipeline().estimator(), this->renderer().dpi_scale());
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

        // The form edits the config in place, so an Open streams exactly what it shows.
        const open_source_dialog::result_t req = _open_dialog.render(_server->config());
        if (req.load_config) { this->_do_load_config(*req.load_config); }
        if (req.open_source) { this->_open_source(); }

        this->_render_record_dialog();

        _recording_save_browser.Display();
        if (_recording_save_browser.HasSelected())
        {
            _ui.record_dlg_path = _recording_save_browser.GetSelected().string();
            _recording_save_browser.ClearSelected();
        }

        _config_save_browser.Display();
        if (_config_save_browser.HasSelected())
        {
            this->_do_save_config(_config_save_browser.GetSelected());
            _config_save_browser.ClearSelected();
        }
    }

    void debugger_app::_open_source()
    {
        app::app_config_t& config = _server->config();

        // Samples taken against the previous source describe a camera that is no longer open.
        _color_sampler.clear();

        // A recording already carries the settings it was shot with; only a live camera takes them.
        if (config.camera.source.has_value() && config.camera.source->is_recording()) {
            config.camera.exposure_us.reset();
            config.camera.gain.reset();
        }

        _server->pipeline().open_source(config);

        // The slider that will decide the next fit starts on the value already in force, so what
        // the panel shows and what the running detector accepts do not disagree until asked to.
        if (const auto* color_tracker = dynamic_cast<const pose::color_marker_tracker*>(
                _server->pipeline().tracker()))
        {
            _ui.color_max_distance = color_tracker->detector_options().model.max_distance;
        }

        _last_seq = 0;
        _plot_panel.reset(); // the new source does not continue the plotted samples
    }

    void debugger_app::_do_load_config(const std::filesystem::path& path)
    {
        app::app_config_t loaded;
        if (std::string err; !app::load_config(path, loaded, err)) {
            spdlog::error("config: {}", err);
            return;
        }

        _server->config() = std::move(loaded);
        _open_dialog.fill(_server->config());
        spdlog::info("config: loaded '{}'", path.string());

        // An open source keeps running the settings it was opened with, so a config profile
        // arriving over them is installed straight away. Left for the next Open, the running
        // tracker would hold the colour and filters of the config profile before it while the file
        // says otherwise, and a save would gather those into this one.
        if (!_server->pipeline().is_source_open()) {
            spdlog::info("config: open a source to run with it");
            return;
        }

        if (!_server->config().camera.source.has_value()) {
            spdlog::info("config: this file names no source, so the open one is closed");
            this->_do_close_source();
            return;
        }

        this->_open_source();
    }

    void debugger_app::_do_save_config(const std::filesystem::path& path)
    {
        // A save gathers the config from two places. `server` stands as loaded, and the open
        // dialog already wrote `camera` and the viewing plane into it. The tuning below did not go
        // there: the control panel edits it on the pipeline, so the live values are read back.
        //
        // Reading back is right only while the pipeline is never older than the config, which
        // holds because every other write to the config opens a source with it straight away.
        net::exo_pose_pipeline& pipe = _server->pipeline();
        app::app_config_t& config = _server->config();

        // Only the running tracker has live values to read back; the other kind's block keeps
        // whatever the config profile was loaded with.
        if (auto* tag_tracker = dynamic_cast<pose::apriltag_tracker*>(pipe.tracker())) {
            config.pose.detector.apriltag = tag_tracker->options();
            config.pose.tag_size_m = tag_tracker->tag_size_m();
        }
        if (auto* color_tracker = dynamic_cast<pose::color_marker_tracker*>(pipe.tracker()))
        {
            config.pose.detector.color_marker.assigner = color_tracker->assigner_options();

            // The colour and the blob filters are read back the same way, and the frame size they
            // were measured on comes from the source that is delivering it. A colour that was never
            // fitted leaves the block absent, which is what an unmeasured installation writes.
            if (const pose::color_marker_detector::options_t detector = color_tracker->detector_options();
                detector.model.valid)
            {
                config.pose.detector.color_marker.calibration = app::color_marker_calibration_t{
                    .detector = detector,
                    .frame_resolution = pipe.source_resolution(),
                };
            }
        }
        if (const auto o = pipe.frontal_options())  { config.pose.frontal = *o; }
        if (const auto o = pipe.sagittal_options()) { config.pose.sagittal = *o; }

        std::string err;
        if (!app::save_config(config, path, err)) {
            spdlog::error("config: {}", err);
            return;
        }

        spdlog::info("config: saved to '{}'", path.string());
    }

    void debugger_app::_do_close_source()
    {
        _server->pipeline().close_source();
        _last_seq = 0;
        _plot_panel.reset();
    }

    void debugger_app::_update_pose_frame()
    {
        // Pull the server's latest annotated frame. The server owns and updates the estimator;
        // nothing to do until a new frame arrives.
        net::exo_pose_pipeline& pipe = _server->pipeline();

        // The classifier's per-pixel images are copied on the frame thread, so the tracker is told
        // each step whether anything is looking at them.
        auto* color_tracker = dynamic_cast<pose::color_marker_tracker*>(pipe.tracker());
        if (color_tracker != nullptr) { color_tracker->set_publish_debug_images(_ui.color_backdrop != 0); }

        if (!pipe.try_get_annotated_frame(_last_frame, _last_source_frame, _last_seq)) { return; }

        // What the camera view shows. The classifier's own two images answer "is this colour being
        // accepted, and how surely", which is what sampling is judged against; the drawn frame
        // answers "is the marker being found", which is what everything else wants.
        cv::Mat view = _last_frame;
        if (color_tracker != nullptr && _ui.color_backdrop != 0)
        {
            const cv::Mat decisions = (_ui.color_backdrop == 1) ? color_tracker->mask()
                                                                : color_tracker->score_image();
            if (!decisions.empty()) { cv::cvtColor(decisions, view, cv::COLOR_GRAY2BGR); }
        }
        _frame_texture.value().update(view);

        // Everything below places joint state on a timeline, so it is timed by the frame the
        // estimator stepped on rather than the one just drawn. A run whose markers are never
        // detected keeps showing that image, which is what an operator needs to see to fix it,
        // while there is still nothing to plot.
        const pose::pose_estimator_base* est = pipe.estimator();
        if (!est || !pipe.has_pose()) { return; }

        // The plot buffers rebase against their first sample, so an absolute value is fine here.
        const hw::timestamp_t ts = pipe.last_timestamp();
        const double t_now = std::chrono::duration<double>{ ts.time_since_epoch() }.count();

        // Capture the full per-frame trace into the rolling ring so a glitch can be dumped with its
        // lead-up right after it is seen on screen.
        // The gates are estimator specific, so they are read off whichever options exist.
        if (_ui.trace_enabled)
        {
            trace_gates_t gates;
            if (const auto o = pipe.frontal_options()) {
                gates.max_hold_ms = o->max_hold.count();
                gates.reset_gap_ms = o->reset_gap.count();
                if (o->enable_hinge_constraint) { gates.hinge_axis = o->hinge_axis_world; }
            }
            else if (const auto o = pipe.sagittal_options()) {
                gates.max_hold_ms = o->max_hold.count();
                gates.reset_gap_ms = o->reset_gap.count();
            }
            // The trace records tag geometry, which only the tag tracker has; a colour run leaves
            // that section empty and keeps the joint state the rest of the trace is about.
            std::vector<pose::tag_detection_t> tags;
            if (auto* tag_tracker = dynamic_cast<pose::apriltag_tracker*>(pipe.tracker())) {
                tags = tag_tracker->last_detections();
            }
            _trace.capture(ts, tags, *est, gates);
        }

        _plot_panel.push(*est, t_now);
    }

    void debugger_app::_render_menu_bar()
    {
        if (!ImGui::BeginMainMenuBar()) { return; }
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open..."))
            {
                _open_dialog.fill(_server->config());
                _open_dialog.show();
            }
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
                if (_ui.record_dlg_path.empty()) {
                    _ui.record_dlg_path = (
                        app::project_dir("recordings") / default_recording_name(pipe.source_backend(), pipe.view_plane())
                    ).string();
                }
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
                    this->_handle_color_sample_click(ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
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
                    ImGui::SameLine();
                    if (bool loop = pipe.is_auto_repeat_enabled(); ImGui::Checkbox("Loop", &loop)) {
                        pipe.set_auto_repeat(loop);
                    }
                    ImGui::SetItemTooltip("Off: playback stops at the end, and |< starts it again.\n"
                                          "Each restart drops the tracking that described the last "
                                          "position, so a colour tuned frame by frame is steadier "
                                          "without it.");
                }

                this->_render_recording_status();
            }
            else
            {
                ImGui::TextUnformatted("No source opened. (File > Open...)");
            }
        }

        // Session section: what an operator works with during a run. None of it is written to a
        // config profile, which is why it stands apart from the tuning below.
        if (ImGui::CollapsingHeader("Session", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::SeparatorText("Rest Pose");
            {
                ImGui::TextUnformatted(pipe.has_rest_pose() ? "Rest Pose: calibrated" : "Rest Pose: N/A");
                ImGui::SameLine();
                if (ImGui::Button("Calibrate")) { pipe.calibrate_rest_pose(); } // the pipeline logs the outcome
                ImGui::SameLine();
                if (ImGui::Button("Clear")) { pipe.clear_rest_pose(); }
            }

            // Rolling ring of full per-frame traces. See a glitch on screen, hit Dump, and the
            // recent history lands in dumps/*.json for analysis.
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

        // Tuning section: everything a config profile carries, ending in the save that writes it.
        if (ImGui::CollapsingHeader("Tuning", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // ----- Tag detection tuning (live; the worker rebuilds the detector on change) -----
            // Shown only while the tag tracker is the one running, since these knobs describe it.
            if (auto* tag_tracker = dynamic_cast<pose::apriltag_tracker*>(pipe.tracker()))
            {
                ImGui::SeparatorText("Tag Detection");

                double tag_size_m = tag_tracker->tag_size_m();
                const double tag_min = 0.005, tag_max = 1.0;
                if (ImGui::DragScalar("Tag size [m]", ImGuiDataType_Double, &tag_size_m,
                        0.001f, &tag_min, &tag_max, "%.3f", ImGuiSliderFlags_AlwaysClamp))
                {
                    tag_tracker->set_tag_size_m(tag_size_m);
                }
                ImGui::SetItemTooltip("Real black-square edge length of the printed tag [m].\n"
                                      "Fixes the metric scale of every estimated 3D position; must match the tag.\n"
                                      "Higher: estimated depth and the whole skeleton scale up.\n"
                                      "Lower: they scale down.");

                // Edit a copy of the current options, push it back only when something changed.
                pose::tag_detector::options_t opt = tag_tracker->options();
                bool changed = false;

                // A sagittal run works off 2D tag centers, so the detector never solves a tag pose
                // and these two knobs have nothing to act on.
                const bool solves_tag_pose = (pipe.view_plane() == pose::view_plane_t::frontal);
                ImGui::BeginDisabled(!solves_tag_pose);
                const char* const methods[] = { "Orthogonal iteration", "Homography (closed form)" };
                int mi = (opt.pose_method == pose::tag_detector::pose_method_t::homography) ? 1 : 0;
                if (ImGui::Combo("Pose method", &mi, methods, IM_ARRAYSIZE(methods))) {
                    opt.pose_method = (mi == 1) ? pose::tag_detector::pose_method_t::homography
                                              : pose::tag_detector::pose_method_t::orthogonal_iteration;
                    changed = true;
                }
                ImGui::SetItemTooltip("How tag->camera pose (hence the 3D position) is solved.\n"
                                      "OI: iterative; most accurate rotation, two candidates, costlier.\n"
                                      "Homography: closed-form; cheaper, translation/depth comparable.");
                ImGui::EndDisabled();

                changed |= ImGui::SliderFloat("quad_decimate", &opt.quad_decimate, 1.0f, 4.0f, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                ImGui::SetItemTooltip("Image downsample factor before quad detection (1.0 = full res).\n"
                                      "The biggest detection CPU knob.\n"
                                      "Higher: much faster, but coarser corners (worse pose/depth) and\n"
                                      "        misses small/distant tags.\n"
                                      "Lower: slower, best corner accuracy.");

                changed |= ImGui::SliderFloat("quad_sigma", &opt.quad_sigma, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
                ImGui::SetItemTooltip("Gaussian blur applied before detection (0 = none).\n"
                                      "Higher: smooths sensor noise (helps low-res/noisy), but erases small tags.\n"
                                      "Lower: sharper corners, no denoising.");

                changed |= ImGui::Checkbox("refine_edges", &opt.refine_edges);
                ImGui::SetItemTooltip("Snap quad edges to image gradients for sub-pixel corners.\n"
                                      "On: better pose/depth accuracy, small extra cost.\n"
                                      "Off: faster, coarser corners (fine when decimating hard).");

                ImGui::BeginDisabled(!solves_tag_pose
                    || opt.pose_method != pose::tag_detector::pose_method_t::orthogonal_iteration);
                changed |= ImGui::SliderInt("num_iters", &opt.num_iters, 1, 100, "%d");
                ImGui::SetItemTooltip("Orthogonal-iteration steps for pose refinement (OI only).\n"
                                      "Higher: more accurate rotation, diminishing returns past ~50.\n"
                                      "Lower: faster, coarser pose.");
                ImGui::EndDisabled();

                changed |= ImGui::SliderInt("num_threads", &opt.num_threads, 1, 16, "%d");
                ImGui::SetItemTooltip("Detector worker threads (no effect on accuracy).\n"
                                      "Higher: faster detection on multi-core CPUs.\n"
                                      "Lower: fewer cores used.");

                if (changed) {
                    tag_tracker->set_options(opt); // worker rebuilds the detector next frame
                }

                ImGui::TextDisabled("Applies live to an open source; rebuilds the detector.");
            }

            // ----- Color marker tuning, shown while that tracker is the one running -----
            if (auto* color_tracker = dynamic_cast<pose::color_marker_tracker*>(pipe.tracker()))
            {
                this->_render_color_marker_control(*color_tracker);
            }

            // ----- Estimator tuning (the two estimators expose different knobs) -----
            // Handed back every frame: assigning options is cheap, so no change detection.
            if (auto frontal = pipe.frontal_options()) {
                this->_render_frontal_estimator_control(*frontal);
                pipe.set_frontal_options(*frontal);
            }
            else if (auto sagittal = pipe.sagittal_options()) {
                this->_render_sagittal_estimator_control(*sagittal);
                pipe.set_sagittal_options(*sagittal);
            }

            // The panel renders only with a source open, so what is written is the tuning that is
            // actually running.
            ImGui::SeparatorText("Config");
            {
                if (ImGui::Button("Save Config..."))
                {
                    _config_save_browser.SetPwd(app::project_dir("configs"));
                    _config_save_browser.SetInputName("new_config.json");
                    _config_save_browser.Open();
                }
                ImGui::SetItemTooltip("Write a config a headless run can be started from: the open source\n"
                                      "and its camera settings, the viewing plane and marker kind, and the\n"
                                      "tuning in this section. Nothing under Session is written.");
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

    // Clicking, or dragging across, a marker feeds the pixels under the cursor to the sampler. The
    // view may be scaled to fit, so screen coordinates are unscaled back to image pixels. Sampled
    // from the undrawn frame: the overlay sits on the markers and would contribute its own pixels.
    void debugger_app::_handle_color_sample_click(const ImVec2& img_min, const ImVec2& img_max)
    {
        if (!_ui.color_sampling || _last_source_frame.empty()) { return; }
        if (!ImGui::IsItemHovered() || !ImGui::IsMouseDown(ImGuiMouseButton_Left)) { return; }

        const float span_x = img_max.x - img_min.x;
        const float span_y = img_max.y - img_min.y;
        if (!(span_x > 0.0f) || !(span_y > 0.0f)) { return; }

        const ImVec2 m = ImGui::GetIO().MousePos;
        const int px = static_cast<int>((m.x - img_min.x) / span_x * static_cast<float>(_last_source_frame.cols));
        const int py = static_cast<int>((m.y - img_min.y) / span_y * static_cast<float>(_last_source_frame.rows));
        if (px < 0 || py < 0 || px >= _last_source_frame.cols || py >= _last_source_frame.rows) { return; }

        _color_sampler.add(_last_source_frame, cv::Point{ px, py }, _ui.color_sample_radius);

        // Mark where the sample was taken, so a drag leaves a visible trail.
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const float r = static_cast<float>(_ui.color_sample_radius) * span_x
                      / static_cast<float>(_last_source_frame.cols);
        dl->AddCircle(m, std::max(3.0f, r), IM_COL32(255, 255, 0, 220), 0, 2.0f);
    }

    void debugger_app::_do_fit_color_model(pose::color_marker_tracker& tracker)
    {
        const std::optional<pose::color_model_t> model = _color_sampler.fit(_ui.color_max_distance);
        if (!model.has_value()) {
            spdlog::warn("color: {} sample(s) is not enough to estimate a 2D covariance ({} needed)",
                _color_sampler.count(), pose::color_sampler::kMinSamples);
            return;
        }

        // Installed on the running tracker, so the very next frame is classified by what was just
        // measured and the operator judges the fit on the thing itself.
        pose::color_marker_detector::options_t opt = tracker.detector_options();
        opt.model = *model;
        tracker.set_detector_options(opt);

        spdlog::info("color: model fitted from {} samples, mean a*{:+.1f} b*{:+.1f}, sd {:.1f}/{:.1f}",
            _color_sampler.count(), model->mean_ab.x(), model->mean_ab.y(),
            std::sqrt(model->cov_ab(0, 0)), std::sqrt(model->cov_ab(1, 1)));
    }

    // Measuring this installation's colour: sample the marker, fit, save. It is done here because
    // this window has the camera open with the config profile that will run it, so the conditions
    // the file records are the conditions production uses.
    void debugger_app::_render_color_model_section(pose::color_marker_tracker& tracker)
    {
        ImGui::SeparatorText("Color Model");

        const double kColorDistMin = 0.5, kColorDistMax = 8.0;

        const pose::color_model_t model = tracker.detector_options().model;
        if (model.valid) {
            ImGui::Text("model       a*%+.1f b*%+.1f  sd %.1f/%.1f  d<%.1f",
                model.mean_ab.x(), model.mean_ab.y(),
                std::sqrt(model.cov_ab(0, 0)), std::sqrt(model.cov_ab(1, 1)), model.max_distance);
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "%s",
                "model       none: sample a marker and fit, or nothing is detected");
        }
        ImGui::SetItemTooltip(
            "The colour this installation's markers photograph as, as a mean and a spread\n"
            "on the a*b* plane. It is measured here and kept in the config, under\n"
            "pose.detector.color_marker.calibration.");

        // ----- What the camera view shows while sampling -----
        const char* const backdrops[] = { "camera", "mask", "membership score" };
        ImGui::Combo("Backdrop", &_ui.color_backdrop, backdrops, IM_ARRAYSIZE(backdrops));
        ImGui::SetItemTooltip(
            "camera            the frame with the detections drawn on it.\n"
            "mask              white where the classifier accepted the pixel.\n"
            "membership score  bright where it is sure, dark where it barely passed.\n"
            "\n"
            "The last two say why a marker is or is not being found, and cost a frame's\n"
            "worth of pixels copied per frame, so they are only produced while shown.");

        // ----- Collecting samples -----
        ImGui::Checkbox("Sample on click", &_ui.color_sampling);
        ImGui::SetItemTooltip(
            "Dragging over the camera view collects the pixels under the cursor.\n"
            "\n"
            "Sample the marker across the whole swing, not in one pose: the spread this\n"
            "fits has to cover the light the marker actually moves through, shadowed side\n"
            "included. Fullscreen (F11) makes the marker easier to hit.");

        ImGui::SliderInt("Sample radius", &_ui.color_sample_radius, 2, 40, "%d px");
        ImGui::SetItemTooltip("Radius of the disc each click collects.\n"
                              "Keep it inside the marker: one pixel of background widens the\n"
                              "spread far more than a hundred good ones narrow it.");

        const bool enough = _color_sampler.count() >= pose::color_sampler::kMinSamples;
        ImGui::Text("samples     %zu  (need %zu)", _color_sampler.count(), pose::color_sampler::kMinSamples);
        ImGui::SetItemTooltip("Pixels collected so far. The minimum is what a 2D covariance needs\n"
                              "to exist at all; a model worth keeping wants thousands.");

        ImGui::SliderScalar("Max distance", ImGuiDataType_Double, &_ui.color_max_distance,
                            &kColorDistMin, &kColorDistMax, "%.2f");
        ImGui::SetItemTooltip(
            "How many standard deviations from the fitted mean still count as the marker.\n"
            "Normally distributed, 2.0 covers about 86%% of the samples and 3.0 about 99%%.\n"
            "\n"
            "Applied by the next Fit.\n"
            "Higher: survives shadow, admits background of a similar hue.\n"
            "Lower: only the colour it was sampled at.");

        ImGui::BeginDisabled(!enough);
        if (ImGui::Button("Fit model")) { this->_do_fit_color_model(tracker); }
        ImGui::EndDisabled();
        ImGui::SetItemTooltip("Computes the mean and covariance of the collected samples and\n"
                              "installs them on the running detector, so the next frame is\n"
                              "classified by them. Disabled until enough samples exist.");

        ImGui::SameLine();
        // Only the samples go. The installed model stays in force, so detection keeps running.
        if (ImGui::Button("Clear samples")) { _color_sampler.clear(); }
        ImGui::SetItemTooltip("Discards the collected samples. The installed model stays until the\n"
                              "next Fit, so detection keeps running while you resample.");
    }

    void debugger_app::_render_color_marker_control(pose::color_marker_tracker& tracker)
    {
        this->_render_color_model_section(tracker);

        // ----- Readout: has the leg been identified, and what did the last frame find -----
        // These two together localize a stall. Nothing detected is a colour, exposure or lighting
        // problem; detected but not identified is an ordering, radius or geometry problem.
        ImGui::SeparatorText("Color Markers");

        const pose::color_marker_assigner::stats_t& assignment = tracker.assigner_stats();
        const std::size_t blobs_found = tracker.last_detection_count();

        if (assignment.locked) {
            ImGui::Text("assignment  locked  (%d/%d joint(s) this frame)",
                assignment.assigned, assignment.candidates);
        } else {
            ImGui::TextDisabled("assignment  searching  (%zu blob(s) found)", blobs_found);
        }
        ImGui::SetItemTooltip(
            "Whether the assigner is following markers it has already identified.\n"
            "\n"
            "Searching with 0 blobs  -> the detector finds nothing: check the colour model,\n"
            "                           the exposure and the lighting.\n"
            "Searching with blobs    -> they are found but not named: check the vertical\n"
            "                           order, the search radius and the geometry check.\n"
            "Locked                  -> normal.");

        ImGui::Text("dropped     radius %d, geometry %s, reference %s",
            assignment.out_of_radius,
            assignment.bad_geometry ? "rejected this frame" : "ok",
            assignment.has_reference ? "captured" : "not captured");
        ImGui::SetItemTooltip(
            "radius     slots with no candidate close enough to their last position.\n"
            "geometry   the whole frame was thrown out because a bone length left its band.\n"
            "reference  whether the rest-pose capture latched one. Without it the geometry\n"
            "           check stays inactive, however it is configured.");

        const pose::marker_reject_stats_t rejects = tracker.reject_stats();
        ImGui::Text("rejected    %d  (small %d, large %d, unfilled %d, elongated %d, faint %d)",
            rejects.total(), rejects.too_small, rejects.too_large,
            rejects.not_filled, rejects.too_long, rejects.low_score);
        ImGui::SetItemTooltip(
            "Candidate blobs the detector's filters dropped, and which filter dropped them.\n"
            "\n"
            "  small     -> lower min area, or move closer / use a bigger marker\n"
            "  large     -> lower max area, or a background region matches the colour\n"
            "  unfilled  -> raise the close kernel, or lower min fill\n"
            "  elongated -> shorten the exposure, or raise max aspect\n"
            "  faint     -> lower min score, or resample the model under this light");

        // ----- Blob filters (live; the worker rebuilds the detector on change) -----
        pose::color_marker_detector::options_t detector = tracker.detector_options();
        bool detector_changed = false;

        const double kAreaMin = 1.0, kAreaMax = 200000.0, kZero = 0.0, kOne = 1.0;
        const double kAspectMin = 1.0, kAspectMax = 8.0;

        detector_changed |= ImGui::SliderScalar("min area [px]", ImGuiDataType_Double, &detector.min_area_px,
                                                &kAreaMin, &kAreaMax, "%.0f", ImGuiSliderFlags_Logarithmic);
        ImGui::SetItemTooltip("Smallest blob accepted. A 20 px disc is about 314 px2.\n"
                              "Higher: speckle is filtered out.\n"
                              "Lower: distant or partly hidden markers survive.");

        detector_changed |= ImGui::SliderScalar("max area [px]", ImGuiDataType_Double, &detector.max_area_px,
                                                &kAreaMin, &kAreaMax, "%.0f", ImGuiSliderFlags_Logarithmic);
        ImGui::SetItemTooltip("Largest blob accepted. Blocks a wall or a garment whose colour matches.");

        detector_changed |= ImGui::SliderScalar("min fill", ImGuiDataType_Double, &detector.min_fill,
                                                &kZero, &kOne, "%.2f");
        ImGui::SetItemTooltip("Blob area over its bounding box. A solid disc is about 0.785.\n"
                              "A glossy marker falling below this wants a bigger close kernel\n"
                              "before it wants a lower threshold: the hole is the real problem.");

        detector_changed |= ImGui::SliderScalar("max aspect", ImGuiDataType_Double, &detector.max_aspect,
                                                &kAspectMin, &kAspectMax, "%.2f");
        ImGui::SetItemTooltip("Bounding box long side over short side. A circle is 1.0.\n"
                              "A long exposure smears a swinging marker into a streak.");

        detector_changed |= ImGui::SliderScalar("min score", ImGuiDataType_Double, &detector.min_score,
                                                &kZero, &kOne, "%.2f");
        ImGui::SetItemTooltip("Mean membership over the blob, 0 to 1. Size and shape can match by\n"
                              "accident; this is the check on the colour itself.");

        detector_changed |= ImGui::SliderInt("open kernel [px]", &detector.open_kernel_px, 0, 15, "%d");
        ImGui::SetItemTooltip("Erode then dilate: removes specks outside the blob. 0 skips it.");

        detector_changed |= ImGui::SliderInt("close kernel [px]", &detector.close_kernel_px, 0, 15, "%d");
        ImGui::SetItemTooltip("Dilate then erode: fills holes inside the blob, which is what a\n"
                              "specular highlight punches through a glossy marker. 0 skips it.");

        if (detector_changed) { tracker.set_detector_options(detector); }

        // ----- Joint assignment (live; handed back below, so it takes effect next frame) -----
        pose::color_marker_assigner::options_t assigner = tracker.assigner_options();

        const double kRadiusMin = 5.0, kRadiusMax = 400.0;
        const double kTolMin = 0.05, kTolMax = 0.95;
        const double kDiaMin = 0.001, kDiaMax = 0.2;

        // The chain of joint names is built from this, so a change drops the lock and the captured
        // spacing with it, and the next frames name the markers from their vertical order again.
        // `midline` is left out: the config refuses it for this field, so offering it here would
        // make a state that cannot be saved and reopened.
        const auto leg_radio = [&assigner](const char* label, pose::joint_side_t side) {
            if (ImGui::RadioButton(label, assigner.leg == side)) { assigner.leg = side; }
        };
        ImGui::TextUnformatted("Marked leg");
        ImGui::SameLine(); leg_radio("Left", pose::joint_side_t::left);
        ImGui::SameLine(); leg_radio("Right", pose::joint_side_t::right);
        ImGui::SetItemTooltip("Which leg carries the markers. A plain disc does not state one, so this does.\n"
                              "Changing it renames every slot: the assignment unlocks and the bone length\n"
                              "reference is dropped until the next rest-pose capture.");

        // The geometry check is the first thing to switch off when the assignment will not settle,
        // since it tells apart a broken rigid-body assumption from a broken order or radius.
        ImGui::Checkbox("Bone length check", &assigner.enable_bone_length_check);
        ImGui::SetItemTooltip(
            "Rejects a frame whose marker spacing left the band captured at rest pose.\n"
            "A one-slot slip shows up as roughly half or double, so it is caught here.\n"
            "\n"
            "Turn it off to tell a broken rigid-body assumption apart from an ordering\n"
            "or radius problem. A visibly tilted camera wants a wider band, not this off.");

        ImGui::BeginDisabled(!assigner.enable_bone_length_check);
        ImGui::SliderScalar("length tolerance", ImGuiDataType_Double, &assigner.bone_length_tolerance,
                            &kTolMin, &kTolMax, "%.2f");
        ImGui::SetItemTooltip("Allowed band around the captured spacing. 0.35 accepts 0.65x to 1.35x.\n"
                              "Raise it when the camera is not square to the sagittal plane: the leg's\n"
                              "own swing then shortens the spacing by a few per cent.");
        ImGui::EndDisabled();

        ImGui::SliderScalar("search radius [px]", ImGuiDataType_Double, &assigner.search_radius_px,
                            &kRadiusMin, &kRadiusMax, "%.0f");
        ImGui::SetItemTooltip("How far from its last position a marker may be found.\n"
                              "Must exceed what a marker travels in one frame.\n"
                              "Higher: keeps up with fast motion, admits background objects.");

        ImGui::SliderInt("lost frames", &assigner.lost_frames_before_full_search, 1, 120, "%d");
        ImGui::SetItemTooltip("Frames of incomplete assignment before the whole frame is searched\n"
                              "again from the vertical order.");

        ImGui::SliderScalar("marker diameter [m]", ImGuiDataType_Double, &assigner.marker_diameter_m,
                            &kDiaMin, &kDiaMax, "%.4f");
        ImGui::SetItemTooltip("Printed disc diameter. Sets the metric scale of the reported\n"
                              "positions only; the joint angles do not depend on it.");

        // Handed back every frame: six numbers, so no change detection.
        tracker.set_assigner_options(assigner);
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

            const auto def = knee.has_value()
                ? pose::get_joint_def(knee.value())
                : std::optional<pose::joint_definition_t>{};

            if (def.has_value()) {
                ImGui::TextUnformatted(std::format("Near leg: {} (tag {})", def->name, def->tag_id).c_str());
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
        const io::recording_options_t options{
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
        const std::filesystem::path dir = app::project_dir("dumps");
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

        const io::recording_stats_t stats = pipe.recording_stats();
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
            if (ImGui::Button("Browse...")) {
                // Open on the proposed name, so browsing only has to change what differs.
                const std::filesystem::path proposed{ _ui.record_dlg_path };
                if (proposed.has_parent_path()) { _recording_save_browser.SetPwd(proposed.parent_path()); }
                _recording_save_browser.SetInputName(proposed.filename().string());
                _recording_save_browser.Open();
            }
            ImGui::SameLine();
            ImGui::TextUnformatted(_ui.record_dlg_path.empty() ? "(no file selected)" : _ui.record_dlg_path.c_str());

            ImGui::Combo("Codec", &_ui.record_dlg_codec, kCodecLabels.data(), static_cast<int>(kCodecLabels.size()));

            const bool is_jpeg = (io::kImageCodecs[static_cast<size_t>(_ui.record_dlg_codec)].codec
                == io::image_codec_t::jpeg);

            ImGui::BeginDisabled(!is_jpeg);
            ImGui::SliderInt("JPEG quality", &_ui.record_dlg_jpeg_quality, 1, 100);
            ImGui::EndDisabled();

            ImGui::TextWrapped("%s", is_jpeg
                ? "Lossy. Roughly 5-15 MB/s at 1080p30."
                : "Lossless pixels, compressed per chunk. Much larger; meant for short clips.");

            // JPEG subsamples chroma, which is the very thing a colour model is measured on. A
            // recording meant for tuning it has to keep the colour the sensor saw.
            if (is_jpeg && dynamic_cast<const pose::color_marker_tracker*>(_server->pipeline().tracker()))
            {
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "%s",
                    "JPEG halves the colour resolution again on top of the sensor's Bayer\n"
                    "pattern. A colour model fitted live may not match this recording on\n"
                    "playback. Record raw when the clip is for tuning the colour.");
            }

            ImGui::Separator();
            ImGui::BeginDisabled(_ui.record_dlg_path.empty());
            if (ImGui::Button("Start", ImVec2(90, 0))) { this->_do_start_recording(); }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(90, 0))) { _ui.record_dlg_show = false; }
        }
        ImGui::End();
    }

} // namespace gui
