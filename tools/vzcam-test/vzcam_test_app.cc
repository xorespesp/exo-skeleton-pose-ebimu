// Debug harness for bringing the Vieworks VZ camera into the project.
//
// Exercises the pieces a sensor backend needs (discovery, open, pull-based acquisition,
// debayering, exposure/gain, ROI, GenICam node access) against a live camera.

#include "hw/backends/vz_device.hh"

#include "gui/app_base.hh"
#include "gui/app_renderer_sdl3.hh"
#include "gui/frame_texture.hh"
#include "gui/log_console.hh"
#include "pose/tag_detector.hh"

#include <imgui.h>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace
{
    // Nodes the harness offers as one-click reads; everything else goes through the explorer.
    constexpr const char* kIdentityNodes[] = {
        "DeviceVendorName", "DeviceModelName", "DeviceSerialNumber",
        "DeviceFirmwareVersion", "DeviceVersion", "DeviceUserID",
    };

    // Keeps the newest capture on hand for the render loop.
    //
    // `vz::device` hands frames out one blocking call at a time, so somebody has to wait on
    // that call, and a loop that draws every frame cannot be the one. This drains the stream
    // on its own thread and keeps only the latest, which is all a preview needs.
    class frame_latch_t final
    {
    public:
        ~frame_latch_t() { this->stop(); }

        void start(vz::device& device)
        {
            this->stop();
            _thread = std::jthread{ [this, &device](std::stop_token stop)
            {
                while (!stop.stop_requested())
                {
                    std::optional<vz::captured_frame_t> captured = device.grab_frame(kGrabTimeoutMs);
                    if (!captured.has_value()) {
                        // A timed-out grab has already waited out its share; a stream that is
                        // gone would otherwise spin here.
                        if (!device.is_streaming()) { break; }
                        continue;
                    }
                    std::scoped_lock lk{ _mtx };
                    _frame = std::move(captured->image);
                }
            } };
        }

        void stop()
        {
            _thread = {}; // requests the stop and joins

            std::scoped_lock lk{ _mtx };
            _frame.release();
        }

        // Empty until the first capture arrives.
        cv::Mat latest() const
        {
            std::scoped_lock lk{ _mtx };
            return _frame;
        }

    private:
        static constexpr uint32_t kGrabTimeoutMs = 500;

        mutable std::mutex _mtx;
        cv::Mat _frame;
        std::jthread _thread; // declared last: joined before the frame it writes goes away
    };

    class vzcam_test_app final : public gui::app_base<gui::app_renderer_sdl3>
    {
    public:
        int run()
        {
            _log.sink()->set_level(spdlog::level::trace);
            spdlog::default_logger()->sinks().push_back(_log.sink());

            if (!this->create("exo vz camera test harness", 1360, 860)) {
                spdlog::error("failed to create window");
                return -1;
            }
            spdlog::info("vzcam-test ready: press Enumerate to discover cameras");

            this->app_base::run();
            _grabber.stop();
            _dev.close();
            this->destroy();
            return 0;
        }

        void render_ui() override
        {
            if (!_tex.has_value()) { _tex.emplace(this->renderer().sdl_renderer()); }

            this->_update_detection();

            const ImGuiViewport* vp = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(vp->WorkPos);
            ImGui::SetNextWindowSize(vp->WorkSize);
            ImGui::Begin("##root", nullptr,
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
                ImGuiWindowFlags_MenuBar);

            this->_render_menu_bar();

            const float dpi = this->renderer().dpi_scale();
            const float side_w = 420.0f * dpi;
            const float log_h = _show_log ? 200.0f * dpi : 0.0f;
            const float content_h = ImGui::GetContentRegionAvail().y - log_h;

            ImGui::BeginChild("##preview", ImVec2{ ImGui::GetContentRegionAvail().x - side_w, content_h });
            this->_render_preview();
            ImGui::EndChild();

            ImGui::SameLine();

            ImGui::BeginChild("##side", ImVec2{ 0.0f, content_h });
            this->_render_controls();
            ImGui::EndChild();

            if (_show_log) {
                ImGui::Separator();
                ImGui::BeginChild("##log", ImVec2{ 0.0f, 0.0f });
                _log.draw();
                ImGui::EndChild();
            }

            ImGui::End();

            if (_show_dump) { this->_render_dump_window(); }
        }

    private:
        void _render_menu_bar()
        {
            if (!ImGui::BeginMenuBar()) { return; }
            if (ImGui::BeginMenu("View")) {
                ImGui::MenuItem("Log", nullptr, &_show_log);
                ImGui::MenuItem("Feature explorer", nullptr, &_show_dump);
                ImGui::Separator();
                ImGui::MenuItem("Fit preview", nullptr, &_fit_preview);
                ImGui::EndMenu();
            }
            const vz::stream_stats_t s = _dev.stats();
            ImGui::Text("   |   %s", _dev.is_open() ? "opened" : "closed");
            if (_dev.is_streaming()) {
                ImGui::Text("|  %ux%u %s -> %s  |  stream %.1f fps  demosaic %.2f ms  |  frames %llu  incomplete %llu  timeouts %llu",
                    s.width, s.height, s.pixel_format.c_str(),
                    _dev.frame_format() == vz::frame_format_t::gray ? "gray" : "bgr",
                    s.fps, s.convert_ms,
                    (unsigned long long)s.frames, (unsigned long long)s.incomplete,
                    (unsigned long long)s.timeouts);
            }
            if (_detect_enabled) {
                ImGui::Text("|  detect %.1f fps (%.2f ms)  tags %d",
                    _detect_fps, _detect_ms, static_cast<int>(_detections.size()));
            }
            ImGui::Text("|  ui %.1f fps", ImGui::GetIO().Framerate);
            ImGui::EndMenuBar();
        }

        // Detects on each new frame, keeping an annotated copy for display.
        // Runs on the UI thread, so the reported rate is what one consumer would reach.
        void _update_detection()
        {
            const cv::Mat frame = _grabber.latest();
            if (frame.empty()) {
                _display.release();
                return;
            }

            const uint64_t seq = _dev.stats().frames;

            if (!_detect_enabled) {
                _display = frame;
                _last_seq = seq;
                _detections.clear();
                return;
            }

            // Re-run only for a frame not seen yet, or right after a settings change.
            if (seq == _last_seq && !_redetect && !_display.empty()) { return; }
            _last_seq = seq;
            _redetect = false;

            if (!_detector.has_value()) { this->_rebuild_detector(); }

            using clock = std::chrono::steady_clock;
            const auto t0 = clock::now();
            _detections = _detector->detect(frame);
            const double ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count();

            // Overlay is drawn in colour: widen gray for display only
            cv::Mat annotated;
            if (frame.channels() == 1) { cv::cvtColor(frame, annotated, cv::COLOR_GRAY2BGR); }
            else { annotated = frame.clone(); }
            pose::draw_tag_detections(annotated, _detections);
            _display = std::move(annotated);

            _detect_ms = (_detect_ms <= 0.0) ? ms : (_detect_ms * 0.9 + ms * 0.1);
            const double dt = std::chrono::duration<double>(clock::now() - _last_detect_at).count();
            _last_detect_at = clock::now();
            if (dt > 0.0 && dt < 5.0) {
                const double inst = 1.0 / dt;
                _detect_fps = (_detect_fps <= 0.0) ? inst : (_detect_fps * 0.9 + inst * 0.1);
            }
        }

        void _apply_roi(const vz::roi_t& roi)
        {
            // ROI nodes are locked while streaming, so bracket the write with stop/resume.
            const bool was_streaming = _dev.is_streaming();
            if (was_streaming) {
                _grabber.stop();
                _dev.stop_stream();
            }

            const bool ok = _dev.set_roi(roi);
            _roi_edit_valid = false;

            if (const auto now = _dev.read_roi()) {
                spdlog::info("vz: ROI apply {} -> {}x{} @ {},{}",
                    ok ? "ok" : "partial", now->width, now->height, now->offset_x, now->offset_y);
            }
            if (was_streaming && _dev.start_stream()) {
                _grabber.start(_dev);
            }
        }

        void _rebuild_detector()
        {
            // No intrinsics -> 2D-only detection, all the overlay needs.
            pose::tag_detector::options_t opt{};
            opt.quad_decimate = _tune.quad_decimate;
            opt.quad_sigma = _tune.quad_sigma;
            opt.refine_edges = _tune.refine_edges;
            opt.num_threads = static_cast<size_t>(_tune.num_threads);
            _detector.emplace(opt);
            _detect_ms = 0.0;
            _detect_fps = 0.0;
            _redetect = true;
        }

        void _render_preview()
        {
            const cv::Mat frame = _display;
            if (frame.empty()) {
                ImGui::TextDisabled("no frame");
                return;
            }
            if (!_tex->update(frame)) {
                ImGui::TextDisabled("texture upload failed");
                return;
            }

            const ImVec2 avail = ImGui::GetContentRegionAvail();
            ImVec2 size{ static_cast<float>(_tex->width()), static_cast<float>(_tex->height()) };
            if (_fit_preview && size.x > 0.0f && size.y > 0.0f) {
                const float k = std::min(avail.x / size.x, avail.y / size.y);
                size = ImVec2{ size.x * k, size.y * k };
            }
            ImGui::Image(_tex->id(), size);

            if (_roi_preview && _roi_edit_valid) {
                const ImVec2 mn = ImGui::GetItemRectMin();
                const ImVec2 mx = ImGui::GetItemRectMax();
                this->_handle_roi_interaction(mn, mx);
                this->_draw_roi_overlay(mn, mx);
            }
        }

        // Drag the pending ROI on the image: interior moves, corner grips resize.
        // Screen deltas are unscaled back to sensor pixels.
        void _handle_roi_interaction(const ImVec2& img_min, const ImVec2& img_max)
        {
            if (_cur_roi.width <= 0 || _cur_roi.height <= 0) { return; }

            const float sx = (img_max.x - img_min.x) / static_cast<float>(_cur_roi.width);
            const float sy = (img_max.y - img_min.y) / static_cast<float>(_cur_roi.height);
            if (!(sx > 0.0f) || !(sy > 0.0f)) { return; }

            const auto to_screen = [&](float x, float y) {
                return ImVec2{ img_min.x + (x - static_cast<float>(_cur_roi.offset_x)) * sx,
                               img_min.y + (y - static_cast<float>(_cur_roi.offset_y)) * sy };
            };

            float x0 = static_cast<float>(_roi_edit[2]);
            float y0 = static_cast<float>(_roi_edit[3]);
            float x1 = x0 + static_cast<float>(_roi_edit[0]);
            float y1 = y0 + static_cast<float>(_roi_edit[1]);

            const ImVec2 p0 = to_screen(x0, y0);
            const ImVec2 p1 = to_screen(x1, y1);
            const ImVec2 saved_cursor = ImGui::GetCursorScreenPos();
            const ImVec2 io_delta = ImGui::GetIO().MouseDelta;
            bool changed = false;

            // Hit areas outside the image would grow the child window, so clamp them to it.
            const auto within_image = [&](const ImVec2& p) {
                return p.x >= img_min.x && p.x <= img_max.x && p.y >= img_min.y && p.y <= img_max.y;
            };

            const ImVec2 m0{ std::clamp(p0.x, img_min.x, img_max.x), std::clamp(p0.y, img_min.y, img_max.y) };
            const ImVec2 m1{ std::clamp(p1.x, img_min.x, img_max.x), std::clamp(p1.y, img_min.y, img_max.y) };

            // Submitted first so the corner grips below win the hover test where they overlap.
            if (m1.x - m0.x >= 1.0f && m1.y - m0.y >= 1.0f) {
                ImGui::SetCursorScreenPos(m0);
                ImGui::InvisibleButton("##roi_move", ImVec2{ m1.x - m0.x, m1.y - m0.y });
                if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
                }
                if (ImGui::IsItemActive()) {
                    const float dx = io_delta.x / sx, dy = io_delta.y / sy;
                    x0 += dx; x1 += dx; y0 += dy; y1 += dy;
                    changed = true;
                }
            }

            struct grip_t { const char* id; ImVec2 at; float* x; float* y; ImGuiMouseCursor cursor; };
            const grip_t grips[] = {
                { "##roi_tl", p0,                  &x0, &y0, ImGuiMouseCursor_ResizeNWSE },
                { "##roi_tr", ImVec2{ p1.x, p0.y }, &x1, &y0, ImGuiMouseCursor_ResizeNESW },
                { "##roi_bl", ImVec2{ p0.x, p1.y }, &x0, &y1, ImGuiMouseCursor_ResizeNESW },
                { "##roi_br", p1,                  &x1, &y1, ImGuiMouseCursor_ResizeNWSE },
            };
            constexpr float kGrip = 12.0f;
            for (const grip_t& g : grips) {
                if (!within_image(g.at)) { continue; }
                ImGui::SetCursorScreenPos(ImVec2{ g.at.x - kGrip * 0.5f, g.at.y - kGrip * 0.5f });
                ImGui::InvisibleButton(g.id, ImVec2{ kGrip, kGrip });
                if (ImGui::IsItemHovered() || ImGui::IsItemActive()) { ImGui::SetMouseCursor(g.cursor); }
                if (ImGui::IsItemActive()) {
                    *g.x += io_delta.x / sx;
                    *g.y += io_delta.y / sy;
                    changed = true;
                }
            }

            // A zero-size item validates the restored cursor; ImGui asserts on the extent without it.
            ImGui::SetCursorScreenPos(saved_cursor);
            ImGui::Dummy(ImVec2{ 0.0f, 0.0f });

            if (!changed) { return; }

            // Grips can flip the rect: normalize, then clamp to the sensor. `inc` snapping is on apply.
            if (x1 < x0) { std::swap(x0, x1); }
            if (y1 < y0) { std::swap(y0, y1); }

            const float max_w = static_cast<float>(_sensor_max[0] > 0 ? _sensor_max[0] : _cur_roi.width);
            const float max_h = static_cast<float>(_sensor_max[1] > 0 ? _sensor_max[1] : _cur_roi.height);
            constexpr float kMinExtent = 16.0f;

            x0 = std::clamp(x0, 0.0f, max_w - kMinExtent);
            y0 = std::clamp(y0, 0.0f, max_h - kMinExtent);
            x1 = std::clamp(x1, x0 + kMinExtent, max_w);
            y1 = std::clamp(y1, y0 + kMinExtent, max_h);

            _roi_edit[0] = static_cast<int>(x1 - x0);
            _roi_edit[1] = static_cast<int>(y1 - y0);
            _roi_edit[2] = static_cast<int>(x0);
            _roi_edit[3] = static_cast<int>(y0);
        }

        // Outlines the pending ROI and dims what it would cut away.
        // Edit values are sensor coords, the frame is only the current readout: rebase before scaling.
        void _draw_roi_overlay(const ImVec2& img_min, const ImVec2& img_max)
        {
            if (_cur_roi.width <= 0 || _cur_roi.height <= 0) { return; }

            const float sx = (img_max.x - img_min.x) / static_cast<float>(_cur_roi.width);
            const float sy = (img_max.y - img_min.y) / static_cast<float>(_cur_roi.height);

            ImVec2 p0{
                img_min.x + (_roi_edit[2] - static_cast<float>(_cur_roi.offset_x)) * sx,
                img_min.y + (_roi_edit[3] - static_cast<float>(_cur_roi.offset_y)) * sy
            };
            ImVec2 p1{ p0.x + _roi_edit[0] * sx, p0.y + _roi_edit[1] * sy };

            p0.x = std::clamp(p0.x, img_min.x, img_max.x);
            p0.y = std::clamp(p0.y, img_min.y, img_max.y);
            p1.x = std::clamp(p1.x, img_min.x, img_max.x);
            p1.y = std::clamp(p1.y, img_min.y, img_max.y);

            ImDrawList* dl = ImGui::GetWindowDrawList();
            const ImU32 dim = IM_COL32(0, 0, 0, 120);
            dl->AddRectFilled(img_min, ImVec2{ img_max.x, p0.y }, dim);           // above
            dl->AddRectFilled(ImVec2{ img_min.x, p1.y }, img_max, dim);           // below
            dl->AddRectFilled(ImVec2{ img_min.x, p0.y }, ImVec2{ p0.x, p1.y }, dim); // left
            dl->AddRectFilled(ImVec2{ p1.x, p0.y }, ImVec2{ img_max.x, p1.y }, dim); // right
            dl->AddRect(p0, p1, IM_COL32(255, 210, 0, 255), 0.0f, ImDrawFlags_None, 2.0f);

            // Corner grips, matching the hit areas in `_handle_roi_interaction()`.
            constexpr float kGrip = 12.0f;
            const ImVec2 corners[] = {
                p0, ImVec2{ p1.x, p0.y }, ImVec2{ p0.x, p1.y }, p1
            };
            for (const ImVec2& c : corners) {
                dl->AddRectFilled(ImVec2{ c.x - kGrip * 0.5f, c.y - kGrip * 0.5f },
                                  ImVec2{ c.x + kGrip * 0.5f, c.y + kGrip * 0.5f },
                                  IM_COL32(255, 210, 0, 255));
            }

            char label[96];
            std::snprintf(label, sizeof(label), "%d x %d @ %d,%d",
                _roi_edit[0], _roi_edit[1], _roi_edit[2], _roi_edit[3]);
            dl->AddText(ImVec2{ p0.x + 6.0f, p0.y + 4.0f }, IM_COL32(255, 210, 0, 255), label);
        }

        void _render_controls()
        {
            // ----- discovery -----
            if (ImGui::CollapsingHeader("Device", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::Button("Enumerate")) {
                    std::string err_msg;
                    _devices = vz::device::enumerate(500, &err_msg);
                    _selected = _devices.empty() ? -1 : 0;
                    spdlog::info("vz: {} device(s) found", _devices.size());
                    if (!err_msg.empty()) { spdlog::error("vz: enumerate: {}", err_msg); }
                    for (const auto& d : _devices) {
                        spdlog::info("  {} / {} / sn={}", d.vendor, d.model, d.serial);
                    }
                }

                for (int i = 0; i < static_cast<int>(_devices.size()); ++i) {
                    const auto& d = _devices[i];
                    char label[256];
                    std::snprintf(label, sizeof(label), "%s  (sn %s)##dev%d",
                        d.model.c_str(), d.serial.c_str(), i);
                    if (ImGui::Selectable(label, _selected == i)) { _selected = i; }
                }
                if (_devices.empty()) { ImGui::TextDisabled("no cameras"); }

                ImGui::BeginDisabled(_selected < 0 || _dev.is_open());
                if (ImGui::Button("Open")) {
                    if (_dev.open(_devices[_selected].serial)) { this->_refresh(); }
                }
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::BeginDisabled(!_dev.is_open());
                if (ImGui::Button("Close")) {
                    _grabber.stop();
                    _dev.close();
                }
                ImGui::EndDisabled();
            }

            if (!_dev.is_open()) {
                if (const std::string err_msg = _dev.last_err_msg(); !err_msg.empty()) {
                    ImGui::TextWrapped("last error: %s", err_msg.c_str());
                }
                return;
            }

            // ----- acquisition -----
            if (ImGui::CollapsingHeader("Acquisition", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::BeginDisabled(_dev.is_streaming());
                if (ImGui::Button("Start")) {
                    if (_dev.start_stream()) { _grabber.start(_dev); }
                }
                ImGui::EndDisabled();
                ImGui::SameLine();
                ImGui::BeginDisabled(!_dev.is_streaming());
                if (ImGui::Button("Stop")) {
                    _grabber.stop();
                    _dev.stop_stream();
                }
                ImGui::EndDisabled();

                const vz::stream_stats_t s = _dev.stats();
                ImGui::Text("payload   %llu bytes", (unsigned long long)s.payload_bytes);
                ImGui::Text("delivered %llu  incomplete %llu  timeouts %llu",
                    (unsigned long long)s.frames, (unsigned long long)s.incomplete,
                    (unsigned long long)s.timeouts);
                ImGui::Text("fps %.2f", s.fps);
                ImGui::Text("demosaic %.2f ms  -> %s", s.convert_ms,
                    _dev.frame_format() == vz::frame_format_t::gray ? "gray" : "bgr");

                this->_enum_combo("PixelFormat");
                this->_enum_combo("AcquisitionMode");
                this->_enum_combo("TriggerMode");
                this->_float_drag("AcquisitionFrameRate");
                this->_enum_combo("AcquisitionFrameRateMode");
            }

            // ----- apriltag -----
            if (ImGui::CollapsingHeader("AprilTag detection", ImGuiTreeNodeFlags_DefaultOpen)) {
                // One channel is enough, so enabling detection puts the camera in gray output.
                if (ImGui::Checkbox("Enable", &_detect_enabled)) {
                    _dev.set_frame_format(_detect_enabled ? vz::frame_format_t::gray
                                                          : vz::frame_format_t::bgr);
                    _redetect = true;
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(tagStandard41h12, 2D only)");

                ImGui::BeginDisabled(!_detect_enabled);
                bool changed = false;
                changed |= ImGui::SliderFloat("quad_decimate", &_tune.quad_decimate, 1.0f, 4.0f, "%.1f");
                changed |= ImGui::SliderFloat("quad_sigma", &_tune.quad_sigma, 0.0f, 2.0f, "%.2f");
                changed |= ImGui::Checkbox("refine_edges", &_tune.refine_edges);
                changed |= ImGui::SliderInt("num_threads", &_tune.num_threads, 1, 16);
                if (changed) { this->_rebuild_detector(); }
                if (ImGui::Button("Reset params")) {
                    _tune = pose::tag_tuning_t{};
                    this->_rebuild_detector();
                }
                ImGui::EndDisabled();

                ImGui::Separator();
                ImGui::Text("tags       %d", static_cast<int>(_detections.size()));
                ImGui::Text("detect     %.2f ms   %.1f fps", _detect_ms, _detect_fps);
                if (!_detections.empty()) {
                    const double margin = std::accumulate(
                        _detections.begin(), _detections.end(), 0.0,
                        [](double a, const pose::tag_detection_t& d) { return a + d.decision_margin; })
                        / static_cast<double>(_detections.size());
                    int hamming = 0;
                    for (const auto& d : _detections) { hamming += d.hamming; }
                    ImGui::Text("margin     %.1f (mean)   hamming %d (sum)", margin, hamming);

                    std::string ids;
                    for (size_t i = 0; i < _detections.size() && i < 16; ++i) {
                        ids += (i ? ", " : "");
                        ids += std::to_string(_detections[i].id);
                    }
                    if (_detections.size() > 16) { ids += ", ..."; }
                    ImGui::TextWrapped("ids        %s", ids.c_str());
                }
            }

            // ----- exposure / gain -----
            if (ImGui::CollapsingHeader("Exposure / Gain", ImGuiTreeNodeFlags_DefaultOpen)) {
                this->_enum_combo("ExposureAuto");
                this->_enum_combo("ExposureTimeMode");
                this->_float_drag("ExposureTime", true);
                ImGui::Separator();
                this->_enum_combo("GainAuto");
                this->_enum_combo("GainSelector");
                this->_float_drag("Gain");
                ImGui::Separator();
                this->_enum_combo("BalanceWhiteAuto");
                this->_float_drag("BlackLevel");
            }

            // ----- roi -----
            if (ImGui::CollapsingHeader("ROI", ImGuiTreeNodeFlags_DefaultOpen)) {
                const auto w = _dev.read_int("Width");
                const auto h = _dev.read_int("Height");
                const auto ox = _dev.read_int("OffsetX");
                const auto oy = _dev.read_int("OffsetY");

                if (!w || !h) {
                    ImGui::TextDisabled("Width/Height not readable");
                } else {
                    ImGui::Text("Width   %lld  [%lld..%lld] inc %lld %s",
                        (long long)w->value, (long long)w->min, (long long)w->max,
                        (long long)w->inc, w->flags.writable ? "rw" : "ro");
                    ImGui::Text("Height  %lld  [%lld..%lld] inc %lld %s",
                        (long long)h->value, (long long)h->min, (long long)h->max,
                        (long long)h->inc, h->flags.writable ? "rw" : "ro");
                    if (ox) {
                        ImGui::Text("OffsetX %lld  [%lld..%lld] inc %lld %s",
                            (long long)ox->value, (long long)ox->min, (long long)ox->max,
                            (long long)ox->inc, ox->flags.writable ? "rw" : "ro");
                    }
                    if (oy) {
                        ImGui::Text("OffsetY %lld  [%lld..%lld] inc %lld %s",
                            (long long)oy->value, (long long)oy->min, (long long)oy->max,
                            (long long)oy->inc, oy->flags.writable ? "rw" : "ro");
                    }

                    if (!_roi_edit_valid) {
                        _roi_edit[0] = static_cast<int>(w->value);
                        _roi_edit[1] = static_cast<int>(h->value);
                        _roi_edit[2] = static_cast<int>(ox ? ox->value : 0);
                        _roi_edit[3] = static_cast<int>(oy ? oy->value : 0);
                        _roi_edit_valid = true;
                    }

                    // Current readout region: what the overlay rebases the pending ROI onto.
                    _cur_roi = vz::roi_t{ w->value, h->value,
                                          ox ? ox->value : 0, oy ? oy->value : 0 };

                    ImGui::DragInt2("size##roi", _roi_edit, 8.0f, 1, _sensor_max[0]);
                    ImGui::DragInt2("offset##roi", _roi_edit + 2, 8.0f, 0, _sensor_max[0]);
                    ImGui::Checkbox("Preview region on image", &_roi_preview);
                    if (_roi_preview) {
                        ImGui::TextDisabled("Drag inside the box to move it, corners to resize.");
                    }

                    if (ImGui::Button("Apply ROI")) {
                        this->_apply_roi(vz::roi_t{
                            _roi_edit[0], _roi_edit[1], _roi_edit[2], _roi_edit[3] });
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Reset")) {
                        this->_apply_roi(vz::roi_t{ _sensor_max[0], _sensor_max[1], 0, 0 });
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Revert edit")) { _roi_edit_valid = false; }
                    ImGui::TextDisabled("Applying stops and restarts acquisition:\nthe ROI nodes are locked while the camera streams.");
                }
            }

            // ----- identity -----
            if (ImGui::CollapsingHeader("Identity")) {
                for (const char* n : kIdentityNodes) {
                    if (const auto v = _dev.read_string(n)) { ImGui::Text("%-22s %s", n, v->c_str()); }
                    else { ImGui::TextDisabled("%-22s n/a", n); }
                }
                if (const auto t = _dev.read_float("DeviceTemperature")) {
                    ImGui::Text("%-22s %.1f %s", "DeviceTemperature", t->value, t->unit.c_str());
                }
            }

            if (const std::string err_msg = _dev.last_err_msg(); !err_msg.empty()) {
                ImGui::Separator();
                ImGui::TextWrapped("last error: %s", err_msg.c_str());
            }
        }

        // Enum node rendered as a combo; writes on selection.
        void _enum_combo(const char* name)
        {
            const auto f = _dev.read_enum(name);
            if (!f) { ImGui::TextDisabled("%s: n/a", name); return; }

            ImGui::BeginDisabled(!f->flags.writable);
            if (ImGui::BeginCombo(name, f->value.c_str())) {
                for (const std::string& e : f->entries) {
                    if (ImGui::Selectable(e.c_str(), e == f->value)) {
                        if (_dev.write_enum(name, e)) { spdlog::info("vz: {} = {}", name, e); }
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::EndDisabled();
        }

        // Camera float node rendered as a drag control. 
        // Ranges vary wildly (us..s for exposure, ~24 dB for gain), so the step comes from the node's own range.
        void _float_drag(const char* name, bool logarithmic = false)
        {
            const auto f = _dev.read_float(name);
            if (!f) { ImGui::TextDisabled("%s: n/a", name); return; }

            float v = static_cast<float>(f->value);
            float step = static_cast<float>((f->max - f->min) / 500.0);
            if (!(step > 0.0f)) { step = 0.01f; }

            const ImGuiSliderFlags flags = ImGuiSliderFlags_AlwaysClamp
                | (logarithmic ? ImGuiSliderFlags_Logarithmic : ImGuiSliderFlags_None);

            ImGui::BeginDisabled(!f->flags.writable);
            char label[128];
            std::snprintf(label, sizeof(label), "%s [%s]", name, f->unit.empty() ? "-" : f->unit.c_str());
            if (ImGui::DragFloat(label, &v, step,
                    static_cast<float>(f->min), static_cast<float>(f->max), "%.3f", flags))
            {
                (void)_dev.write_float(name, static_cast<double>(v));
            }
            ImGui::EndDisabled();
        }

        void _render_dump_window()
        {
            ImGui::SetNextWindowSize(ImVec2{ 760.0f, 520.0f }, ImGuiCond_FirstUseEver);
            if (ImGui::Begin("Feature explorer", &_show_dump)) {
                ImGui::BeginDisabled(!_dev.is_open());
                if (ImGui::Button("Read all")) {
                    _dump = _dev.dump_features();
                    spdlog::info("vz: dumped {} feature nodes", _dump.size());
                }
                ImGui::EndDisabled();
                ImGui::SameLine();
                if (ImGui::Button("Copy")) {
                    std::string all;
                    for (const std::string& l : _dump) { all += l; all += '\n'; }
                    ImGui::SetClipboardText(all.c_str());
                }
                ImGui::SameLine();
                _dump_filter.Draw("filter", 200.0f);
                ImGui::Separator();

                ImGui::BeginChild("##dumplist");
                for (const std::string& line : _dump) {
                    if (!_dump_filter.PassFilter(line.c_str())) { continue; }
                    ImGui::TextUnformatted(line.c_str());
                }
                ImGui::EndChild();
            }
            ImGui::End();
        }

        void _refresh()
        {
            _roi_edit_valid = false;
            _dump.clear();
            // Sensor extents are fixed for the session; the ROI editor clamps against them.
            if (const auto wm = _dev.read_int("WidthMax"))  { _sensor_max[0] = static_cast<int>(wm->value); }
            if (const auto hm = _dev.read_int("HeightMax")) { _sensor_max[1] = static_cast<int>(hm->value); }
            for (const char* n : kIdentityNodes) {
                if (const auto v = _dev.read_string(n)) { spdlog::info("vz: {} = {}", n, *v); }
            }
        }

    private:
        vz::device _dev;
        frame_latch_t _grabber;
        std::vector<vz::device_info_t> _devices;
        int _selected{ -1 };

        std::optional<gui::frame_texture> _tex;
        gui::log_console _log;

        // tag detection: annotated copy shown in the preview, plus its cost counters
        std::optional<pose::tag_detector> _detector;
        pose::tag_tuning_t _tune{};
        std::vector<pose::tag_detection_t> _detections;
        cv::Mat _display;                        // frame as drawn (annotated when detection is on)
        uint64_t _last_seq{ UINT64_MAX };        // stream frame counter the overlay was built from
        bool _detect_enabled{ false };
        bool _redetect{ true };                  // forces one re-run after a settings change
        double _detect_ms{ 0.0 };
        double _detect_fps{ 0.0 };
        std::chrono::steady_clock::time_point _last_detect_at{ std::chrono::steady_clock::now() };

        bool _show_log{ true };
        bool _show_dump{ false };
        bool _fit_preview{ true };

        int  _roi_edit[4]{ 0, 0, 0, 0 }; // pending width, height, offset_x, offset_y
        bool _roi_edit_valid{ false };
        bool _roi_preview{ true };       // outline the pending ROI over the live image
        vz::roi_t _cur_roi{};            // region currently read out, refreshed by the ROI panel
        int  _sensor_max[2]{ 0, 0 };     // `WidthMax` / `HeightMax`, read once per open

        std::vector<std::string> _dump;
        ImGuiTextFilter _dump_filter;
    };

} // namespace

int main(int, char**)
{
    spdlog::set_level(spdlog::level::debug);
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

    vzcam_test_app app;
    return app.run();
}
