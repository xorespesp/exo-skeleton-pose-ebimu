// Debug harness for bringing the Vieworks VZ camera into the project.
//
// Exercises the pieces a sensor backend needs (discovery, open, pull-based acquisition,
// debayering, exposure/gain, ROI, GenICam node access) against a live camera.

#include "hw/backends/vz_device.hh"

#include "gui/app_base.hh"
#include "gui/app_renderer_sdl3.hh"
#include "gui/frame_texture.hh"
#include "gui/log_console.hh"
#include "pose/color_marker_detector.hh"
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
#include <string_view>
#include <thread>
#include <vector>

namespace
{
    // Rows the marker table reserves. Detections come and go every frame, so the table is given a
    // fixed height and scrolls: without it the panels below shift up and down with the count.
    constexpr int kMarkerTableRows = 6;

    // Slider bounds for the color-marker settings. ImGui takes these by pointer for double widgets.
    constexpr double kZero = 0.0, kOne = 1.0;
    constexpr double kDistMin = 0.5, kDistMax = 8.0;
    constexpr double kAreaMin = 1.0, kAreaMax = 200000.0;
    constexpr double kAspectMin = 1.0, kAspectMax = 8.0;

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
            if (_dev.is_streaming())
            {
                ImGui::Text("|  %ux%u %s -> %s  |  stream %.1f fps  demosaic %.2f ms  |  frames %llu  incomplete %llu  timeouts %llu",
                    s.width, s.height, s.pixel_format.c_str(),
                    _dev.frame_format() == vz::frame_format_t::gray ? "gray" : "bgr",
                    s.fps, s.convert_ms,
                    (unsigned long long)s.frames, (unsigned long long)s.incomplete,
                    (unsigned long long)s.timeouts);
            }

            if (_detect_mode == detect_mode_t::apriltag)
            {
                ImGui::Text("|  detect %.1f fps (%.2f ms)  tags %d",
                    _detect_fps, _detect_ms, static_cast<int>(_detections.size()));
            }
            else if (_detect_mode == detect_mode_t::color_marker)
            {
                ImGui::Text("|  detect %.1f fps (%.2f ms)  markers %d",
                    _detect_fps, _detect_ms, static_cast<int>(_markers.size()));
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
                _frame.release();
                _display.release();
                return;
            }
            _frame = frame; // sampling reads the raw image, not the annotated copy

            const uint64_t seq = _dev.stats().frames;

            if (_detect_mode == detect_mode_t::none) {
                _display = frame;
                _last_seq = seq;
                _detections.clear();
                _markers.clear();
                return;
            }

            // Re-run only for a frame not seen yet, or right after a settings change.
            if (seq == _last_seq && !_redetect && !_display.empty()) { return; }
            _last_seq = seq;
            _redetect = false;

            using clock = std::chrono::steady_clock;
            const auto t0 = clock::now();

            cv::Mat annotated;
            if (_detect_mode == detect_mode_t::apriltag)
            {
                if (!_detector.has_value()) { this->_rebuild_detector(); }
                _detections = _detector->detect(frame);

                // Overlay is drawn in colour: widen gray for display only
                if (frame.channels() == 1) { cv::cvtColor(frame, annotated, cv::COLOR_GRAY2BGR); }
                else { annotated = frame.clone(); }
                pose::draw_tag_detections(annotated, _detections);
            }
            else
            {
                _markers = _color_detector.detect(frame);
                annotated = this->_build_color_view(frame);
                pose::draw_marker_detections(annotated, _markers);
            }

            const double ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
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
            _detector.emplace(_detector_opt);
            _detect_ms = 0.0;
            _detect_fps = 0.0;
            _redetect = true;
        }

        // Backdrop for the color-marker overlay: the image itself, the binary mask, or the
        // per-pixel membership score. Seeing the mask and the score is what makes a mis-tuned
        // model obvious, so they get their own view rather than a separate window.
        cv::Mat _build_color_view(const cv::Mat& frame) const
        {
            const cv::Mat& src = (_color_view == 1) ? _color_detector.mask()
                               : (_color_view == 2) ? _color_detector.score_image()
                                                    : frame;
            cv::Mat out;
            if (src.empty()) { out = cv::Mat::zeros(frame.size(), CV_8UC3); }
            else if (src.channels() == 1) { cv::cvtColor(src, out, cv::COLOR_GRAY2BGR); }
            else { out = src.clone(); }
            return out;
        }

        // Switches the camera output to what the selected detector reads.
        void _apply_detect_mode()
        {
            _dev.set_frame_format(_detect_mode == detect_mode_t::apriltag ? vz::frame_format_t::gray
                                                                         : vz::frame_format_t::bgr);
            _detections.clear();
            _markers.clear();
            _detect_ms = 0.0;
            _detect_fps = 0.0;
            _redetect = true;
        }

        // Refits the color model from whatever the sampler has collected so far.
        void _refit_color_model()
        {
            if (const auto model = _sampler.fit(_color_max_distance)) {
                _color_detector.set_model(*model);
                spdlog::info("color: model fitted from {} samples, mean a*{:+.1f} b*{:+.1f}",
                    _sampler.count(), model->mean_ab.x(), model->mean_ab.y());
            } else {
                spdlog::warn("color: need at least {} samples, have {}",
                    pose::color_sampler::kMinSamples, _sampler.count());
            }
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

            if (_sampling) {
                this->_handle_sample_click(ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
            }

            if (_roi_preview && _roi_edit_valid) {
                const ImVec2 mn = ImGui::GetItemRectMin();
                const ImVec2 mx = ImGui::GetItemRectMax();
                this->_handle_roi_interaction(mn, mx);
                this->_draw_roi_overlay(mn, mx);
            }
        }

        // Clicking (or dragging across) a marker feeds the pixels under the cursor to the sampler.
        // The preview may be scaled to fit, so screen coordinates are unscaled back to image pixels.
        void _handle_sample_click(const ImVec2& img_min, const ImVec2& img_max)
        {
            if (_frame.empty()) { return; }
            if (!ImGui::IsItemHovered() || !ImGui::IsMouseDown(ImGuiMouseButton_Left)) { return; }

            const float span_x = img_max.x - img_min.x;
            const float span_y = img_max.y - img_min.y;
            if (!(span_x > 0.0f) || !(span_y > 0.0f)) { return; }

            const ImVec2 m = ImGui::GetIO().MousePos;
            const int px = static_cast<int>((m.x - img_min.x) / span_x * static_cast<float>(_frame.cols));
            const int py = static_cast<int>((m.y - img_min.y) / span_y * static_cast<float>(_frame.rows));
            if (px < 0 || py < 0 || px >= _frame.cols || py >= _frame.rows) { return; }

            _sampler.add(_frame, cv::Point{ px, py }, _sample_radius);

            // Draw where the sample was taken so a drag leaves a visible trail.
            ImDrawList* dl = ImGui::GetWindowDrawList();
            const float r = static_cast<float>(_sample_radius) * span_x / static_cast<float>(_frame.cols);
            dl->AddCircle(m, std::max(3.0f, r), IM_COL32(255, 255, 0, 220), 0, 2.0f);
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

            // ----- detector selection -----
            if (ImGui::CollapsingHeader("Detection", ImGuiTreeNodeFlags_DefaultOpen)) {
                // Each detector wants a different camera output, so the mode owns the frame format.
                const char* const modes[] = { "Off", "AprilTag (gray)", "Color marker (bgr)" };
                int mode = static_cast<int>(_detect_mode);
                if (ImGui::Combo("Mode", &mode, modes, IM_ARRAYSIZE(modes))) {
                    _detect_mode = static_cast<detect_mode_t>(mode);
                    this->_apply_detect_mode();
                }
                ImGui::Text("detect     %.2f ms   %.1f fps", _detect_ms, _detect_fps);
            }

            // ----- apriltag -----
            if (_detect_mode == detect_mode_t::apriltag &&
                ImGui::CollapsingHeader("Tag detection", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::TextDisabled("(tagStandard41h12, 2D only)");

                bool changed = false;
                changed |= ImGui::SliderFloat("quad_decimate", &_detector_opt.quad_decimate, 1.0f, 4.0f, "%.1f");
                changed |= ImGui::SliderFloat("quad_sigma", &_detector_opt.quad_sigma, 0.0f, 2.0f, "%.2f");
                changed |= ImGui::Checkbox("refine_edges", &_detector_opt.refine_edges);
                changed |= ImGui::SliderInt("num_threads", &_detector_opt.num_threads, 1, 16);
                if (changed) { this->_rebuild_detector(); }
                if (ImGui::Button("Reset params")) {
                    _detector_opt = pose::tag_detector::options_t{};
                    this->_rebuild_detector();
                }

                ImGui::Separator();
                ImGui::Text("tags       %d", static_cast<int>(_detections.size()));
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

            // ----- color marker -----
            if (_detect_mode == detect_mode_t::color_marker) { this->_render_color_marker_panel(); }

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

        // NOTE: tooltips stay in English because ImGui's built-in font carries no Korean glyphs.
        //       The Korean commentary lives in the comments here and in color_marker_detector.hh.
        void _render_color_marker_panel()
        {
            pose::color_marker_detector::options_t& opt = _color_detector.options();

            // =====================================================================
            // 색 모델: "이 픽셀이 마커 색인가" 를 판정하는 기준
            // =====================================================================
            //
            // **어떤 색도 코드에 박혀 있지 않다.** 값을 손으로 입력하는 것이 아니라 화면에서
            // 마커를 직접 찍어 만든다. 시험소마다 조명과 카메라가 달라 같은 마커도 다른 색으로
            // 찍히므로, 표본에서 뽑는 방식만이 현장에서 통한다. Fit 전까지는 검출이 비어 있다.
            //
            // 참고로 인쇄용 마젠타(#E6007E)를 정상적인 조명에서 찍으면 대략 a*+79 b*-3 근처가
            // 나온다. 이것은 상수가 아니라 진단 기준이다. 마젠타 마커를 찍었는데 여기서 크게
            // 벗어난 값이 나오면 인쇄, 조명, 화이트밸런스 중 하나를 의심해야 한다.
            ImGui::SeparatorText("Color model");

            // 켜면 프리뷰를 클릭하거나 드래그할 때 커서 아래 픽셀이 표본으로 쌓인다.
            ImGui::Checkbox("Sample on click", &_sampling);
            ImGui::SetItemTooltip(
                "Click or drag over a marker in the live preview to collect the pixels\n"
                "under the cursor into the sample set.\n"
                "Samples accumulate across clicks and frames until you press Fit.\n"
                "\n"
                "Move the leg through several positions while sampling. A model built\n"
                "from one pose does not cover the lighting the marker will actually meet,\n"
                "and it will start dropping the marker mid-stride.");

            // 클릭 한 번에 몇 픽셀 반경까지 담을지. 마커 안쪽에 머물러야 한다.
            ImGui::SliderInt("Sample radius", &_sample_radius, 2, 40, "%d px");
            ImGui::SetItemTooltip(
                "How far around the cursor pixels are taken, in image pixels.\n"
                "Keep it inside the coloured disc: anything reaching the black ring or\n"
                "the background drags the model toward those colours.\n"
                "\n"
                "Higher: fills the sample set faster, but risks catching the rim.\n"
                "Lower: safer, needs more clicks.");

            ImGui::Text("samples    %zu  (need %zu)", _sampler.count(),
                        pose::color_sampler::kMinSamples);
            ImGui::SetItemTooltip(
                "Pixels collected so far. The stated minimum is only what the fit needs\n"
                "to be defined at all; a few thousand across varied poses is what makes\n"
                "the model hold up in the lab.");

            // 표본이 최소치를 넘어야 적합이 가능하다.
            const bool enough = _sampler.count() >= pose::color_sampler::kMinSamples;
            ImGui::BeginDisabled(!enough);
            if (ImGui::Button("Fit model")) { this->_refit_color_model(); }
            ImGui::EndDisabled();
            ImGui::SetItemTooltip(
                "Computes the mean and the covariance of the collected samples, installs\n"
                "them as the colour model, and rebuilds the lookup table the classifier\n"
                "reads. Disabled until enough samples exist.");

            ImGui::SameLine();
            // 표본만 버린다. 이미 설치된 모델은 다음 Fit 까지 그대로 쓰인다.
            if (ImGui::Button("Clear samples")) { _sampler.clear(); }
            ImGui::SetItemTooltip(
                "Discards the collected samples. The installed model stays in force until\n"
                "the next Fit, so detection keeps running while you resample.");

            // 분류기에서 사람이 맞춰야 하는 설정값은 이것 하나뿐이다. 모델 중심에서 몇
            // 표준편차만큼 떨어진 픽셀까지 마커로 인정할 것인가를 정한다.
            // (HSV 방식이라면 H/S/V 최소·최대 여섯 개를 맞춰야 하는 자리다.)
            if (ImGui::SliderScalar("Max distance", ImGuiDataType_Double, &_color_max_distance,
                    &kDistMin, &kDistMax, "%.2f")) {
                opt.model.max_distance = _color_max_distance;
                _color_detector.rebuild_lut();
                _redetect = true;
            }
            ImGui::SetItemTooltip(
                "How far from the model's centre a pixel may sit and still count as the\n"
                "marker, in Mahalanobis distance. That distance is normalised by how the\n"
                "sampled colours are spread, so it follows the shape of the sample cloud\n"
                "instead of a fixed box.\n"
                "\n"
                "Roughly: 2.0 keeps ~86% of the sampled pixels, 3.0 keeps ~99%.\n"
                "\n"
                "Higher: the marker's rim is included, but background starts leaking in.\n"
                "Lower: only the core survives, blobs shrink and may fall under min area.");

            // 적합된 모델의 중심과 퍼진 폭. 모델이 없을 때도 같은 줄 수를 유지해서 패널 높이가
            // 변하지 않게 한다. 표준편차가 지나치게 작으면 한 자세에서만 표본을 뽑았다는 뜻이라,
            // 조명이 조금만 변해도 마커를 놓치게 된다.
            if (opt.model.valid) {
                ImGui::Text("mean       a*%+.1f  b*%+.1f",
                            opt.model.mean_ab.x(), opt.model.mean_ab.y());
            } else {
                ImGui::TextDisabled("mean       (no model yet: sample a marker, then Fit)");
            }
            ImGui::SetItemTooltip(
                "Centre of the model on the a*b* plane.\n"
                "a* runs green to red, b* runs blue to yellow. Lightness is deliberately\n"
                "left out so shadows and exposure changes do not move the model.");

            if (opt.model.valid) {
                ImGui::Text("spread     a*%.1f  b*%.1f  (std dev)",
                            std::sqrt(opt.model.cov_ab(0, 0)), std::sqrt(opt.model.cov_ab(1, 1)));
            } else {
                ImGui::TextDisabled("spread     -");
            }
            ImGui::SetItemTooltip(
                "How wide the sampled colours are spread on each axis, in a*b* units.\n"
                "\n"
                "A very small spread means everything was sampled from one pose under one\n"
                "light. The model will then be brittle: the moment the marker moves into a\n"
                "slightly different light it falls outside and the marker is lost.\n"
                "Keep sampling across poses until the spread stops growing.");

            // =====================================================================
            // 블롭 필터: 색이 맞는 픽셀 덩어리 중 무엇을 마커로 인정할지
            // =====================================================================
            //
            // 색만 맞으면 배경의 붉은 물체나 옷도 함께 걸린다. 
            // 여기서 크기와 모양으로 한 번 더 거른다. 
            // 네 게이트 중 하나라도 어긋나면 그 덩어리는 통째로 버려진다.
            //
            // 크기(area)와 모양(fill, aspect)은 재는 대상이 다르다.
            //   area   얼마나 큰가        픽셀 개수
            //   fill   얼마나 꽉 찼나     면적 / 외접 사각형 면적
            //   aspect 얼마나 길쭉한가    외접 사각형의 긴 변 / 짧은 변
            //
            // 원반과 도넛은 면적이 같아도 채움율이 다르므로, area 만으로는 구분되지 않는다.
            // 외접 사각형은 덩어리를 감싸는 가장 작은 직사각형이며, 원을 직접 찾는 대신 찾은
            // 덩어리가 원처럼 생겼는지를 이 사각형으로 검사한다. (자세한 근거는
            // color_marker_detector.hh 의 options_t 주석 참고)
            ImGui::SeparatorText("Blob filter");

            bool changed = false;

            // 크기 하한 [px²]. 지름 d 원의 면적은 pi*d^2/4 이므로 지름 20 px 이면 약 314 px².
            changed |= ImGui::DragScalar("min area", ImGuiDataType_Double, &opt.min_area_px, 1.0f,
                                         &kAreaMin, &kAreaMax, "%.0f px");
            ImGui::SetItemTooltip(
                "Smallest blob accepted, in pixels of area.\n"
                "A disc of diameter d covers pi*d^2/4, so a 20 px marker is about 314 px.\n"
                "\n"
                "Higher: drops speckle and distant noise.\n"
                "Lower: keeps markers that are far away or partly occluded.");

            // 크기 상한 [px²]. 색이 우연히 맞는 넓은 배경(벽, 옷)을 잘라낸다.
            changed |= ImGui::DragScalar("max area", ImGuiDataType_Double, &opt.max_area_px, 10.0f,
                                         &kAreaMin, &kAreaMax, "%.0f px");
            ImGui::SetItemTooltip(
                "Largest blob accepted, in pixels of area.\n"
                "Catches broad background regions that happen to match the colour, such as\n"
                "a painted wall or a piece of clothing.");

            // 채움율 = 덩어리 면적 / 외접 사각형 면적. 꽉 찬 원이면 pi/4 = 0.785 다.
            // 면적이 같아도 모양이 다를 수 있으므로, min area 가 못 잡는 것을 이것이 잡는다.
            //   원반  면적 314, 사각형 400  ->  0.785
            //   도넛  면적 314, 사각형 900  ->  0.35   (면적은 같은데 채움율이 다르다)
            changed |= ImGui::SliderScalar("min fill", ImGuiDataType_Double, &opt.min_fill,
                                           &kZero, &kOne, "%.2f");
            ImGui::SetItemTooltip(
                "Blob area divided by the area of its bounding box.\n"
                "A filled circle scores pi/4 = 0.785. A ring, a crescent or a ragged patch\n"
                "scores lower.\n"
                "\n"
                "Higher: solid round blobs only.\n"
                "Lower: tolerant of ragged shapes, but background clutter gets through.\n"
                "If a glossy marker is being rejected here, raise the close kernel first:\n"
                "a specular highlight punches a hole and drops the fill.");

            // 종횡비 = 외접 사각형의 긴 변 / 짧은 변. 원이면 가로세로가 같으므로 1 이다.
            // 노출이 길면 스윙하는 마커가 줄무늬로 늘어나 이 값이 커진다.
            //   지름 20 px, 이미지 상 속도 3478 px/s 기준
            //   노출 2 ms -> 27x20 -> 1.35        노출 8 ms -> 48x20 -> 2.4
            changed |= ImGui::SliderScalar("max aspect", ImGuiDataType_Double, &opt.max_aspect,
                                           &kAspectMin, &kAspectMax, "%.2f");
            ImGui::SetItemTooltip(
                "Bounding box long side over short side. A circle is 1.0.\n"
                "\n"
                "A long exposure smears a swinging marker into a streak, which pushes this\n"
                "well above 1. Keep it loose until the exposure is short enough that the\n"
                "marker stays round while the leg swings.\n"
                "\n"
                "Higher: survives motion blur, admits elongated background objects.\n"
                "Lower: strictly round blobs only.");

            // 열림 = 침식 후 팽창. 마스크 "밖"의 작은 점 노이즈를 지운다. 0 이면 건너뛴다.
            //   before:  ####  .  .        after:  ####
            //           ######    .               ######
            changed |= ImGui::SliderInt("open kernel", &opt.open_kernel_px, 0, 15, "%d px");
            ImGui::SetItemTooltip(
                "Erode then dilate, in pixels. Isolated specks smaller than the kernel\n"
                "disappear in the erode and never come back, while real blobs are restored\n"
                "to their original size by the dilate. 0 skips this step.\n"
                "\n"
                "Higher: cleaner mask, but a thin marker rim gets eaten away.\n"
                "Lower: keeps every pixel the classifier accepted, speckle included.");

            // 닫힘 = 팽창 후 침식. 열림과 목적이 반대로, 마스크 "안"의 구멍을 메운다.
            // 광택 마커는 정반사로 한가운데가 날아가 도넛이 되는데, 그대로 두면 하이라이트가
            // 다리 움직임을 따라 이동하면서 무게중심까지 흔들린다. 0 이면 건너뛴다.
            //   before:  ######           after:  ######
            //          ##    ##                 ########
            //           ######                   ######
            changed |= ImGui::SliderInt("close kernel", &opt.close_kernel_px, 0, 15, "%d px");
            ImGui::SetItemTooltip(
                "Dilate then erode, in pixels. Fills holes inside a blob. 0 skips this step.\n"
                "\n"
                "A glossy marker blows out to white in the middle, leaving a hole. Without\n"
                "closing it the blob is a donut, and its centroid wanders as the highlight\n"
                "moves with the leg.\n"
                "\n"
                "Higher: fills larger holes, but merges blobs that sit close together.\n"
                "Lower: leaves holes, which also shows up as a low fill value.");

            if (changed) { _redetect = true; }

            // =====================================================================
            // 결과: 무엇이 잡혔고 무엇이 왜 걸러졌는가
            // =====================================================================
            ImGui::SeparatorText("Result");

            // 오버레이를 어디에 그릴지. 마스크와 점수를 직접 보는 것이 튜닝의 핵심이다.
            const char* const views[] = { "Image", "Mask", "Score" };
            if (ImGui::Combo("Backdrop", &_color_view, views, IM_ARRAYSIZE(views))) {
                _redetect = true;
            }
            ImGui::SetItemTooltip(
                "What the overlay is drawn on top of.\n"
                "\n"
                "Image: the camera frame as captured.\n"
                "Mask:  the binary mask after the morphology steps. This is exactly what\n"
                "       the blob filters see, so it shows why a blob was the shape it was.\n"
                "Score: per-pixel membership, black through white. A soft fade at the\n"
                "       marker's edge means the model covers it well; a hard edge means\n"
                "       Max distance is cutting into the marker.");

            ImGui::Text("markers    %d", static_cast<int>(_markers.size()));

            // 걸러진 후보의 사유별 개수. 마커가 안 잡힐 때 어느 노브를 풀지 바로 알려준다.
            const pose::marker_reject_stats_t& r = _color_detector.reject_stats();
            ImGui::Text("rejected   %d  (small %d, large %d, unfilled %d, elongated %d)",
                        r.total(), r.too_small, r.too_large, r.not_filled, r.too_long);
            ImGui::SetItemTooltip(
                "Candidate blobs the filters dropped, and which filter dropped them.\n"
                "\n"
                "This is the fastest way to find out why a marker is missing:\n"
                "  small     -> lower min area, or move closer / use a bigger marker\n"
                "  large     -> lower max area, or a background region matches the colour\n"
                "  unfilled  -> raise the close kernel, or lower min fill\n"
                "  elongated -> shorten the exposure, or raise max aspect\n"
                "\n"
                "If everything reads 0 and no markers appear, the colour model itself is\n"
                "not matching: check the Score backdrop and raise Max distance.");

            // 표 자체에는 툴팁을 걸 수 없으므로 바로 위 라벨이 컬럼 설명을 갖는다.
            ImGui::TextDisabled("detections  (hover for column meanings)");
            ImGui::SetItemTooltip(
                "One row per accepted marker, largest first.\n"
                "\n"
                "x, y   subpixel centre in image pixels. Values that always land on .00\n"
                "       mean the weighted centroid is not working, usually because Max\n"
                "       distance is so tight that every accepted pixel carries the same\n"
                "       weight.\n"
                "dia    diameter derived from the blob area, as if the blob were a circle.\n"
                "       Compare it against what the marker should measure at this distance;\n"
                "       a value well under that means the model is clipping the rim.\n"
                "fill   area over bounding box. About 0.785 for a clean disc. Lower means\n"
                "       the blob has a hole or a ragged outline.\n"
                "score  mean membership over the blob, 0 to 1. Near 1.0 the pixels sit at\n"
                "       the heart of the model; a low value means the marker is only just\n"
                "       inside the threshold and is about to be lost.");

            // 검출 개수는 프레임마다 바뀐다. 높이를 고정하고 안에서 스크롤시켜야 이 아래
            // 패널들이 위아래로 흔들리지 않는다.
            const float table_h = ImGui::GetTextLineHeightWithSpacing() * (kMarkerTableRows + 1);
            if (ImGui::BeginTable("##markers", 5,
                    ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp |
                    ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersInnerV,
                    ImVec2(0.0f, table_h))) {
                ImGui::TableSetupScrollFreeze(0, 1); // 스크롤해도 머리글은 남는다
                ImGui::TableSetupColumn("x");
                ImGui::TableSetupColumn("y");
                ImGui::TableSetupColumn("dia");
                ImGui::TableSetupColumn("fill");
                ImGui::TableSetupColumn("score");
                ImGui::TableHeadersRow();
                for (const pose::marker_detection_t& m : _markers) {
                    ImGui::TableNextRow();
                    // x, y    서브픽셀 중심 [px]. 소수점이 항상 .00 으로만 나오면 가중
                    //         무게중심이 동작하지 않는다는 신호다.
                    // dia     면적을 원으로 환산한 지름. 이 거리에서 나와야 할 값보다 한참
                    //         작으면 색 모델이 마커 테두리를 잘라내고 있는 것이다.
                    // fill    0.785 근처가 정상. 낮으면 구멍이 있거나 윤곽이 찢어진 것.
                    // score   블롭 평균 소속도(0~1). 낮으면 임계에 간신히 걸쳐 있어 곧 놓칠 상태다
                    //         현재 판정에는 쓰이지 않는 표시 전용 값이다.
                    ImGui::TableNextColumn(); ImGui::Text("%.2f", m.center.x);
                    ImGui::TableNextColumn(); ImGui::Text("%.2f", m.center.y);
                    ImGui::TableNextColumn(); ImGui::Text("%.1f", m.diameter_px);
                    ImGui::TableNextColumn(); ImGui::Text("%.2f", m.fill);
                    ImGui::TableNextColumn(); ImGui::Text("%.2f", m.score);
                }
                ImGui::EndTable();
            }
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

        // Which detector the harness runs. Each wants its own camera output: apriltag reads one
        // channel, color markers need all three, so the mode drives the frame format.
        enum class detect_mode_t { none, apriltag, color_marker };

        detect_mode_t _detect_mode{ detect_mode_t::none };

        // apriltag detection
        std::optional<pose::tag_detector> _detector;
        pose::tag_detector::options_t _detector_opt{};
        std::vector<pose::tag_detection_t> _detections;

        // color marker detection: the model is sampled from the live image, so the sampler
        // accumulates across clicks and frames until there is enough to fit.
        pose::color_marker_detector _color_detector;
        pose::color_sampler _sampler;
        std::vector<pose::marker_detection_t> _markers;
        double _color_max_distance{ 3.0 };
        int _sample_radius{ 6 };     // clicks collect pixels within this radius [px]
        bool _sampling{ false };     // clicking the preview feeds the sampler
        int _color_view{ 0 };        // 0 image, 1 mask, 2 membership score

        cv::Mat _frame;                          // newest raw frame, kept for sampling
        cv::Mat _display;                        // frame as drawn (annotated when detection is on)
        uint64_t _last_seq{ UINT64_MAX };        // stream frame counter the overlay was built from
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

    // Exercises the color-marker detector against a drawn image, so the sampling / fitting /
    // detection chain can be checked without a camera on the desk. Returns a process exit code.
    int run_color_selftest()
    {
        // cv::circle takes an integer centre, so the disc is drawn in fixed point (`shift` says how
        // many of the low bits are fractional). Without this the marker would land on a whole pixel
        // and the test could not tell a subpixel centroid from a rounded one.
        constexpr int kShift = 4;                  // 1/16 px
        constexpr int kUnit = 1 << kShift;
        constexpr double kTrueX = 320.375, kTrueY = 241.6875, kTrueRadius = 11.0;

        // Grey background, a black ring, the colored disc, and then sensor noise over all of it.
        // Noise goes on last on purpose: a perfectly flat disc would make the fitted covariance
        // collapse onto its floor, and the subpixel weighting would never be exercised.
        cv::Mat img(480, 640, CV_8UC3, cv::Scalar(96, 98, 100));

        const cv::Point truth(cvRound(kTrueX * kUnit), cvRound(kTrueY * kUnit));
        cv::circle(img, truth, cvRound((kTrueRadius + 2.0) * kUnit),
                   cv::Scalar(0, 0, 0), -1, cv::LINE_AA, kShift);
        cv::circle(img, truth, cvRound(kTrueRadius * kUnit),
                   cv::Scalar(126, 0, 230), -1, cv::LINE_AA, kShift);

        cv::Mat noise(img.size(), CV_8UC3);
        cv::randn(noise, cv::Scalar::all(0), cv::Scalar::all(6));
        img += noise;

        // Several clicks, as an operator would make them: the sampler accumulates.
        pose::color_sampler sampler;
        sampler.add(img, cv::Point{ 320, 242 }, 7);
        sampler.add(img, cv::Point{ 317, 239 }, 5);
        sampler.add(img, cv::Point{ 323, 245 }, 5);
        spdlog::info("selftest: {} samples collected", sampler.count());

        const auto model = sampler.fit(3.0);
        if (!model.has_value()) {
            spdlog::error("selftest: fit failed");
            return 1;
        }
        spdlog::info("selftest: model mean a*{:+.1f} b*{:+.1f}, spread a*{:.2f} b*{:.2f}",
            model->mean_ab.x(), model->mean_ab.y(),
            std::sqrt(model->cov_ab(0, 0)), std::sqrt(model->cov_ab(1, 1)));

        pose::color_marker_detector detector;
        detector.set_model(*model);
        const std::vector<pose::marker_detection_t> found = detector.detect(img);

        spdlog::info("selftest: {} marker(s), rejected {}",
            found.size(), detector.reject_stats().total());
        if (found.size() != 1) {
            spdlog::error("selftest: expected exactly one marker");
            return 1;
        }

        const pose::marker_detection_t& m = found.front();
        const double err = std::hypot(m.center.x - kTrueX, m.center.y - kTrueY);
        spdlog::info("selftest: centre ({:.2f}, {:.2f}) vs truth ({:.2f}, {:.2f}), error {:.3f} px",
            m.center.x, m.center.y, kTrueX, kTrueY, err);
        spdlog::info("selftest: diameter {:.2f} px (drawn {:.1f}), fill {:.2f}, score {:.2f}",
            m.diameter_px, 2.0 * kTrueRadius, m.fill, m.score);

        // The weighted centroid lands within a few hundredths of a pixel on a clean synthetic disc.
        // A tenth of a pixel leaves margin while still failing if the subpixel path regresses.
        if (err > 0.1) {
            spdlog::error("selftest: centre error too large");
            return 1;
        }

        // A gray frame carries no color to classify, so the detector has to decline it.
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        if (!detector.detect(gray).empty()) {
            spdlog::error("selftest: a gray frame should yield nothing");
            return 1;
        }

        spdlog::info("selftest: ok");
        return 0;
    }

} // namespace

int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::debug);
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

    for (int i = 1; i < argc; ++i) {
        if (std::string_view{ argv[i] } == "--color-selftest") { return run_color_selftest(); }
    }

    vzcam_test_app app;
    return app.run();
}
