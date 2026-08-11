#include "open_source_dialog.hh"

#include <imgui.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <optional>

namespace gui
{
    open_source_dialog::open_source_dialog()
    {
        _recording_browser.SetTitle("Open recording file");
        _recording_browser.SetTypeFilters({ ".mcap" });
        _recording_browser.SetPwd(app::project_dir("recordings"));

        _intrinsics_browser.SetTitle("Open camera calibration");
        _intrinsics_browser.SetTypeFilters({ ".yml", ".yaml", ".xml" });
        _intrinsics_browser.SetPwd(app::project_dir("configs"));

        _config_load_browser.SetTitle("Load config");
        _config_load_browser.SetTypeFilters({ ".json" });
    }

    void open_source_dialog::fill(const app::app_config_t& config)
    {
        const app::camera_config_t& cam = config.camera;

        _manual_exposure = cam.exposure_us.has_value();
        if (cam.exposure_us.has_value()) { _exposure = *cam.exposure_us; }
        _manual_gain = cam.gain.has_value();
        if (cam.gain.has_value()) { _gain = *cam.gain; }

        _view_plane = config.pose.view_plane;
        _marker_kind = config.pose.detector.kind;
        _marked_leg = config.pose.detector.color_marker.assigner.leg;
        _intrinsics = cam.intrinsics_file;

        // Only the field the config named, so the other keeps what was typed into it.
        if (cam.source.has_value())
        {
            if (cam.source->is_k4a_device()) {
                _device = static_cast<int>(cam.source->k4a_device_index());
                _kind = source_kind_t::k4a_device;
            } else if (cam.source->is_vz_device()) {
                _device = static_cast<int>(cam.source->vz_device_index());
                _kind = source_kind_t::vz_device;
            } else {
                _recording = cam.source->recording_path().string();
                _kind = source_kind_t::recording;
            }
        }
    }

    bool open_source_dialog::_apply(app::app_config_t& config) const
    {
        if (_kind == source_kind_t::recording)
        {
            if (_recording.empty()) { spdlog::warn("no recording file selected"); return false; }
            config.camera.source = app::source_address::recording(_recording);
        }
        else
        {
            // A recording carries the settings it was shot with, so only a camera takes these.
            config.camera.exposure_us = _manual_exposure ? std::optional<int32_t>{ _exposure } : std::nullopt;
            config.camera.gain = _manual_gain ? std::optional<int32_t>{ _gain } : std::nullopt;
            config.camera.intrinsics_file = _intrinsics;

            const auto index = static_cast<uint32_t>(_device);
            config.camera.source = (_kind == source_kind_t::vz_device)
                ? app::source_address::vz_device(index)
                : app::source_address::k4a_device(index);
        }

        config.pose.view_plane = _view_plane;
        config.pose.detector.kind = _marker_kind;
        config.pose.detector.color_marker.assigner.leg = _marked_leg;
        return true;
    }

    open_source_dialog::result_t open_source_dialog::render(app::app_config_t& config)
    {
        result_t result;
        if (!_show)
        {
            this->_render_browsers(result); // a pick made before the dialog was dismissed still lands
            return result;
        }

        ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_Appearing);
        if (ImGui::Begin("Open Source", &_show, ImGuiWindowFlags_NoCollapse))
        {
            if (ImGui::Button("Load Config..."))
            {
                _config_load_browser.SetPwd(app::project_dir("configs"));
                _config_load_browser.Open();
            }
            ImGui::SetItemTooltip(
                "Reads a saved config over the current settings and fills this dialog from it.\n"
                "An open source is reopened with it at once; with none open it takes effect on\n"
                "Open. Either way it replaces what the control panel has been tuned to since the\n"
                "last save.");

            ImGui::Separator();

            // The marker kind and the viewing plane decide which detector and estimator run, and
            // swapping either mid-stream would invalidate the rest pose, so both are picked here.
            const auto marker_kind_radio = [this](const char* label, app::marker_kind_t val) {
                if (ImGui::RadioButton(label, _marker_kind == val)) { _marker_kind = val; }
            };
            ImGui::TextUnformatted("Markers");
            marker_kind_radio("AprilTag", app::marker_kind_t::apriltag);
            ImGui::SameLine();
            marker_kind_radio("Color", app::marker_kind_t::color_marker);
            ImGui::SetItemTooltip("AprilTag: each tag states its own id, so a detection names its joint.\n"
                                  "Color:    plain discs, named by their order down the leg. The camera\n"
                                  "          streams colour, and the colour itself is measured on site.");

            const bool color_markers = (_marker_kind == app::marker_kind_t::color_marker);
            if (color_markers)
            {
                // A disc solves no distance and states no side, so colour settles both here.
                // `midline` is not offered as a leg: the config refuses it for this field.
                _view_plane = pose::view_plane_t::sagittal;

                const auto leg_radio = [this](const char* label, pose::joint_side_t side) {
                    if (ImGui::RadioButton(label, _marked_leg == side)) { _marked_leg = side; }
                };
                ImGui::TextUnformatted("Marked leg");
                ImGui::SameLine(); leg_radio("Left", pose::joint_side_t::left);
                ImGui::SameLine(); leg_radio("Right", pose::joint_side_t::right);
                ImGui::SetItemTooltip("Which leg carries the markers. The discs are named down the\n"
                                      "chain from it, and the side decides which way the angles swing.");
            }

            const auto view_plane_radio = [this](const char* label, pose::view_plane_t val) {
                if (ImGui::RadioButton(label, _view_plane == val)) { _view_plane = val; }
            };
            ImGui::TextUnformatted("Viewing plane");
            ImGui::BeginDisabled(color_markers);
            view_plane_radio("Frontal", pose::view_plane_t::frontal);
            ImGui::SameLine();
            view_plane_radio("Sagittal", pose::view_plane_t::sagittal);
            ImGui::EndDisabled();
            ImGui::SetItemTooltip("Frontal: camera faces the exo; both legs tagged, rig solved in 3D.\n"
                                  "Sagittal: camera at the side; only the near leg is marked and its\n"
                                  "          angles are read off the image plane (no tag pose solve).");

            ImGui::Separator();

            const auto kind_radio = [this](const char* label, source_kind_t val) {
                if (ImGui::RadioButton(label, _kind == val)) { _kind = val; }
            };
            kind_radio("K4A camera", source_kind_t::k4a_device);
            ImGui::SameLine();
            kind_radio("VZ camera", source_kind_t::vz_device);
            ImGui::SameLine();
            kind_radio("Recording", source_kind_t::recording);
            ImGui::Separator();

            if (_kind == source_kind_t::recording)
            {
                if (ImGui::Button("Browse...")) { _recording_browser.Open(); }
                ImGui::SameLine();
                ImGui::TextUnformatted(_recording.empty() ? "(no file selected)" : _recording.c_str());
            }
            else
            {
                // Both backends name a camera by its position in their own enumeration.
                ImGui::InputInt("Device index", &_device);
                if (_device < 0) { _device = 0; }

                // A fitted colour sits at one brightness and the open refuses a camera free to
                // leave it, so only the choice of auto goes away. The values stay editable.
                if (color_markers) {
                    _manual_exposure = true;
                    _manual_gain = true;
                }

                ImGui::BeginDisabled(color_markers);
                ImGui::Checkbox("Manual exposure [us]", &_manual_exposure);
                ImGui::EndDisabled();
                if (_manual_exposure)
                {
                    ImGui::SameLine();
                    ImGui::InputInt("##exposure", &_exposure);
                }

                ImGui::BeginDisabled(color_markers);
                ImGui::Checkbox("Manual gain", &_manual_gain);
                ImGui::EndDisabled();
                if (_manual_gain)
                {
                    ImGui::SameLine();
                    ImGui::InputInt("##gain", &_gain);
                    ImGui::SetItemTooltip("K4A: raw gain. VZ: gain in dB.");
                }

                if (color_markers) {
                    ImGui::TextDisabled("Color markers are measured at one brightness, so both are fixed.");
                }

                if (_kind == source_kind_t::vz_device)
                {
                    ImGui::TextUnformatted("Calibration");
                    if (ImGui::Button("Browse...##intr")) { _intrinsics_browser.Open(); }
                    ImGui::SameLine();
                    if (ImGui::Button("Clear##intr")) { _intrinsics.clear(); }
                    ImGui::SameLine();
                    ImGui::TextWrapped("%s", _intrinsics.empty()
                        ? "(none: tag poses will not be solved)"
                        : _intrinsics.c_str());
                    ImGui::SetItemTooltip(
                        "OpenCV FileStorage (.yml/.xml) from a chessboard calibration.\n"
                        "Must have been measured at the camera's own frame size.");
                }
            }

            ImGui::Separator();
            if (ImGui::Button("Open", ImVec2(90, 0)) && this->_apply(config))
            {
                result.open_source = true;
                _show = false;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(90, 0))) { _show = false; }
        }
        ImGui::End();

        this->_render_browsers(result);
        return result;
    }

    void open_source_dialog::_render_browsers(result_t& result)
    {
        _config_load_browser.Display();
        if (_config_load_browser.HasSelected())
        {
            result.load_config = _config_load_browser.GetSelected();
            _config_load_browser.ClearSelected();
        }

        _recording_browser.Display();
        if (_recording_browser.HasSelected())
        {
            _recording = _recording_browser.GetSelected().string();
            _kind = source_kind_t::recording;
            _recording_browser.ClearSelected();
        }

        _intrinsics_browser.Display();
        if (_intrinsics_browser.HasSelected())
        {
            _intrinsics = _intrinsics_browser.GetSelected().string();
            _intrinsics_browser.ClearSelected();
        }
    }

} // namespace gui
