#pragma once
#include "app_config.hh"

#include "pose/joints_def.hh"
#include "pose/view_plane.hh"

#include <imgui.h> // imfilebrowser.h requires it first
#include <imfilebrowser.h>

#include <filesystem>
#include <optional>
#include <string>

namespace gui
{
    enum class source_kind_t { k4a_device, vz_device, recording };

    // The Open Source form: a view over `app_config_t` in both directions. `fill` reads the
    // settings in force into the fields, `render` writes them back on Open.
    class open_source_dialog
    {
    public:
        struct result_t
        {
            bool open_source{ false }; // the form is in the config; open a source with it
            // Picked under Load Config...; reading it over the settings is the owner's, not the form's.
            std::optional<std::filesystem::path> load_config;
        };

        open_source_dialog();

        // An Open writes the whole form back, so a form older than the config would undo whatever
        // moved it. The owner fills before opening the dialog, and again when the config changes
        // under an open one.
        void fill(const app::app_config_t& config);

        void show() { _show = true; }

        // Draws the dialog and the file browsers it owns. A recording with no file picked leaves
        // the dialog standing and raises nothing.
        result_t render(app::app_config_t& config);

    private:
        bool _apply(app::app_config_t& config) const; // false when there is nothing to open
        void _render_browsers(result_t& result);

    private:
        bool _show{ false };

        source_kind_t _kind{ source_kind_t::k4a_device };
        pose::view_plane_t _view_plane{ pose::view_plane_t::frontal };
        app::marker_kind_t _marker_kind{ app::marker_kind_t::apriltag };
        pose::joint_side_t _marked_leg{ pose::joint_side_t::left }; // colour markers only

        int _device{ 0 }; // index within whichever camera backend is selected
        bool _manual_exposure{ false };
        int _exposure{ 8000 };
        bool _manual_gain{ false };
        int _gain{ 0 };
        std::string _recording; // kept alongside `_device`, so switching kind discards neither
        std::string _intrinsics; // calibration file for a camera that reports none

        ImGui::FileBrowser _recording_browser;
        ImGui::FileBrowser _intrinsics_browser;
        ImGui::FileBrowser _config_load_browser; // picks an existing file, so no new-filename flag
    };

} // namespace gui
