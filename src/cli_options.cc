#include "cli_options.hh"

#include <format>
#include <string>
#include <vector>

namespace app
{
    namespace
    {
        // Option-group headings, which is what splits the two families in `--help`.
        constexpr const char* kConfigGroup = "Config mode";
        constexpr const char* kDirectGroup = "Direct mode";

    } // namespace

    void add_cli_options(CLI::App& app, cli_options_t& o)
    {
        auto* config = app.add_option("-c,--config", o.config_name,
            "Installation config: <name>.json read from the configs folder, or a path when it\n"
            "carries a folder. Names the marker kind; the options below always run the AprilTag one")
            ->group(kConfigGroup);

        // Callbacks fire only when the option is present, which is what lets an omitted one stay
        // omitted. Defaults live in `app_config_t` alone; a second copy here would always win.
        auto* device = app.add_option_function<std::string>(
            "-d,--device",
            [&o](const std::string& text) {
                const std::optional<source_address> addr = source_address::try_parse(text);
                // A path parses fine but belongs to --input, so it is refused here rather than
                // quietly opening a recording from the camera option.
                if (!addr.has_value() || addr->is_recording()) {
                    throw CLI::ValidationError("--device", "expected k4a:<index> or vz:<index>");
                }
                o.source = *addr;
            },
            "Camera to open: k4a:<index> | vz:<index>")
            ->group(kDirectGroup);

        auto* input = app.add_option_function<std::string>(
            "-i,--input",
            [&o](const std::string& path) { o.source = source_address::recording(path); },
            "MCAP recording file path to open")
            ->group(kDirectGroup);

        // A run opens one source; accepting both would silently drop one.
        device->excludes(input);
        input->excludes(device);

        auto* plane = app.add_option_function<std::string>(
            "--view-plane",
            [&o](const std::string& name) {
                const auto value = pose::view_plane_from_name(name);
                if (!value.has_value()) {
                    throw CLI::ValidationError("--view-plane", "expected frontal or sagittal");
                }
                o.view_plane = *value;
            },
            "Camera viewing plane: frontal | sagittal")
            ->group(kDirectGroup);

        auto* tag_size = app.add_option_function<double>(
            "-s,--tag-size",
            [&o](const double v) {
                if (v <= 0.0) { throw CLI::ValidationError("--tag-size", "must be greater than zero"); }
                o.tag_size_m = v;
            },
            "AprilTag black-square edge length [m]")
            ->group(kDirectGroup);

        auto* exposure = app.add_option_function<int32_t>(
            "-e,--exposure-us",
            [&o](const int32_t v) { o.exposure_us = v; },
            "Manual color exposure [us] (default: auto)")
            ->group(kDirectGroup);

        auto* gain = app.add_option_function<int32_t>(
            "-g,--gain",
            [&o](const int32_t v) { o.gain = v; },
            "Manual color gain (default: auto)")
            ->group(kDirectGroup);

        // Four values rather than a parsed string, so CLI11 reports a malformed rectangle
        // rather than this code having to.
        auto* roi = app.add_option_function<std::vector<int>>(
            "--roi",
            [&o](const std::vector<int>& v) { o.roi = hw::roi_t{ v[0], v[1], v[2], v[3] }; },
            "Restrict frames to a region of the full frame: X Y WIDTH HEIGHT")
            ->expected(4)
            ->type_name("X Y W H")
            ->group(kDirectGroup);

        auto* port = app.add_option_function<uint16_t>(
            "-p,--port",
            [&o](const uint16_t v) { o.port = v; },
            "WebSocket listen port")
            ->group(kDirectGroup);

        // A config file already says all of the above, so mixing the two would raise the
        // question of which one won.
        for (auto* direct : { device, input, plane, tag_size, exposure, gain, roi, port }) {
            config->excludes(direct);
            direct->excludes(config);
        }

        app.add_flag("--dump-config", o.dump_config,
            "Print the resolved config as JSON and exit");
    }

    bool resolve_config(
        const cli_options_t& o,
        app_config_t& out,
        std::filesystem::path& out_file,
        std::string& err)
    {
        out_file.clear();

        // A config file is read only when one is named. Nothing named leaves the built-in defaults
        // standing, which the direct options then adjust; where those are absent too, a bare run is
        // exactly the defaults.
        //
        // Those defaults leave `pose.detector` on the AprilTag path. A colour marker takes a
        // calibration measured on site, and naming its file is what a profile or the debugger's
        // open dialog does.
        if (o.config_name.empty())
        {
            app_config_t cfg;
            if (o.source.has_value())     { cfg.camera.source = *o.source; }
            if (o.view_plane.has_value()) { cfg.pose.view_plane = *o.view_plane; }
            if (o.exposure_us.has_value()){ cfg.camera.exposure_us = *o.exposure_us; }
            if (o.gain.has_value())       { cfg.camera.gain = *o.gain; }
            if (o.roi.has_value())        { cfg.camera.roi = *o.roi; }
            if (o.tag_size_m.has_value()) { cfg.pose.tag_size_m = *o.tag_size_m; }
            if (o.port.has_value())       { cfg.server.port = *o.port; }
            out = std::move(cfg);
            return true;
        }

        const std::filesystem::path path = find_config_file(o.config_name);
        if (path.empty()) {
            err = std::format("config '{}' was not found", o.config_name);
            return false;
        }
        if (!load_config(path, out, err)) {
            err = std::format("config '{}': {}", path.string(), err);
            return false;
        }
        out_file = path;
        return true;
    }

} // namespace app
