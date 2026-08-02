#include "cli_options.hh"

#include <map>
#include <string>
#include <vector>

namespace app
{
    namespace
    {
        const std::map<std::string, pose::view_plane_t> kViewPlanes{
            { "frontal",  pose::view_plane_t::frontal  },
            { "sagittal", pose::view_plane_t::sagittal },
        };

    } // namespace

    void add_source_options(CLI::App& app, source_options& o)
    {
        auto* device = app.add_option_function<std::string>(
            "-d,--device",
            [&o](const std::string& text) {
                const std::optional<source_address> addr = source_address::try_parse(text);
                // A path parses fine but belongs to --input, so it is refused here rather than
                // quietly opening a recording from the camera option.
                if (!addr.has_value() || addr->is_recording()) {
                    throw CLI::ValidationError(
                        "--device", "expected k4a:<index> or vz:<index>");
                }
                o.source_addr = *addr;
            },
            "Camera to open: k4a:<index> | vz:<index>");

        auto* input = app.add_option_function<std::string>(
            "-i,--input",
            [&o](const std::string& path) { o.source_addr = source_address::recording(path); },
            "MCAP recording file path to open");

        // A run opens one source; accepting both would silently drop one.
        device->excludes(input);
        input->excludes(device);

        // Fixed for the process: a camera does not move between the two views mid-run.
        app.add_option("--view-plane", o.view_plane, "Camera viewing plane: frontal | sagittal")
            ->transform(CLI::CheckedTransformer(kViewPlanes, CLI::ignore_case))
            ->default_str("frontal");

        app.add_option("-s,--tag-size", o.tag_size_m, "AprilTag black-square edge length [m]")->default_val(0.05);
        app.add_option("-e,--exposure-us", o.exposure_us, "Manual color exposure [us] (default: auto)");
        app.add_option("-g,--gain", o.gain, "Manual color gain (default: auto)");

        // Four values rather than a parsed string, so CLI11 reports a malformed rectangle
        // rather than this code having to.
        app.add_option_function<std::vector<int>>(
            "--roi",
            [&o](const std::vector<int>& v) { o.color_roi = hw::roi_t{ v[0], v[1], v[2], v[3] }; },
            "Restrict frames to a region of the full frame: X Y WIDTH HEIGHT")
            ->expected(4)
            ->type_name("X Y W H");
    }

} // namespace app
