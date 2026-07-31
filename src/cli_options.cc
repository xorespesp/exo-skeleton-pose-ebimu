#include "cli_options.hh"

#include <map>
#include <string>

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
        auto* device = app.add_option_function<uint32_t>(
            "-d,--device",
            [&o](const uint32_t index) { o.source_addr = source_address::device(index); },
            "Camera device index to open");

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
    }

} // namespace app
