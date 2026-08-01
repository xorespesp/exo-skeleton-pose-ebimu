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

        const std::map<std::string, hw::frame_format_t> kFrameFormats{
            { "bgr8",  hw::frame_format_t::bgr8  },
            { "gray8", hw::frame_format_t::gray8 },
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

        app.add_option("--color-format", o.color_format, "Frame pixel layout: bgr8 | gray8")
            ->transform(CLI::CheckedTransformer(kFrameFormats, CLI::ignore_case))
            ->default_str("bgr8");

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
