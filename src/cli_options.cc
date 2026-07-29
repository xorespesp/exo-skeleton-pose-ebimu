#include "cli_options.hh"

#include <string>

namespace app
{
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

        app.add_option("-s,--tag-size", o.tag_size_m, "AprilTag black-square edge length [m]")->default_val(0.05);
        app.add_option("-e,--exposure-us", o.exposure_us, "Manual color exposure [us] (default: auto)");
        app.add_option("-g,--gain", o.gain, "Manual color gain (default: auto)");
    }

} // namespace app
