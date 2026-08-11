#include "cli_options.hh"

#include <format>
#include <string>

namespace app
{
    void add_cli_options(CLI::App& app, cli_options_t& o)
    {
        app.add_option("-c,--config", o.config_name,
            "Installation config profile: <name>.json read from the configs folder, or a path\n"
            "when it carries a folder");

        // A callback fires only when the option is present, which is what lets an omitted one stay
        // omitted. Defaults live in `app_config_t` alone; a second copy here would always win.
        app.add_option_function<uint16_t>(
            "-p,--port",
            [&o](const uint16_t v) { o.port = v; },
            "WebSocket listen port, overriding what the config profile says");

        app.add_flag("--dump-config", o.dump_config,
            "Print the resolved config as JSON and exit");

        // Repeatable, so an int counts the occurrences.
        app.add_flag("-v,--verbose", o.verbosity,
            "Lower the terminal log level: -v for debug (protocol rx/tx trace), -vv for trace");
    }

    bool resolve_config(
        const cli_options_t& o,
        app_config_t& out,
        std::filesystem::path& out_file,
        std::string& err)
    {
        out_file.clear();

        // A config file is read only when one is named. Nothing named leaves the built-in defaults
        // standing, which is exactly what a bare run is.
        //
        // Those defaults leave `pose.detector` on the AprilTag path. A colour marker takes a
        // calibration measured on site, and naming its file is what a profile or the debugger's
        // open dialog does.
        if (o.config_name.empty())
        {
            out = app_config_t{};
        }
        else
        {
            const std::filesystem::path path = find_config_file(o.config_name);
            if (path.empty()) {
                err = std::format("config '{}' was not found", o.config_name);
                return false;
            }
            // Reads into a value of its own and assigns only once the whole file parsed and
            // validated, so a failure here leaves `out` as it was.
            if (!load_config(path, out, err)) {
                err = std::format("config '{}': {}", path.string(), err);
                return false;
            }
            out_file = path;
        }

        // The port says where this run listens rather than what it opens, so it lands on top of
        // the settings whichever of the two above they came from.
        if (o.port.has_value()) { out.server.port = *o.port; }

        return true;
    }

} // namespace app
