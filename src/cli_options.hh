#pragma once
#include "app_config.hh"

#include <CLI/CLI.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace app
{
    // What the command line said, before it becomes an `app_config_t`.
    //
    // A config profile is the one place a setup is named. The rest say how this run is exposed and
    // watched, so they layer on top of whichever profile it was without contradicting one.
    struct cli_options_t
    {
        std::string config_name;      // --config; empty: nothing named on the command line
        std::optional<uint16_t> port; // -p; unset leaves the profile's `server.port` standing

        bool dump_config{ false }; // print the resolved config and exit
        int verbosity{ 0 };        // -v lowers the terminal to debug, -vv to trace
    };

    void add_cli_options(CLI::App& app, cli_options_t& o);

    // The config the run uses: the file `--config` named, or the built-in defaults where it named
    // none, with `--port` applied over either. A file is read only when one is named.
    // `out_file` names the file it came from and is empty for the built-in defaults.
    [[nodiscard]] bool resolve_config(
        const cli_options_t& o,
        app_config_t& out,
        std::filesystem::path& out_file,
        std::string& err
    );

} // namespace app
