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
    // The two ways of naming a setup are mutually exclusive and the parser enforces it, which
    // is what keeps a precedence rule from ever being needed.
    struct cli_options_t
    {
        // Config mode
        std::string config_name; // --config; empty: nothing named on the command line

        // Direct mode. Each unset field leaves the built-in default standing.
        std::optional<source_address> source;
        std::optional<pose::view_plane_t> view_plane;
        std::optional<double> tag_size_m;
        std::optional<int32_t> exposure_us;
        std::optional<int32_t> gain;
        std::optional<hw::roi_t> roi;
        std::optional<uint16_t> port;

        bool dump_config{ false }; // print the resolved config and exit

        // Whether anything in the direct group was given.
        [[nodiscard]] bool has_direct_options() const;
    };

    void add_cli_options(CLI::App& app, cli_options_t& o);

    // The config the run uses: the named file (or the default profile, where nothing else was said) or the built-in defaults with the direct options applied. 
    // `out_file` names the file it came from and is empty for the built-in defaults.
    [[nodiscard]] bool resolve_config(
        const cli_options_t& o,
        app_config_t& out,
        std::filesystem::path& out_file,
        std::string& err
    );

} // namespace app
