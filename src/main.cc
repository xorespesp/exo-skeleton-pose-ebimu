#include "app_config.hh"
#include "cli_options.hh"
#include "gui/debugger_app.hh"
#include "net/exo_pose_server.hh"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    spdlog::set_default_logger(spdlog::stderr_color_mt("exo"));

    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    spdlog::flush_on(spdlog::level::info);

    // The logger passes every severity; each sink decides what it keeps. The terminal takes
    // info and up, while the debugger's in-GUI console attaches a sink that records everything
    // (its severity toggles filter the view, so a record must be there to be revealed later).
    spdlog::set_level(spdlog::level::trace);
    for (auto& sink : spdlog::default_logger()->sinks()) { sink->set_level(spdlog::level::info); }

    CLI::App cli{ "exo-skeleton-pose" };

    // Bare invocation launches the debugger GUI, which embeds a WebSocket server the operator starts/stops from the Server menu.
    app::cli_options_t gui_cli;
    app::add_cli_options(cli, gui_cli);

    // `serve` launches a headless WebSocket pose server instead. 
    // Its flags land in a struct of their own, so the mode that ran is the one whose flags are read.
    app::cli_options_t serve_cli;
    CLI::App* serve = cli.add_subcommand("serve", "Run the headless WebSocket pose server");
    app::add_cli_options(*serve, serve_cli);

    CLI11_PARSE(cli, argc, argv);

    const app::cli_options_t& parsed = serve->parsed() ? serve_cli : gui_cli;

    // The terminal is the only sink standing here; the debugger's console attaches its own later.
    if (parsed.verbosity > 0)
    {
        const auto level = parsed.verbosity >= 2 ? spdlog::level::trace : spdlog::level::debug;
        for (auto& sink : spdlog::default_logger()->sinks()) { sink->set_level(level); }
        spdlog::flush_on(level);
    }

    app::app_config_t config;
    std::filesystem::path config_file; // empty on the built-in defaults
    if (std::string err; !app::resolve_config(parsed, config, config_file, err)) {
        spdlog::error("{}", err);
        return -1;
    }

    // Plain stdout, so the output stays machine-readable rather than carrying log decoration.
    if (parsed.dump_config) {
        std::cout << app::dump_config(config) << '\n';
        return 0;
    }

    spdlog::info("config: {}", config_file.empty() ? "built-in defaults" : config_file.string());

    try
    {
        if (serve->parsed()) {
            return net::exo_pose_server{ config }.run();
        }

        // Debugger GUI
        return gui::debugger_app{ config }.run();
    }
    catch (const std::exception& e)
    {
        spdlog::error("fatal: {}", e.what());
        return -1;
    }
}
