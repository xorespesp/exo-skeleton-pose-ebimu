#pragma once
#include "source_address.hh"

#include <CLI/CLI.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace app
{
    // Frame source cli options
    struct source_options
    {
        std::optional<source_address> source_addr; // unset: nothing to auto-open

        double tag_size_m{ 0.05 };
        std::optional<int32_t> exposure_us;
        std::optional<int32_t> gain;
    };

    void add_source_options(CLI::App& app, source_options& o);

} // namespace app
