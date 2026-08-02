#pragma once
#include <cstdint>
#include <optional>
#include <string_view>

namespace hw
{
    // Which kind of backend a source is driven by.
    // Split from the configs so a consumer that only needs to name the backend
    // (a log line, a recording's provenance) does not pull them in.
    enum class source_backend_t : uint8_t {
        k4a,
        vz,
        recording
    };

    constexpr std::string_view source_backend_to_str(source_backend_t backend)
    {
        switch (backend) {
        case source_backend_t::k4a:       return "k4a";
        case source_backend_t::vz:        return "vz";
        case source_backend_t::recording: return "recording";
        }
        return "?";
    }

    // nullopt if `str` is not one of the backends above.
    constexpr std::optional<source_backend_t> source_backend_from_str(std::string_view str)
    {
        if (str == source_backend_to_str(source_backend_t::k4a))       { return source_backend_t::k4a; }
        if (str == source_backend_to_str(source_backend_t::vz))        { return source_backend_t::vz; }
        if (str == source_backend_to_str(source_backend_t::recording)) { return source_backend_t::recording; }
        return std::nullopt;
    }

} // namespace hw
