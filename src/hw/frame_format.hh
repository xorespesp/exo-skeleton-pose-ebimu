#pragma once
#include <cstdint>
#include <optional>
#include <string_view>

namespace hw
{
    enum class frame_format_t : uint8_t { 
        bgr8, 
        gray8 
    };

    constexpr std::string_view frame_format_to_str(frame_format_t format)
    {
        switch (format) {
        case frame_format_t::bgr8:  return "bgr8";
        case frame_format_t::gray8: return "gray8";
        }
        return "?";
    }
    
    constexpr std::optional<frame_format_t> frame_format_from_str(std::string_view str)
    {
        if (str == frame_format_to_str(frame_format_t::bgr8))  { return frame_format_t::bgr8; }
        if (str == frame_format_to_str(frame_format_t::gray8)) { return frame_format_t::gray8; }
        return std::nullopt;
    }

} // namespace hw
