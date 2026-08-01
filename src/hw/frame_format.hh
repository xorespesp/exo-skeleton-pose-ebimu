#pragma once
#include <cstdint>
#include <string_view>

namespace hw
{
    enum class frame_format_t : uint8_t { 
        bgr8, 
        gray8 
    };

    constexpr std::string_view frame_format_name(frame_format_t format)
    {
        switch (format) {
        case frame_format_t::bgr8:  return "bgr8";
        case frame_format_t::gray8: return "gray8";
        }
        return "?";
    }

} // namespace hw
