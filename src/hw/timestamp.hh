#pragma once
#include <chrono>

namespace hw
{
    // Unix timestamp with nanosecond resolution.
    using timestamp_t = std::chrono::sys_time<std::chrono::nanoseconds>;

} // namespace hw
