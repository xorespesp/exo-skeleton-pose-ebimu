#pragma once
#include "timestamp.hh"

#include <chrono>
#include <optional>

namespace hw
{
    // Maps a source's own monotonic clock onto Unix time.
    //
    // The offset is sampled once, from the first reading handed in, and from then on only
    // added. Frame-to-frame spacing therefore comes out exactly as the source reported it;
    // reading the wall clock per frame would fold the host's scheduling jitter into every
    // interval instead.
    //
    // A source whose timestamps are already Unix does not need this.
    class clock_anchor_t final
    {
    public:
        // Takes a reading of the source's clock and returns that same instant in Unix time.
        // The first call fixes the mapping.
        timestamp_t to_unix(std::chrono::nanoseconds source_ts)
        {
            if (!_offset.has_value()) {
                const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch());
                _offset = now - source_ts;
            }
            return timestamp_t{ source_ts + *_offset };
        }

    private:
        std::optional<std::chrono::nanoseconds> _offset; // unix timestamp - source timestamp
    };

} // namespace hw
