#pragma once
#include "frame_format.hh"
#include "timestamp.hh"

#include <opencv2/core.hpp>

#include <atomic>
#include <cstdint>
#include <utility>

namespace hw
{
    // Single image handed to observers
    class sensor_frame final
    {
    public:
        sensor_frame(
            cv::Mat image,
            frame_format_t format,
            timestamp_t timestamp)
            : _timestamp{ timestamp }
            // NOTE: `isSubmatrix()` is true for a view, false for a full matrix. The latter can be moved out; the former must be copied.
            , _image{ image.isSubmatrix() ? image.clone() : std::move(image) }
            , _format{ format }
        { }

        sensor_frame(const sensor_frame&) = delete;
        sensor_frame& operator=(const sensor_frame&) = delete;
        
        // Unique ID for this frame, monotonically increasing with each new frame. (in whole process lifetime)
        uint64_t id() const noexcept { return _id; }

        // The instant the source captured this frame.
        timestamp_t timestamp() const noexcept { return _timestamp; }

        const cv::Mat& image() const noexcept { return _image; }
        frame_format_t format() const noexcept { return _format; }
        
    private:
        inline static std::atomic<uint64_t> s_next_id{ 1 };

        const uint64_t _id{ s_next_id.fetch_add(1, std::memory_order_relaxed) };
        const timestamp_t _timestamp;
        const cv::Mat _image; // narrowed to the ROI when one is in force
        const frame_format_t _format;
    };

} // namespace hw
