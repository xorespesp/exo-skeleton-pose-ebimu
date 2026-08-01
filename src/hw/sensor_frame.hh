#pragma once
#include "frame_format.hh"

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <utility>

namespace hw
{
    // Single image handed to observers
    class sensor_frame final
    {
    public:
        sensor_frame(
            cv::Mat color_image,
            frame_format_t color_format,
            std::chrono::microseconds timestamp)
            // NOTE: `isSubmatrix()` is true for a view, false for a full matrix. The latter can be moved out; the former must be copied.
            : _color_image{ color_image.isSubmatrix() ? color_image.clone() : std::move(color_image) }
            , _color_format{ color_format }
            , _timestamp{ timestamp }
        { }

        sensor_frame(const sensor_frame&) = delete;
        sensor_frame& operator=(const sensor_frame&) = delete;
        
        // Unique ID for this frame, monotonically increasing with each new frame. (in whole process lifetime)
        uint64_t id() const noexcept { return _id; }

        const cv::Mat& color_image() const noexcept { return _color_image; }
        frame_format_t color_format() const noexcept { return _color_format; }

        std::chrono::microseconds timestamp() const noexcept { return _timestamp; } // device timestamp
        double timestamp_in_sec() const { return std::chrono::duration<double>{ _timestamp }.count(); }

    private:
        inline static std::atomic<uint64_t> s_next_id{ 1 };

        const uint64_t _id{ s_next_id.fetch_add(1, std::memory_order_relaxed) };
        const cv::Mat _color_image; // narrowed to the ROI when one is in force
        const frame_format_t _color_format;
        const std::chrono::microseconds _timestamp;
    };

} // namespace hw
