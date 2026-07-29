#pragma once
#include "sensor_frameset.hh"

#include <opencv2/core.hpp>

#include <chrono>
#include <utility>

namespace hw
{
    // Playback frameset: one frame already decoded out of a recording.
    class mcap_frameset final : public sensor_frameset {
    public:
        mcap_frameset(cv::Mat color_bgr, std::chrono::microseconds timestamp)
            : _color_bgr{ std::move(color_bgr) }
            , _timestamp{ timestamp }
        { }

        bool has_color_image() const override { return !_color_bgr.empty(); }
        std::chrono::microseconds get_color_timestamp() const override { return _timestamp; }
        cv::Mat get_color_image() const override { return _color_bgr; } // BGR (CV_8UC3)

    private:
        cv::Mat _color_bgr;
        std::chrono::microseconds _timestamp;
    };

} // namespace hw
