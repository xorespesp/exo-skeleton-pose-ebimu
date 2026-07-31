#pragma once
#include "hw/sensor_frameset.hh"

#include <k4a/k4a.hpp>
#include <opencv2/core.hpp>

#include <chrono>

namespace hw
{
    // K4A backend frameset: wraps a k4a::capture and exposes color as BGR.
    class k4a_frameset final : public sensor_frameset {
    public:
        explicit k4a_frameset(k4a::capture capture)
            : _capture{ std::move(capture) }
        { }

        bool has_color_image() const override;
        std::chrono::microseconds get_color_timestamp() const override;
        cv::Mat get_color_image() const override; // BGR (CV_8UC3)

    private:
        k4a::capture _capture;
        mutable cv::Mat _color_bgr_cache; // cached BGRA/MJPG -> BGR conversion
    };

} // namespace hw
