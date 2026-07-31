#include "k4a_frameset.hh"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <stdexcept>

namespace hw
{
    bool k4a_frameset::has_color_image() const
    {
        const k4a::image color = _capture.get_color_image();
        return color.is_valid() && color.get_size() > 0;
    }

    std::chrono::microseconds k4a_frameset::get_color_timestamp() const
    {
        const k4a::image color = _capture.get_color_image();
        if (!color.is_valid()) { return std::chrono::microseconds{ 0 }; }
        return color.get_device_timestamp();
    }

    cv::Mat k4a_frameset::get_color_image() const
    {
        if (!_color_bgr_cache.empty()) { return _color_bgr_cache; }

        const k4a::image color = _capture.get_color_image();
        if (!color.is_valid() || color.get_size() == 0) {
            throw std::runtime_error{ "k4a_frameset: color image is invalid" };
        }

        // Color image -> 8-bit BGR
        switch (color.get_format()) {
        case K4A_IMAGE_FORMAT_COLOR_BGRA32: {
            const cv::Mat bgra_view( // Zero-copy view over the k4a BGRA buffer
                color.get_height_pixels(),
                color.get_width_pixels(),
                CV_8UC4,
                const_cast<void*>(static_cast<const void*>(color.get_buffer())),
                static_cast<size_t>(color.get_stride_bytes())
            );
            cv::cvtColor(bgra_view, _color_bgr_cache, cv::COLOR_BGRA2BGR);
            break;
        }
        case K4A_IMAGE_FORMAT_COLOR_MJPG: {
            const cv::Mat jpeg_view( // Zero-copy view of the k4a JPEG buffer
                1,
                static_cast<int>(color.get_size()),
                CV_8UC1,
                const_cast<void*>(static_cast<const void*>(color.get_buffer()))
            );
            _color_bgr_cache = cv::imdecode(jpeg_view, cv::IMREAD_COLOR);
            break;
        }
        default:
            throw std::runtime_error{
                "sensor frame provider: unsupported color image format (only MJPG / BGRA32)"
            };
        }

        return _color_bgr_cache;
    }

} // namespace hw
