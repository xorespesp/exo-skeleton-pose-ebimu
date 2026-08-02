#pragma once
#include "hw/clock_anchor.hh"
#include "hw/sensor_frame_source.hh"

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <opencv2/core.hpp>

#include <mutex>
#include <optional>
#include <string>

namespace hw
{
    ////////////////////////////////////////////////////////////////////////////////////////
    // Orbbec K4A Wrapper backend.
    // (live capture only; recordings are read back through io::mcap_record_player)
    ////////////////////////////////////////////////////////////////////////////////////////

    // Copy the K4A color camera parameters into the SDK-agnostic calibration_t.
    // Shared with the offline mkv->mcap converter, which is the only reader of the K4A recording format.
    calibration_t k4a_to_calibration(const k4a_calibration_t& k4a_calib);

    // Decode a K4A colour image into `format`, restricted to `roi` when one is given.
    // (in full-frame pixels, already fitted to the image)
    // The result owns its pixels.
    //
    // Narrowing happens before the conversion wherever the source layout allows it, 
    // so the conversion only reads and writes the pixels that will be delivered.
    cv::Mat k4a_color_to_mat(
        const k4a::image& color,
        frame_format_t format,
        std::optional<roi_t> roi
    );

    // Live camera source.
    class k4a_device_capturer final : public sensor_frame_source {
    public:
        // Manual color controls; nullopt leaves the camera on auto.
        struct color_controls {
            std::optional<int32_t> exposure_us; // manual exposure time [us]
            std::optional<int32_t> gain; // manual gain
        };

        k4a_device_capturer() = default;
        ~k4a_device_capturer() override;

        [[nodiscard]] bool open(
            uint32_t device_index,
            const color_controls& controls = {},
            frame_format_t color_format = frame_format_t::bgr8 // what delivered frames carry
        ) noexcept;

        bool is_valid() const override;
        void close() override;

        const calibration_t& get_calibration() const override { return _calib; }
        frame_format_t get_color_format() const override { return _color_format; }

        // NOTE: The k4a device supports no hardware ROI, so this is a software crop.
        std::optional<roi_t> try_set_color_roi(const roi_t& roi) override;

        [[nodiscard]] std::optional<sensor_frameset> fetch_next_sensor_frameset() override;

    private:
        mutable std::mutex _mtx;
        k4a_device_t _device{ nullptr };
        k4a_device_configuration_t _config{ K4A_DEVICE_CONFIG_INIT_DISABLE_ALL };
        calibration_t _calib{};
        std::string _serialnum;
        frame_format_t _color_format{ frame_format_t::bgr8 };
        std::optional<roi_t> _color_roi; // nullopt: whole frames
        clock_anchor_t _clock_anchor;    // maps the device clock onto Unix time
    };

} // namespace hw
