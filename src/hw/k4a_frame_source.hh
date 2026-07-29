#pragma once
#include "sensor_frame_source.hh"

#include <k4a/k4a.h>

#include <mutex>
#include <optional>
#include <string>

namespace hw
{
    ////////////////////////////////////////////////////////////////////////////////////////
    // Orbbec K4A Wrapper backend.
    // (live capture only; recordings are read back through hw::mcap_record_player)
    ////////////////////////////////////////////////////////////////////////////////////////

    // Copy the K4A color camera parameters into the SDK-agnostic calibration_t. 
    // Shared with the offline mkv->mcap converter, which is the only reader of the K4A recording format.
    calibration_t k4a_to_calibration(const k4a_calibration_t& k4a_calib);

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
            const color_controls& controls = {}
        ) noexcept;

        bool is_valid() const override;
        void close() override;

        const calibration_t& get_calibration() const override { return _calib; }
        Eigen::Vector2i get_color_camera_resolution() const override { return _calib.color_resolution; }
        Eigen::Vector2f get_color_camera_fov() const override { return _calib.color_fov; }

        [[nodiscard]] std::unique_ptr<sensor_frameset> fetch_next_sensor_frameset() override;

    private:
        mutable std::mutex _mtx;
        k4a_device_t _device{ nullptr };
        k4a_device_configuration_t _config{ K4A_DEVICE_CONFIG_INIT_DISABLE_ALL };
        calibration_t _calib{};
        std::string _serialnum;
    };

} // namespace hw
