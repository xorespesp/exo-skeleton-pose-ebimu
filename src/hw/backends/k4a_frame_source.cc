#include "k4a_frame_source.hh"

#include "k4a_frameset.hh"

#include <k4a/k4a.hpp>

#include <spdlog/spdlog.h>

#include <cmath>
#include <optional>
#include <stdexcept>

namespace hw
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;
    }

    // Copy the color camera parameters into the SDK-agnostic calibration_t.
    calibration_t k4a_to_calibration(const k4a_calibration_t& k4a_calib)
    {
        const k4a_calibration_camera_t& cc = k4a_calib.color_camera_calibration;
        const auto& p = cc.intrinsics.parameters.param; // {cx,cy,fx,fy,k1..k6,codx,cody,p2,p1,...}

        calibration_t out{};
        out.color_intr = intrinsic_t{
            p.fx, p.fy, p.cx, p.cy,
            cc.resolution_width, cc.resolution_height
        };
        out.color_dist = distortion_t{
            p.k1, p.k2, p.k3, p.k4, p.k5, p.k6, p.p1, p.p2
        };
        out.color_resolution = Eigen::Vector2i{ cc.resolution_width, cc.resolution_height };

        const float h_fov = (p.fx > 0.0f)
            ? static_cast<float>(2.0 * std::atan(cc.resolution_width / (2.0 * p.fx)) * 180.0 / kPi)
            : 0.0f;
        const float v_fov = (p.fy > 0.0f)
            ? static_cast<float>(2.0 * std::atan(cc.resolution_height / (2.0 * p.fy)) * 180.0 / kPi)
            : 0.0f;
        out.color_fov = Eigen::Vector2f{ h_fov, v_fov };

        return out;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // k4a_device_capturer
    ///////////////////////////////////////////////////////////////////////////////////////////////

    k4a_device_capturer::~k4a_device_capturer()
    {
        this->close();
    }

    bool k4a_device_capturer::open(
        const uint32_t device_index, 
        const color_controls& controls) noexcept try
    {
        std::scoped_lock lk{ _mtx };
        if (_device) { throw std::runtime_error{ "k4a_device_capturer: already opened" }; }

        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        config.depth_mode = K4A_DEPTH_MODE_OFF; // RGB only
        config.synchronized_images_only = false; // no depth to sync with

        k4a_device_t device = nullptr;
        if (K4A_FAILED(::k4a_device_open(device_index, &device))) {
            throw std::runtime_error{ "k4a_device_capturer: failed to open device" };
        }

        k4a_calibration_t k4a_calib{};
        if (K4A_FAILED(::k4a_device_get_calibration(
            device, config.depth_mode, config.color_resolution, &k4a_calib)))
        {
            ::k4a_device_close(device);
            throw std::runtime_error{ "k4a_device_capturer: failed to get calibration" };
        }

        if (K4A_FAILED(::k4a_device_start_cameras(device, &config))) {
            ::k4a_device_close(device);
            throw std::runtime_error{ "k4a_device_capturer: failed to start cameras" };
        }

        // Color controls (non-fatal if unsupported). The device retains previous
        // settings across opens while it stays connected, so nullopt must actively
        // reset the control to AUTO rather than leave a stale manual value in place.
        const auto apply_color_control = [device](k4a_color_control_command_t cmd,
                                                  std::optional<int32_t> value, const char* name)
        {
            const auto mode = value.has_value() ? K4A_COLOR_CONTROL_MODE_MANUAL : K4A_COLOR_CONTROL_MODE_AUTO;
            if (K4A_FAILED(::k4a_device_set_color_control(device, cmd, mode, value.value_or(0)))) {
                spdlog::warn("k4a: failed to set {} {}", name, value.has_value() ? "manual" : "auto");
            } else if (value.has_value()) {
                spdlog::info("k4a: manual {} {}", name, *value);
            } else {
                spdlog::info("k4a: auto {}", name);
            }
        };
        apply_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, controls.exposure_us, "exposure");
        apply_color_control(K4A_COLOR_CONTROL_GAIN, controls.gain, "gain");

        // serial number (for logging; non-fatal if it fails)
        std::string serialnum;
        {
            size_t needed = 0;
            if (::k4a_device_get_serialnum(device, nullptr, &needed) == K4A_BUFFER_RESULT_TOO_SMALL && needed > 1) {
                serialnum.resize(needed);
                if (::k4a_device_get_serialnum(device, &serialnum[0], &needed) == K4A_BUFFER_RESULT_SUCCEEDED
                    && !serialnum.empty() && serialnum.back() == '\0') {
                    // std::string expects there to not be as null terminator at the end of its data but
                    // k4a_device_get_serialnum adds a null terminator, so we drop the last character of the string after we
                    // get the result back.
                    serialnum.pop_back();
                }
            }
        }

        _device = device;
        _config = config;
        _calib = k4a_to_calibration(k4a_calib);
        _serialnum = std::move(serialnum);

        spdlog::info("k4a device opened (S/N: {})", _serialnum.empty() ? "<unknown>" : _serialnum);
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("k4a_device_capturer::open failed: {}", e.what());
        return false;
    }

    bool k4a_device_capturer::is_valid() const
    {
        std::scoped_lock lk{ _mtx };
        return _device != nullptr;
    }

    void k4a_device_capturer::close()
    {
        std::scoped_lock lk{ _mtx };
        if (_device) {
            ::k4a_device_stop_cameras(_device);
            ::k4a_device_close(_device);
            _device = nullptr;
        }
    }

    std::unique_ptr<sensor_frameset> k4a_device_capturer::fetch_next_sensor_frameset()
    {
        std::scoped_lock lk{ _mtx };
        if (!_device) { return nullptr; }

        k4a_capture_t capture_handle = nullptr;
        const k4a_wait_result_t wait_result = ::k4a_device_get_capture(
            _device, 
            &capture_handle, 
            1000 /* ms; finite so the polling thread can wake for join */
        );

        if (wait_result == K4A_WAIT_RESULT_FAILED) {
            throw std::runtime_error{ "k4a_device_capturer: failed to get capture" };
        }
        if (wait_result == K4A_WAIT_RESULT_TIMEOUT) {
            return nullptr; // provider retries
        }

        k4a::capture capture{ capture_handle };
        if (!capture.is_valid()) { return nullptr; }

        return std::make_unique<k4a_frameset>(std::move(capture));
    }

} // namespace hw
