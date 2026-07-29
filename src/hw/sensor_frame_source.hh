#pragma once
#include "calibration.hh"
#include "sensor_frameset.hh"

#include <Eigen/Core>

#include <chrono>
#include <memory>

namespace hw
{
    // Abstract camera backend interface. SDK-agnostic.
    class sensor_frame_source {
    public:
        virtual ~sensor_frame_source() = default;

        virtual bool is_valid() const = 0;
        virtual void close() = 0;

        virtual const calibration_t& get_calibration() const = 0;
        virtual Eigen::Vector2i      get_color_camera_resolution() const = 0; // (width, height)
        virtual Eigen::Vector2f      get_color_camera_fov()        const = 0; // (h_fov, v_fov) [deg]

        // Blocking. Returns nullptr on EOF / timeout / disconnect.
        [[nodiscard]] virtual std::unique_ptr<sensor_frameset> fetch_next_sensor_frameset() = 0;
    };

    // Recording playback backends.
    class record_player_source : public sensor_frame_source {
    public:
        virtual std::chrono::microseconds get_recording_length() const = 0;
        virtual std::chrono::microseconds get_first_record_timestamp() const = 0;
        virtual std::chrono::microseconds get_last_record_timestamp() const = 0;

        virtual void seek_begin() = 0;
        virtual void seek_end() = 0;
        virtual void seek_timestamp(std::chrono::microseconds offset) = 0;

        virtual bool auto_repeat_enabled() const = 0;
        virtual void enable_auto_repeat(bool enable) = 0;
    };

} // namespace hw
