#pragma once
#include "hw/backends/vz_device.hh"
#include "hw/clock_anchor.hh"
#include "hw/sensor_frame_source.hh"
#include "hw/source_config.hh"

#include <mutex>
#include <optional>

namespace hw
{
    ////////////////////////////////////////////////////////////////////////////////////////
    // Vieworks VZ camera backend
    ////////////////////////////////////////////////////////////////////////////////////////
    class vz_frame_source final : public sensor_frame_source
    {
    public:
        vz_frame_source() = default;
        ~vz_frame_source() override;

        // Configures the camera and leaves it idle; acquisition begins with the first fetch.
        [[nodiscard]] bool open(const vz_device_config_t& config) noexcept;

        bool is_valid() const override;
        void close() override;

        // The whole sensor, whatever readout window is programmed. Narrowing is the caller's
        // to account for, since it is the one that knows the window it asked for.
        const calibration_t& get_calibration() const override { return _calib; }
        frame_format_t get_frame_format() const override { return _frame_format; }

        // Programmed into the camera's readout window, so frames arrive already narrowed and
        // the transport carries only those pixels. GenICam snaps each field to its own
        // increment, so the window that comes back is rarely the one asked for.
        std::optional<roi_t> try_set_roi(const roi_t& roi) override;

        [[nodiscard]] std::optional<sensor_frameset> fetch_next_sensor_frameset() override;

    private:
        mutable std::mutex _mtx;
        vz::device _device;
        calibration_t _calib{};
        frame_format_t _frame_format{ frame_format_t::gray8 };
        clock_anchor_t _clock_anchor; // maps the camera's capture clock onto Unix time
    };

} // namespace hw
