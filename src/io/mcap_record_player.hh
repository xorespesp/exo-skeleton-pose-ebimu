#pragma once
#include "recording_reader.hh"

#include "hw/sensor_frame_source.hh"

#include <chrono>
#include <filesystem>
#include <mutex>

namespace io
{
    // Playback source for our own recordings. (SDK-agnostic)
    class mcap_record_player final : public hw::record_player_source {
    public:
        mcap_record_player() = default;
        ~mcap_record_player() override;

        // Fails if the file is not one of our recordings, or carries no camera stream.
        [[nodiscard]] bool open(const std::filesystem::path& recording_file) noexcept;

        bool is_valid() const override;
        void close() override;

        const hw::calibration_t& get_calibration() const override { return _calib; }

        // Recordings carry 8-bit BGR, so there is nothing here to request.
        hw::frame_format_t get_color_format() const override { return hw::frame_format_t::bgr8; }

        // A recording holds whole frames, so this is a software crop.
        std::optional<hw::roi_t> try_set_color_roi(const hw::roi_t& roi) override;

        [[nodiscard]] std::optional<hw::sensor_frameset> fetch_next_sensor_frameset() override;

        std::chrono::microseconds get_recording_length() const override { return _last_ts - _first_ts; }
        std::chrono::microseconds get_first_record_timestamp() const override { return _first_ts; }
        std::chrono::microseconds get_last_record_timestamp() const override { return _last_ts; }

        void seek_begin() override;
        void seek_end() override;
        void seek_timestamp(std::chrono::microseconds offset) override;

        bool auto_repeat_enabled() const override;
        void enable_auto_repeat(bool enable) override;

    private:
        mutable std::mutex _mtx;
        recording_reader _reader;
        bool _opened{ false };
        stream_id_t _stream_id{ 0 }; // the camera stream being played back
        hw::calibration_t _calib{};
        std::optional<hw::roi_t> _color_roi; // nullopt: whole frames
        std::chrono::microseconds _first_ts{ 0 };
        std::chrono::microseconds _last_ts{ 0 };
        bool _auto_repeat{ false };
    };

} // namespace io
