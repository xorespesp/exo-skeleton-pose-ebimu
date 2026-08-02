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

        hw::frame_format_t get_color_format() const override { return _color_format; }

        // A recording holds whole frames, so this is a software crop.
        std::optional<hw::roi_t> try_set_color_roi(const hw::roi_t& roi) override;

        [[nodiscard]] std::optional<hw::sensor_frameset> fetch_next_sensor_frameset() override;

        std::chrono::nanoseconds get_recording_length() const override { return _last_timestamp - _first_timestamp; }
        hw::timestamp_t get_first_record_timestamp() const override { return _first_timestamp; }
        hw::timestamp_t get_last_record_timestamp() const override { return _last_timestamp; }

        void seek_begin() override;
        void seek_end() override;
        void seek_timestamp(hw::timestamp_t timestamp) override;

        bool auto_repeat_enabled() const override;
        void enable_auto_repeat(bool enable) override;

    private:
        mutable std::mutex _mtx;
        recording_reader _reader;
        bool _opened{ false };
        stream_id_t _stream_id{ 0 }; // the camera stream being played back
        hw::calibration_t _calib{};
        hw::frame_format_t _color_format{};
        std::optional<hw::roi_t> _color_roi; // nullopt: whole frames
        hw::timestamp_t _first_timestamp{};
        hw::timestamp_t _last_timestamp{};
        bool _auto_repeat{ false };
    };

} // namespace io
