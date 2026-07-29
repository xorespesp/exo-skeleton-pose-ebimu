#pragma once
#include "sensor_frame_source.hh"

#include "io/recording_reader.hh"

#include <chrono>
#include <filesystem>
#include <mutex>

namespace hw
{
    // Playback source for our own recordings. (SDK-agnostic)
    class mcap_record_player final : public record_player_source {
    public:
        mcap_record_player() = default;
        ~mcap_record_player() override;

        // Fails if the file is not one of our recordings, or carries no camera stream.
        [[nodiscard]] bool open(const std::filesystem::path& recording_file) noexcept;

        bool is_valid() const override;
        void close() override;

        const calibration_t& get_calibration() const override { return _calib; }
        Eigen::Vector2i get_color_camera_resolution() const override { return _calib.color_resolution; }
        Eigen::Vector2f get_color_camera_fov() const override { return _calib.color_fov; }

        [[nodiscard]] std::unique_ptr<sensor_frameset> fetch_next_sensor_frameset() override;

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
        io::recording_reader _reader;
        bool _opened{ false };
        io::stream_id_t _stream_id{ 0 }; // the camera stream being played back
        calibration_t _calib{};
        std::chrono::microseconds _first_ts{ 0 };
        std::chrono::microseconds _last_ts{ 0 };
        bool _auto_repeat{ false };
    };

} // namespace hw
