#pragma once
#include "recording_message.hh"

#include "hw/calibration.hh"

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace mcap { class McapWriter; }

namespace io
{
    // Handle for a camera stream within a recording.
    using stream_id_t = uint16_t;

    struct camera_stream_info
    {
        std::string id{ "color0" };
        hw::calibration_t calibration{};
        std::optional<int32_t> exposure_us{}; // nullopt = the camera was left on auto
        std::optional<int32_t> gain{};
        std::string source_name{};
    };

    struct recording_options
    {
        image_codec codec{ image_codec::jpeg };
        encode_options encode{};
    };

    struct recording_stats
    {
        uint64_t frames_written{};
        uint64_t frames_dropped{}; // filled in by the queue in front of the writer; the writer never drops
        uint64_t payload_bytes{};  // encoded message bytes, before chunk compression
        uint64_t file_bytes{};     // size on disk
        std::chrono::nanoseconds duration{};
    };

    // Writes camera streams into an MCAP recording.
    // NOTE: thread-unsafe except stats(), which may be read while another thread writes frames.
    class recording_writer final
    {
    public:
        explicit recording_writer(const recording_options& options = {});
        ~recording_writer();
        recording_writer(const recording_writer&) = delete;
        recording_writer& operator=(const recording_writer&) = delete;

        [[nodiscard]] bool open(const std::filesystem::path& path) noexcept;
        [[nodiscard]] bool is_opened() const noexcept { return _writer != nullptr; }
        void close() noexcept; // idempotent; stats() stays readable afterwards

        // Registers a new camera stream. Call before write_frame().
        [[nodiscard]] std::optional<stream_id_t> add_camera_stream(const camera_stream_info& info) noexcept;

        // Write a frame to the given stream.
        [[nodiscard]] bool write_frame(
            stream_id_t stream_id,
            const cv::Mat& bgr, // CV_8UC3
            std::chrono::nanoseconds device_timestamp // the recording's time axis; seeking and playback pacing run on it
        ) noexcept;

        // Thread-safe
        [[nodiscard]] recording_stats stats() const noexcept;

        [[nodiscard]] const std::filesystem::path& path() const noexcept { return _path; }

    private:
        void _write_message(
            uint16_t channel,
            uint32_t sequence,
            std::chrono::nanoseconds timestamp,
            std::span<const std::byte> payload
        );

    private:
        struct camera_stream {
            std::string id;
            uint16_t image_channel{};
            uint16_t calibration_channel{};
            std::vector<std::byte> calibration_payload;
            bool calibration_written{ false };
            uint32_t sequence{ 0 };
        };

        recording_options                _options;
        std::unique_ptr<mcap::McapWriter> _writer;
        std::filesystem::path            _path;
        std::vector<camera_stream>       _streams;

        uint16_t _image_schema{ 0 };
        uint16_t _calibration_schema{ 0 };

        // Written by the writing thread, read by stats() from anywhere.
        std::atomic<uint64_t> _frames_written{ 0 };
        std::atomic<uint64_t> _payload_bytes{ 0 };
        std::atomic<int64_t>  _first_timestamp_ns{ 0 };
        std::atomic<int64_t>  _last_timestamp_ns{ 0 };
        std::atomic_bool      _has_frames{ false };
    };

} // namespace io
