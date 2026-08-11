#pragma once
#include "recording_message.hh"

#include "hw/calibration.hh"
#include "hw/source_backend.hh"

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

    struct camera_stream_info_t
    {
        // Labels this stream within the recording: it forms the topic paths and doubles as
        // the camera's coordinate frame name.
        std::string stream_name{ "color0" };
        hw::calibration_t calibration{};
        hw::frame_format_t color_format{ hw::frame_format_t::bgr8 }; // the layout every frame of this stream carries
        hw::source_backend_t source_backend{};
        std::string source_name{};
        std::optional<double> exposure_us{}; // nullopt = the camera was left on auto
        std::optional<double> gain{};
    };

    struct recording_options_t
    {
        // TODO: per-stream codecs, for when a recording carries more than one camera. Two things
        //       follow this one file-wide and both have to give: the image schema `open()`
        //       registers, and the chunk compression the codec's `chunk_compress` decides.
        image_codec_t codec{ image_codec_t::jpeg };
        encode_options_t encode{};
    };

    struct recording_stats_t
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
        explicit recording_writer(const recording_options_t& options = {});
        ~recording_writer();
        recording_writer(const recording_writer&) = delete;
        recording_writer& operator=(const recording_writer&) = delete;

        [[nodiscard]] bool open(const std::filesystem::path& path) noexcept;
        [[nodiscard]] bool is_opened() const noexcept { return _writer != nullptr; }
        void close() noexcept; // idempotent; stats() stays readable afterwards

        // Registers a new camera stream. Call before write_frame().
        [[nodiscard]] std::optional<stream_id_t> add_camera_stream(const camera_stream_info_t& info) noexcept;

        // Write a frame to the given stream.
        // Fails if the image is not in the layout the stream was registered with.
        [[nodiscard]] bool write_frame(
            stream_id_t stream_id,
            const cv::Mat& image,
            hw::timestamp_t timestamp // the recording's time axis; seeking and playback pacing run on it
        ) noexcept;

        // Thread-safe
        [[nodiscard]] recording_stats_t stats() const noexcept;

        [[nodiscard]] const std::filesystem::path& path() const noexcept { return _path; }

    private:
        void _write_mcap_message(
            uint16_t channel_id,
            uint32_t msg_sequence,
            hw::timestamp_t timestamp,
            std::span<const std::byte> msg_payload
        );

    private:
        struct camera_stream_t {
            std::string stream_name;
            hw::frame_format_t color_format{};
            uint16_t image_channel_id{};
            uint16_t calibration_channel_id{};
            hw::calibration_t calibration{};
            bool calibration_written{ false };
            uint32_t next_image_sequence{ 0 };
        };

        recording_options_t _options;
        std::unique_ptr<mcap::McapWriter> _writer;
        std::filesystem::path _path;
        std::vector<camera_stream_t> _streams;

        uint16_t _image_schema_id{ 0 };
        uint16_t _calibration_schema_id{ 0 };

        // Written by the writing thread, read by stats() from anywhere.
        std::atomic<uint64_t> _frames_written{ 0 };
        std::atomic<uint64_t> _payload_bytes{ 0 };
        std::atomic<hw::timestamp_t> _first_timestamp{};
        std::atomic<hw::timestamp_t> _last_timestamp{};
        std::atomic_bool _has_frames{ false };
    };

} // namespace io
