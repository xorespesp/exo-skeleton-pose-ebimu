#pragma once
#include "recording_message.hh"
#include "recording_writer.hh" // stream_id_t

#include "hw/calibration.hh"
#include "hw/source_backend.hh"

#include <opencv2/core.hpp>

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace mcap { class McapReader; }

namespace io
{
    struct recorded_camera_stream_t
    {
        std::string stream_name; // e.g, "color0"
        stream_id_t stream_id{}; // identifies this stream to seek_timestamp()/fetch_next_frame()
        image_codec_t codec{};
        hw::frame_format_t color_format{}; // the layout fetch_next_frame() hands back
        hw::calibration_t calibration{};

        // What produced the frames, as the recording reports it. Empty where the recording
        // says nothing, says something this build cannot read, or the camera chose the value.
        std::optional<hw::source_backend_t> source_backend;
        std::string source_name;
        std::optional<double> exposure_us;
        std::optional<double> gain;
    };

    // Reads back what recording_writer produced. 
    // Knows the container, not the camera: swapping the capture backend does not touch this.
    //
    // NOTE: thread-unsafe.
    class recording_reader final
    {
    public:
        recording_reader();
        ~recording_reader();
        recording_reader(const recording_reader&) = delete;
        recording_reader& operator=(const recording_reader&) = delete;

        // Fails if the file is not a recording of ours, or has no camera stream.
        [[nodiscard]] bool open(const std::filesystem::path& path) noexcept;
        bool is_opened() const noexcept { return _opened; }
        void close() noexcept;

        std::span<const recorded_camera_stream_t> camera_streams() const noexcept { return _streams; }

        hw::timestamp_t first_timestamp() const noexcept { return _first_timestamp; }
        hw::timestamp_t last_timestamp() const noexcept { return _last_timestamp; }

        struct frame_t
        {
            hw::timestamp_t timestamp{};
            cv::Mat image; // in the stream's `color_format`
        };

        // Each stream advances and seeks on its own cursor, so reading or seeking one does
        // not disturb the others. `stream_id` is a handle from camera_streams().

        // Restarts the stream's iteration at the first frame at or after `timestamp`.
        void seek_timestamp(stream_id_t stream_id, hw::timestamp_t timestamp) noexcept;

        // nullopt at the end of the stream. Only this stream's frames are decoded.
        [[nodiscard]] std::optional<frame_t> fetch_next_frame(stream_id_t stream_id) noexcept;

    private:
        // Builds a fresh cursor for the stream, positioned at `from`. 
        // MCAP's iterator is forward-only and its start time is fixed when the view is created, 
        // so seeking is rebuilding the view; open and seek_timestamp both go through here.
        void _restart_cursor(stream_id_t stream_id, hw::timestamp_t from);

    private:
        // A message view paired with its iterator: the iterator points into the view, so the
        // two live and die together and a seek replaces both. One per stream.
        struct playback_cursor_t;

        std::unique_ptr<mcap::McapReader> _reader;
        std::vector<recorded_camera_stream_t> _streams;
        std::vector<std::unique_ptr<playback_cursor_t>> _cursors; // parallel to _streams

        hw::timestamp_t _first_timestamp{};
        hw::timestamp_t _last_timestamp{};
        bool _opened{ false };
    };

} // namespace io
