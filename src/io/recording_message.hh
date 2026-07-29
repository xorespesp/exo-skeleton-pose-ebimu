#pragma once
#include "hw/calibration.hh"

#include <opencv2/core.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace io
{
    // Message payloads of a recording: FlatBuffers-encoded Foxglove schemas
    // (see proto/foxglove/), so recordings decode in external tooling too.

    // How color frames are stored in a recording.
    enum class image_codec
    {
        jpeg,     // lossy; re-encoded per frame
        raw_bgr8, // lossless pixels; the container's chunk compression does the shrinking
    };

    // One row per codec; a new codec is a row here plus a branch in encode_frame()/decode_frame().
    struct image_codec_desc
    {
        image_codec      codec;
        std::string_view id;             // stored in the channel metadata; selects the decoder on read
        std::string_view schema_name;    // fully-qualified FlatBuffers root type
        bool             chunk_compress; // false for payloads that are already compressed
    };

    inline constexpr std::array kImageCodecs{
        image_codec_desc{ .codec = image_codec::jpeg,     .id = "jpeg",     .schema_name = "foxglove.CompressedImage", .chunk_compress = false },
        image_codec_desc{ .codec = image_codec::raw_bgr8, .id = "raw_bgr8", .schema_name = "foxglove.RawImage",        .chunk_compress = true  },
    };

    // nullptr if the codec is unknown (e.g. a recording written by a newer build).
    const image_codec_desc* find_image_codec(image_codec codec) noexcept;
    const image_codec_desc* find_image_codec(std::string_view id) noexcept;

    inline constexpr std::string_view kCalibrationSchemaName{ "foxglove.CameraCalibration" };

    // Binary schemas (.bfbs) embedded at build time, written into the recording so
    // other tools can decode its messages.
    std::span<const uint8_t> image_schema_bytes(image_codec codec) noexcept;
    std::span<const uint8_t> calibration_schema_bytes() noexcept;

    struct encode_options
    {
        int jpeg_quality{ 90 }; // ignored by raw_bgr8
    };

    // bgr is CV_8UC3. Throws on an unsupported image or an encoder failure.
    std::vector<std::byte> encode_frame(
        image_codec codec,
        const cv::Mat& bgr,
        std::chrono::nanoseconds timestamp,
        std::string_view frame_id,
        const encode_options& options
    );

    // Inverse of encode_frame(). Returns an empty cv::Mat if the payload is malformed.
    cv::Mat decode_frame(image_codec codec, std::span<const std::byte> payload) noexcept;

    std::vector<std::byte> encode_calibration(
        const hw::calibration_t& calibration,
        std::string_view frame_id
    );

    // Returns a zeroed calibration_t if the payload is malformed.
    hw::calibration_t decode_calibration(std::span<const std::byte> payload) noexcept;

} // namespace io
