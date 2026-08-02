#pragma once
#include "hw/calibration.hh"
#include "hw/frame_format.hh"
#include "hw/timestamp.hh"

#include <opencv2/core.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace io
{
    // Message payloads of a recording: FlatBuffers-encoded Foxglove schemas
    // (see proto/foxglove/), so recordings decode in external tooling too.

    // How color frames are stored in a recording. The pixel layout is the other axis,
    // declared per camera stream (see `camera_stream_info_t::color_format`).
    enum class image_codec_t
    {
        jpeg, // lossy; re-encoded per frame
        raw,  // lossless pixels; the container's chunk compression does the shrinking
    };

    // One row per codec; a new codec is a row here plus a branch in encode_frame()/decode_frame().
    struct image_codec_desc_t
    {
        image_codec_t codec;
        std::string_view id; // stored in the channel metadata; selects the decoder on read
        std::string_view schema_name; // fully-qualified FlatBuffers root type
        bool chunk_compress; // false for payloads that are already compressed
    };

    inline constexpr std::array kImageCodecs{
        image_codec_desc_t{ .codec = image_codec_t::jpeg, .id = "jpeg", .schema_name = "foxglove.CompressedImage", .chunk_compress = false },
        image_codec_desc_t{ .codec = image_codec_t::raw,  .id = "raw",  .schema_name = "foxglove.RawImage",        .chunk_compress = true  },
    };

    // nullptr if the codec is unknown (e.g. a recording written by a newer build).
    const image_codec_desc_t* find_image_codec(image_codec_t codec) noexcept;
    const image_codec_desc_t* find_image_codec(std::string_view id) noexcept;

    inline constexpr std::string_view kCalibrationSchemaName{ "foxglove.CameraCalibration" };

    // A camera setting as metadata text; nullopt is spelled "auto".
    std::string encode_camera_setting(std::optional<double> value);
    std::optional<double> decode_camera_setting(std::string_view encoded) noexcept;

    // Binary schemas (.bfbs) embedded at build time, 
    // written into the recording so other tools can decode its messages.
    std::span<const uint8_t> image_schema_bytes(image_codec_t codec) noexcept;
    std::span<const uint8_t> calibration_schema_bytes() noexcept;

    struct encode_options_t
    {
        int jpeg_quality{ 90 }; // ignored by raw
    };

    // `coord_frame_id` names the camera's optical coordinate frame, which is what a viewer
    // places the image in. Pass the same name to encode_calibration(): matching names are
    // what pair a calibration with the images it describes.

    // NOTE: `image` must carry the pixel layout `format` names.
    //       Throws on a mismatch or an encoder failure.
    std::vector<std::byte> encode_frame(
        image_codec_t codec,
        const cv::Mat& image,
        hw::frame_format_t format,
        hw::timestamp_t timestamp,
        std::string_view coord_frame_id,
        const encode_options_t& options
    );

    // Inverse of encode_frame(), producing an image in `format`.
    // Returns an empty cv::Mat if the payload is malformed or stores another layout.
    cv::Mat decode_frame(
        image_codec_t codec,
        std::span<const std::byte> payload,
        hw::frame_format_t format
    ) noexcept;

    std::vector<std::byte> encode_calibration(
        const hw::calibration_t& calibration,
        hw::timestamp_t timestamp,
        std::string_view coord_frame_id
    );

    // Returns a zeroed calibration_t if the payload is malformed.
    hw::calibration_t decode_calibration(std::span<const std::byte> payload) noexcept;

} // namespace io
