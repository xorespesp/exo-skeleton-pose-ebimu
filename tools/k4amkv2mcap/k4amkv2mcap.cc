// Offline K4A .mkv recording -> our .mcap recording converter
#include "hw/backends/k4a_frame_source.hh" // hw::k4a_to_calibration, hw::k4a_color_to_mat
#include "hw/clock_anchor.hh"
#include "io/recording_writer.hh"

#include <k4a/k4a.hpp>
#include <k4arecord/playback.h>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace
{
    io::image_codec_t parse_codec(const std::string& name)
    {
        if (const io::image_codec_desc_t* desc = io::find_image_codec(name)) { return desc->codec; }
        throw std::invalid_argument{ "unknown codec '" + name + "' (expected jpeg or raw)" };
    }

    // RAII for the K4A playback handle: the loop below has several throwing exits.
    struct playback_handle_t
    {
        k4a_playback_t h{ nullptr };
        ~playback_handle_t() { if (h) { ::k4a_playback_close(h); } }
    };
} // namespace

int main(int argc, char** argv) try
{
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

    CLI::App app{ "Convert a K4A .mkv recording to our .mcap format" };

    std::string input_path;
    std::string output_path;
    std::string codec_name{ "jpeg" };
    int jpeg_quality{ 90 };

    app.add_option("input", input_path, "K4A .mkv recording to read")->required();
    app.add_option("-o,--output", output_path, "Output .mcap path (default: input with .mcap extension)");
    app.add_option("-c,--codec", codec_name, "Color codec: jpeg or raw")->default_val("jpeg");
    app.add_option("-q,--jpeg-quality", jpeg_quality, "JPEG quality 1-100 (ignored for raw)")
        ->default_val(90)->check(CLI::Range(1, 100));
    CLI11_PARSE(app, argc, argv);

    const std::filesystem::path input{ input_path };
    if (!std::filesystem::is_regular_file(input)) {
        spdlog::error("input is not a file: {}", input.string());
        return 1;
    }

    const std::filesystem::path output = output_path.empty()
        ? std::filesystem::path{ input }.replace_extension(".mcap")
        : std::filesystem::path{ output_path };

    const io::image_codec_t codec = parse_codec(codec_name);

    // --- open the K4A recording -----------------------------------------------------------
    playback_handle_t playback;
    if (K4A_FAILED(::k4a_playback_open(input.string().c_str(), &playback.h))) {
        spdlog::error("failed to open K4A recording: {}", input.string());
        return 1;
    }

    k4a_record_configuration_t record_config{};
    if (K4A_FAILED(::k4a_playback_get_record_configuration(playback.h, &record_config))
        || !record_config.color_track_enabled)
    {
        spdlog::error("recording has no color track: {}", input.string());
        return 1;
    }

    // Whatever the recording holds (MJPG, NV12, ...) is decoded to one format, 
    // so the conversion below has a single input layout to work from.
    if (K4A_FAILED(::k4a_playback_set_color_conversion(playback.h, K4A_IMAGE_FORMAT_COLOR_BGRA32))) {
        spdlog::error("failed to set color conversion on the recording");
        return 1;
    }

    k4a_calibration_t k4a_calib{};
    if (K4A_FAILED(::k4a_playback_get_calibration(playback.h, &k4a_calib))) {
        spdlog::error("failed to read calibration from the recording");
        return 1;
    }

    // --- open the output ------------------------------------------------------------------
    io::recording_writer writer{ io::recording_options_t{
        .codec = codec,
        .encode = { .jpeg_quality = jpeg_quality },
    } };
    if (!writer.open(output)) {
        spdlog::error("failed to open output: {}", output.string());
        return 1;
    }

    const auto stream = writer.add_camera_stream(io::camera_stream_info_t{
        .stream_name = "color0",
        .calibration = hw::k4a_to_calibration(k4a_calib),
        .color_format = hw::frame_format_t::bgr8,
        .source_backend = hw::source_backend_t::k4a,
        .source_name = input.filename().string(),
        // K4A recordings do not expose the per-capture exposure/gain through this path,
        // so they are left unset rather than guessed.
        .exposure_us = std::nullopt,
        .gain = std::nullopt,
    });
    if (!stream.has_value()) {
        spdlog::error("failed to register the camera stream");
        return 1;
    }

    spdlog::info("converting '{}' -> '{}' (codec: {})", input.string(), output.string(), codec_name);

    // --- transcode ------------------------------------------------------------------------
    // A .mkv carries device timestamps and no wall clock, and the playback API exposes no
    // capture date, so the frames are anchored at the moment of conversion. Their spacing is
    // exact; their absolute times say when this file was made, matching `create_timestamp`.
    hw::clock_anchor_t clock_anchor;

    uint64_t frames = 0;
    uint64_t skipped = 0;
    for (;;) {
        k4a_capture_t handle = nullptr;
        const k4a_stream_result_t result = ::k4a_playback_get_next_capture(playback.h, &handle);
        if (result == K4A_STREAM_RESULT_EOF) { break; }
        if (result != K4A_STREAM_RESULT_SUCCEEDED) {
            spdlog::error("failed to read capture after {} frames", frames);
            return 1;
        }

        const k4a::capture capture{ handle };
        const k4a::image color = capture.get_color_image();
        if (!color.is_valid() || color.get_size() == 0) { ++skipped; continue; }

        // Whole frames, in the layout the stream was registered with.
        const cv::Mat bgr = hw::k4a_color_to_mat(color, hw::frame_format_t::bgr8, std::nullopt);
        const auto ts = clock_anchor.to_unix(color.get_device_timestamp());
        if (!writer.write_frame(*stream, bgr, ts)) {
            spdlog::error("failed to write frame {}", frames);
            return 1;
        }

        if (++frames % 100 == 0) { spdlog::info("  {} frames...", frames); }
    }

    writer.close();

    const io::recording_stats_t stats = writer.stats();
    spdlog::info("done: {} frames written ({} frames without color skipped), {:.1f} s, {:.1f} MB",
        stats.frames_written, skipped,
        std::chrono::duration<double>{ stats.duration }.count(),
        static_cast<double>(stats.file_bytes) / (1024.0 * 1024.0));

    if (stats.frames_written == 0) {
        spdlog::warn("no frames were written; the recording may not contain color images");
        return 1;
    }
    return 0;
}
catch (const std::exception& e)
{
    spdlog::error("mkv2mcap failed: {}", e.what());
    return 1;
}
