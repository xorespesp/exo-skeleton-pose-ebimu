#include "recording_writer.hh"

#include <mcap/writer.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <format>
#include <stdexcept>

namespace io
{
    namespace
    {
        // Bumped when the layout of a recording changes in a way readers must know about.
        constexpr std::string_view kFormatVersion{ "1" };

        // FlatBuffers is the message encoding for every channel we write.
        constexpr std::string_view kMessageEncoding{ "flatbuffer" };

        // Chunks are the unit of both compression and the seek index: small enough that a
        // seek decompresses little, large enough that raw frames still compress well.
        constexpr uint64_t kChunkSize = 2 * 1024 * 1024;

        mcap::ByteArray to_byte_array(std::span<const uint8_t> bytes)
        {
            const auto* first = reinterpret_cast<const std::byte*>(bytes.data());
            return mcap::ByteArray{ first, first + bytes.size() };
        }

        std::string to_metadata_value(std::optional<int32_t> value)
        {
            return value.has_value() ? std::to_string(*value) : std::string{ "auto" };
        }

        uint64_t unix_now_ns()
        {
            return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        }
    } // namespace

    recording_writer::recording_writer(const recording_options& options)
        : _options{ options }
    { }

    recording_writer::~recording_writer()
    {
        this->close();
    }

    bool recording_writer::open(const std::filesystem::path& path) noexcept try
    {
        if (_writer) { throw std::runtime_error{ "recording_writer: already opened" }; }

        const image_codec_desc* codec = find_image_codec(_options.codec);
        if (!codec) { throw std::invalid_argument{ "recording_writer: unknown image codec" }; }

        mcap::McapWriterOptions options{ "" }; // no well-known profile; the schemas describe the messages
        options.library = "exo-skeleton-pose";
        options.chunkSize = kChunkSize;
        options.compression = codec->chunk_compress ? mcap::Compression::Zstd : mcap::Compression::None;
        options.compressionLevel = mcap::CompressionLevel::Fastest; // recording runs live; do not stall the encoder

        auto writer = std::make_unique<mcap::McapWriter>();
        if (const mcap::Status status = writer->open(path.string(), options); !status.ok()) {
            throw std::runtime_error{ std::format(
                "recording_writer: failed to open '{}': {}", path.string(), status.message) };
        }

        // The schemas travel with the recording, so anything that speaks MCAP can decode it.
        mcap::Schema image_schema{
            codec->schema_name, kMessageEncoding, to_byte_array(image_schema_bytes(_options.codec))
        };
        writer->addSchema(image_schema);

        mcap::Schema calibration_schema{
            kCalibrationSchemaName, kMessageEncoding, to_byte_array(calibration_schema_bytes())
        };
        writer->addSchema(calibration_schema);

        writer->write(mcap::Metadata{
            .name = "exo/recording",
            .metadata = {
                { "format_version", std::string{ kFormatVersion } },
                { "codec", std::string{ codec->id } },
                { "jpeg_quality", std::to_string(_options.encode.jpeg_quality) },
                { "started_unix_ns", std::to_string(unix_now_ns()) },
            },
        });

        _writer = std::move(writer);
        _path = path;
        _image_schema = image_schema.id;
        _calibration_schema = calibration_schema.id;
        _streams.clear();
        _frames_written.store(0, std::memory_order_relaxed);
        _payload_bytes.store(0, std::memory_order_relaxed);
        _has_frames.store(false, std::memory_order_relaxed);

        spdlog::info("recording opened: {} (codec: {})", path.string(), codec->id);
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("recording_writer::open failed: {}", e.what());
        return false;
    }

    std::optional<stream_id_t> recording_writer::add_camera_stream(const camera_stream_info& info) noexcept try
    {
        if (!_writer) { throw std::runtime_error{ "recording_writer: not opened" }; }
        if (info.id.empty()) { throw std::invalid_argument{ "recording_writer: camera stream needs an id" }; }

        const bool duplicate = std::ranges::any_of(_streams,
            [&info](const camera_stream& s) { return s.id == info.id; });
        if (duplicate) {
            throw std::invalid_argument{ std::format(
                "recording_writer: camera stream '{}' already registered", info.id) };
        }

        const image_codec_desc* codec = find_image_codec(_options.codec);
        if (!codec) { throw std::invalid_argument{ "recording_writer: unknown image codec" }; }

        // On the channel rather than only in the file metadata, so a reader picks a
        // decoder per stream.
        mcap::Channel image_channel{
            std::format("/camera/{}/image", info.id), kMessageEncoding, _image_schema,
            { { "codec", std::string{ codec->id } } }
        };
        _writer->addChannel(image_channel);

        mcap::Channel calibration_channel{
            std::format("/camera/{}/calibration", info.id), kMessageEncoding, _calibration_schema,
            {
                { "source_name", info.source_name },
                { "exposure_us", to_metadata_value(info.exposure_us) },
                { "gain", to_metadata_value(info.gain) },
            }
        };
        _writer->addChannel(calibration_channel);

        _streams.push_back(camera_stream{
            .id = info.id,
            .image_channel = image_channel.id,
            .calibration_channel = calibration_channel.id,
            .calibration_payload = encode_calibration(info.calibration, info.id),
        });

        spdlog::info("recording: camera stream '{}' registered ({}x{})",
            info.id, info.calibration.color_resolution.x(), info.calibration.color_resolution.y());

        return static_cast<stream_id_t>(_streams.size() - 1);
    }
    catch (const std::exception& e)
    {
        spdlog::error("recording_writer::add_camera_stream failed: {}", e.what());
        return std::nullopt;
    }

    bool recording_writer::write_frame(
        const stream_id_t stream_id,
        const cv::Mat& bgr,
        const std::chrono::nanoseconds device_timestamp) noexcept try
    {
        if (!_writer) { throw std::runtime_error{ "recording_writer: not opened" }; }
        if (stream_id >= _streams.size()) { throw std::invalid_argument{ "recording_writer: unknown camera stream" }; }

        camera_stream& s = _streams[stream_id];

        // The calibration rides the first frame's timestamp rather than time zero, so
        // the recording's time range stays the range of its frames.
        if (!s.calibration_written) {
            this->_write_message(s.calibration_channel, 0, device_timestamp, s.calibration_payload);
            s.calibration_written = true;
        }

        const std::vector<std::byte> payload = encode_frame(
            _options.codec, bgr, device_timestamp, s.id, _options.encode);
        this->_write_message(s.image_channel, s.sequence++, device_timestamp, payload);

        _frames_written.fetch_add(1, std::memory_order_relaxed);
        _payload_bytes.fetch_add(payload.size(), std::memory_order_relaxed);
        if (!_has_frames.load(std::memory_order_relaxed)) {
            _first_timestamp_ns.store(device_timestamp.count(), std::memory_order_relaxed);
            _has_frames.store(true, std::memory_order_relaxed);
        }
        _last_timestamp_ns.store(device_timestamp.count(), std::memory_order_relaxed);

        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("recording_writer::write_frame failed: {}", e.what());
        return false;
    }

    void recording_writer::_write_message(
        const uint16_t channel,
        const uint32_t sequence,
        const std::chrono::nanoseconds timestamp,
        const std::span<const std::byte> payload)
    {
        const auto time = static_cast<mcap::Timestamp>(timestamp.count());

        mcap::Message message{};
        message.channelId = channel;
        message.sequence = sequence;
        message.logTime = time;
        message.publishTime = time;
        message.dataSize = payload.size();
        message.data = payload.data();

        if (const mcap::Status status = _writer->write(message); !status.ok()) {
            throw std::runtime_error{ std::format("failed to write message: {}", status.message) };
        }
    }

    void recording_writer::close() noexcept
    {
        if (!_writer) { return; }

        _writer->close();
        _writer.reset();
        _streams.clear();

        const recording_stats s = this->stats();
        spdlog::info("recording closed: {} ({} frames, {:.1f} s, {:.1f} MB)",
            _path.string(),
            s.frames_written,
            std::chrono::duration<double>{ s.duration }.count(),
            static_cast<double>(s.file_bytes) / (1024.0 * 1024.0));
    }

    recording_stats recording_writer::stats() const noexcept
    {
        std::error_code ec;
        const uint64_t size = _path.empty() ? 0u : std::filesystem::file_size(_path, ec);

        const auto first = _first_timestamp_ns.load(std::memory_order_relaxed);
        const auto last = _last_timestamp_ns.load(std::memory_order_relaxed);
        const bool has_frames = _has_frames.load(std::memory_order_relaxed);

        return recording_stats{
            .frames_written = _frames_written.load(std::memory_order_relaxed),
            .payload_bytes = _payload_bytes.load(std::memory_order_relaxed),
            .file_bytes = ec ? 0u : size,
            .duration = has_frames ? std::chrono::nanoseconds{ last - first } : std::chrono::nanoseconds{ 0 },
        };
    }

} // namespace io
