#include "recording_reader.hh"

#include <mcap/reader.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <format>
#include <stdexcept>

namespace io
{
    namespace
    {
        constexpr std::string_view kImageTopicSuffix{ "/image" };
        constexpr std::string_view kCameraTopicPrefix{ "/camera/" };

        // The file-level metadata record every recording of ours carries.
        constexpr std::string_view kRecordingMetadataName{ "exo/recording" };

        std::string image_topic(std::string_view stream_name)
        {
            return std::format("{}{}{}", kCameraTopicPrefix, stream_name, kImageTopicSuffix);
        }

        // "/camera/color0/image" -> "color0"; empty if the topic is not one of ours.
        std::string_view stream_name_from_image_topic(std::string_view topic) noexcept
        {
            if (!topic.starts_with(kCameraTopicPrefix) || !topic.ends_with(kImageTopicSuffix)) {
                return {};
            }
            topic.remove_prefix(kCameraTopicPrefix.size());
            topic.remove_suffix(kImageTopicSuffix.size());
            return topic;
        }

        // Empty when the channel does not carry the key.
        std::string_view channel_metadata(const mcap::Channel& channel, const std::string& key) noexcept
        {
            const auto it = channel.metadata.find(key);
            return (it != channel.metadata.end()) ? std::string_view{ it->second } : std::string_view{};
        }

        void log_problem(const mcap::Status& status)
        {
            spdlog::warn("recording_reader: {}", status.message);
        }

        std::span<const std::byte> payload_of(const mcap::Message& message) noexcept
        {
            return { message.data, static_cast<size_t>(message.dataSize) };
        }

        // The file-level "exo/recording" metadata, empty when the recording carries none.
        // Reached through the index readSummary() builds, which gives the record's byte range.
        mcap::KeyValueMap read_recording_metadata(mcap::McapReader& reader) noexcept try
        {
            mcap::IReadable* source = reader.dataSource();
            if (!source) { return {}; }

            const auto& indexes = reader.metadataIndexes();
            const auto it = indexes.find(std::string{ kRecordingMetadataName });
            if (it == indexes.end()) { return {}; }

            mcap::RecordReader records{
                *source, it->second.offset, it->second.offset + it->second.length
            };
            const std::optional<mcap::Record> record = records.next();
            if (!record.has_value() || record->opcode != mcap::OpCode::Metadata) { return {}; }

            mcap::Metadata metadata;
            if (const mcap::Status status = mcap::McapReader::ParseMetadata(*record, &metadata);
                !status.ok())
            {
                spdlog::warn("recording_reader: could not parse the file metadata: {}", status.message);
                return {};
            }
            return metadata.metadata;
        }
        catch (const std::exception& e)
        {
            spdlog::warn("recording_reader: could not read the file metadata: {}", e.what());
            return {};
        }
    } // namespace

    // The iterator holds a reference to the view, so neither may outlive the other and
    // the view must not move once the iterator exists. Bundling them enforces that.
    struct recording_reader::playback_cursor_t
    {
        mcap::LinearMessageView view;
        mcap::LinearMessageView::Iterator it;
        mcap::LinearMessageView::Iterator end;

        playback_cursor_t(
            mcap::McapReader& reader, 
            const mcap::ReadMessageOptions& options)
            : view{ reader.readMessages(log_problem, options) }
            , it{ view.begin() }
            , end{ view.end() }
        { }
    };

    recording_reader::recording_reader() = default;

    recording_reader::~recording_reader()
    {
        this->close();
    }

    bool recording_reader::open(const std::filesystem::path& path) noexcept try
    {
        if (_opened) { throw std::runtime_error{ "recording_reader: already opened" }; }

        if (!std::filesystem::is_regular_file(path)) {
            throw std::invalid_argument{ "recording_reader: invalid recording file path" };
        }

        auto reader = std::make_unique<mcap::McapReader>();
        if (const mcap::Status status = reader->open(path.string()); !status.ok()) {
            throw std::runtime_error{ std::format("failed to open '{}': {}", path.string(), status.message) };
        }

        // The summary carries the chunk index, the statistics and the metadata index, all of
        // which are read below. Without it a seek would have to rescan the file; the fallback
        // scan rebuilds the same information for recordings cut short by a crash.
        if (const mcap::Status status = reader->readSummary(
            mcap::ReadSummaryMethod::AllowFallbackScan, log_problem); !status.ok())
        {
            throw std::runtime_error{ std::format("failed to read summary: {}", status.message) };
        }

        const std::optional<mcap::Statistics> statistics = reader->statistics();
        if (!statistics.has_value()) {
            throw std::runtime_error{ "recording has no statistics record" };
        }

        const mcap::KeyValueMap file_metadata = read_recording_metadata(*reader);
        spdlog::debug("recording_reader: '{}' file metadata ({} entries)",
            path.filename().string(), file_metadata.size());
        for (const auto& [key, value] : file_metadata) {
            spdlog::debug("  {} = {}", key, value);
        }

        // Every camera stream in the file, discovered from its channels. Nothing here
        // knows how many cameras there are supposed to be.
        std::vector<recorded_camera_stream_t> streams;
        for (const auto& [channel_id, channel] : reader->channels()) {
            const std::string_view stream_name = stream_name_from_image_topic(channel->topic);
            if (stream_name.empty()) { continue; }

            const auto codec_it = channel->metadata.find("codec");
            if (codec_it == channel->metadata.end()) {
                spdlog::warn("recording_reader: channel '{}' has no codec; skipping", channel->topic);
                continue;
            }

            const image_codec_desc_t* codec = find_image_codec(codec_it->second);
            if (!codec) {
                spdlog::warn("recording_reader: channel '{}' uses unknown codec '{}'; skipping",
                    channel->topic, codec_it->second);
                continue;
            }

            const auto format_it = channel->metadata.find("color_format");
            if (format_it == channel->metadata.end()) {
                spdlog::warn("recording_reader: channel '{}' has no color format; skipping", channel->topic);
                continue;
            }

            const std::optional<hw::frame_format_t> color_format = hw::frame_format_from_str(format_it->second);
            if (!color_format.has_value()) {
                spdlog::warn("recording_reader: channel '{}' uses unknown color format '{}'; skipping",
                    channel->topic, format_it->second);
                continue;
            }

            // Provenance is for diagnosis, not decoding, so a stream stays usable
            // even when this build cannot make sense of the backend it names.
            std::optional<hw::source_backend_t> source_backend;
            if (const std::string_view backend = channel_metadata(*channel, "source_backend");
                !backend.empty())
            {
                source_backend = hw::source_backend_from_str(backend);
                if (!source_backend.has_value()) {
                    spdlog::warn("recording_reader: channel '{}' names an unknown source backend '{}'"
                        , channel->topic
                        , backend
                    );
                }
            }

            streams.push_back(recorded_camera_stream_t{
                .stream_name = std::string{ stream_name },
                .stream_id = static_cast<stream_id_t>(streams.size()),
                .codec = codec->codec,
                .color_format = *color_format,
                .source_backend = source_backend,
                .source_name = std::string{ channel_metadata(*channel, "source_name") },
                .exposure_us = decode_camera_setting(channel_metadata(*channel, "exposure_us")),
                .gain = decode_camera_setting(channel_metadata(*channel, "gain")),
            });
        }

        if (streams.empty()) {
            throw std::runtime_error{ "recording has no camera stream" };
        }

        // Each stream's calibration is a single message on its own topic, read once here so
        // the playback cursors carry only frames.
        for (recorded_camera_stream_t& stream : streams) {
            const std::string topic = std::format("/camera/{}/calibration", stream.stream_name);

            mcap::ReadMessageOptions options{};
            options.topicFilter = [&topic](const std::string_view candidate) { return candidate == topic; };

            mcap::LinearMessageView view = reader->readMessages(log_problem, options);
            const auto it = view.begin();
            if (it == view.end()) {
                spdlog::warn("recording_reader: camera stream '{}' has no calibration", stream.stream_name);
                continue;
            }
            stream.calibration = decode_calibration(payload_of(it->message));
        }

        _reader = std::move(reader);
        _streams = std::move(streams);
        _first_timestamp = hw::timestamp_t{ std::chrono::nanoseconds{ statistics->messageStartTime } };
        _last_timestamp = hw::timestamp_t{ std::chrono::nanoseconds{ statistics->messageEndTime } };
        _opened = true;

        _cursors.resize(_streams.size());
        for (const recorded_camera_stream_t& stream : _streams) {
            this->_restart_cursor(stream.stream_id, _first_timestamp);
        }

        spdlog::info("recording opened: {} ({} camera stream(s), {:.1f} s)",
            path.string(),
            _streams.size(),
            std::chrono::duration<double>{ _last_timestamp - _first_timestamp }.count()
        );
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("recording_reader::open failed: {}", e.what());
        return false;
    }

    void recording_reader::close() noexcept
    {
        _cursors.clear(); // point into the reader; must go first
        if (_reader) {
            _reader->close();
            _reader.reset();
        }
        _streams.clear();
        _opened = false;
    }

    void recording_reader::seek_timestamp(
        const stream_id_t stream_id,
        const hw::timestamp_t timestamp) noexcept try
    {
        if (!_opened || stream_id >= _cursors.size()) { return; }

        const auto upper = std::max(_first_timestamp, _last_timestamp);
        this->_restart_cursor(stream_id, std::clamp(timestamp, _first_timestamp, upper));
    }
    catch (const std::exception& e)
    {
        spdlog::error("recording_reader::seek_timestamp failed: {}", e.what());
        if (stream_id < _cursors.size()) { _cursors[stream_id].reset(); }
    }

    std::optional<recording_reader::frame_t> recording_reader::fetch_next_frame(
        const stream_id_t stream_id) noexcept try
    {
        if (!_opened || stream_id >= _cursors.size()) { return std::nullopt; }

        playback_cursor_t* cursor = _cursors[stream_id].get();
        if (!cursor) { return std::nullopt; }

        const recorded_camera_stream_t& cam = _streams[stream_id]; // parallel to _cursors; index already checked

        while (cursor->it != cursor->end) {
            const mcap::MessageView& view = *cursor->it;
            const auto timestamp = hw::timestamp_t{ std::chrono::nanoseconds{ view.message.logTime } };

            // The payload dies when the iterator advances, so decode before stepping.
            cv::Mat image = decode_frame(cam.codec, payload_of(view.message), cam.color_format);
            ++cursor->it;

            if (image.empty()) { continue; } // already logged; skip the bad frame rather than end playback

            return frame_t{
                .timestamp = timestamp,
                .image = std::move(image) 
            };
        }

        return std::nullopt;
    }
    catch (const std::exception& e)
    {
        spdlog::error("recording_reader::fetch_next_frame failed: {}", e.what());
        return std::nullopt;
    }

    void recording_reader::_restart_cursor(
        const stream_id_t stream_id,
        const hw::timestamp_t from)
    {
        _cursors[stream_id].reset();

        // startTime is baked into the view here and cannot be changed afterwards
        // (the iterator only moves forward), so a new position means a new view.
        mcap::ReadMessageOptions options{};
        options.startTime = static_cast<mcap::Timestamp>(std::max<int64_t>(0, from.time_since_epoch().count()));
        // Filter to this stream's image topic, so the cursor never touches another stream's
        // messages and only this stream's frames reach fetch_next_frame().
        options.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;
        options.topicFilter = [topic = image_topic(_streams[stream_id].stream_name)](const std::string_view t) {
            return t == topic;
        };

        _cursors[stream_id] = std::make_unique<playback_cursor_t>(*_reader, options);
    }

} // namespace io
