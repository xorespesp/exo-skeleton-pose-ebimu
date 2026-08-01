#include "mcap_record_player.hh"

#include <spdlog/spdlog.h>

#include <stdexcept>

namespace io
{
    namespace
    {
        std::chrono::microseconds to_us(std::chrono::nanoseconds ns) noexcept
        {
            return std::chrono::duration_cast<std::chrono::microseconds>(ns);
        }
    } // namespace

    mcap_record_player::~mcap_record_player()
    {
        this->close();
    }

    bool mcap_record_player::open(const std::filesystem::path& recording_file) noexcept try
    {
        std::scoped_lock lk{ _mtx };
        if (_opened) { throw std::runtime_error{ "mcap_record_player: already opened" }; }

        if (!_reader.open(recording_file)) {
            throw std::runtime_error{ "mcap_record_player: failed to open recording" };
        }

        const std::span<const recorded_camera_stream> streams = _reader.camera_streams();
        if (streams.size() > 1) {
            spdlog::warn("mcap_record_player: recording has {} camera streams, playing back '{}' only",
                streams.size(), streams.front().id
            );
        }

        _stream_id = streams.front().stream_id;
        _calib = streams.front().calibration;
        _first_ts = to_us(_reader.first_timestamp());
        _last_ts = to_us(_reader.last_timestamp());
        _opened = true;

        spdlog::info("recording playback ready (length: {} ms)",
            std::chrono::duration_cast<std::chrono::milliseconds>(_last_ts - _first_ts).count()
        );
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("mcap_record_player::open failed: {}", e.what());
        _reader.close();
        return false;
    }

    bool mcap_record_player::is_valid() const
    {
        std::scoped_lock lk{ _mtx };
        return _opened;
    }

    void mcap_record_player::close()
    {
        std::scoped_lock lk{ _mtx };
        if (_opened) {
            _reader.close();
            _opened = false;
        }
    }

    std::optional<hw::sensor_frameset> mcap_record_player::fetch_next_sensor_frameset()
    {
        std::scoped_lock lk{ _mtx };
        if (!_opened) { return std::nullopt; }

        std::optional<recording_reader::frame> frame = _reader.fetch_next_frame(_stream_id);

        if (!frame.has_value()) {
            if (!_auto_repeat) { return std::nullopt; } // EOF; the provider reports the stream end
            _reader.seek_timestamp(_stream_id, _reader.first_timestamp());
            frame = _reader.fetch_next_frame(_stream_id);
            if (!frame.has_value()) { return std::nullopt; }
        }

        cv::Mat image = std::move(frame->bgr);
        if (_color_roi.has_value()) {
            image = image(cv::Rect{ _color_roi->x, _color_roi->y, _color_roi->width, _color_roi->height });
        }

        return hw::sensor_frameset{ std::make_shared<hw::sensor_frame>(
            std::move(image),
            hw::frame_format_t::bgr8,
            to_us(frame->timestamp)
        ) };
    }

    std::optional<hw::roi_t> mcap_record_player::try_set_color_roi(const hw::roi_t& roi)
    {
        std::scoped_lock lk{ _mtx };

        const hw::roi_t clipped = hw::clamp_roi(roi,
            _calib.color_resolution.x(),
            _calib.color_resolution.y()
        );

        if (clipped.is_empty()) { return std::nullopt; }

        _color_roi = clipped;
        return _color_roi;
    }

    void mcap_record_player::seek_begin()
    {
        std::scoped_lock lk{ _mtx };
        if (_opened) { _reader.seek_timestamp(_stream_id, _reader.first_timestamp()); }
    }

    void mcap_record_player::seek_end()
    {
        std::scoped_lock lk{ _mtx };
        if (_opened) { _reader.seek_timestamp(_stream_id, _reader.last_timestamp()); }
    }

    void mcap_record_player::seek_timestamp(const std::chrono::microseconds offset)
    {
        std::scoped_lock lk{ _mtx };
        if (_opened) { _reader.seek_timestamp(_stream_id, std::chrono::duration_cast<std::chrono::nanoseconds>(offset)); }
    }

    bool mcap_record_player::auto_repeat_enabled() const
    {
        std::scoped_lock lk{ _mtx };
        return _auto_repeat;
    }

    void mcap_record_player::enable_auto_repeat(const bool enable)
    {
        std::scoped_lock lk{ _mtx };
        _auto_repeat = enable;
    }

} // namespace io
