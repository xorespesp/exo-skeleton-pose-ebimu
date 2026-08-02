#include "frame_recorder.hh"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace io
{
    frame_recorder::frame_recorder(
        const recording_options_t& options, 
        const size_t queue_depth)
        : _queue_depth{ std::max<size_t>(1, queue_depth) }
        , _writer{ options }
    { }

    frame_recorder::~frame_recorder()
    {
        this->stop();
    }

    bool frame_recorder::start(
        const std::filesystem::path& path,
        const camera_stream_info_t& camera) noexcept try
    {
        if (_is_started.load(std::memory_order_relaxed)) {
            throw std::runtime_error{ "frame_recorder: already started" };
        }

        if (!_writer.open(path)) {
            throw std::runtime_error{ "frame_recorder: failed to open the recording" };
        }

        const std::optional<stream_id_t> new_stream = _writer.add_camera_stream(camera);
        if (!new_stream.has_value()) {
            _writer.close();
            throw std::runtime_error{ "frame_recorder: failed to register the camera stream" };
        }
        _stream_id = *new_stream;

        {
            std::scoped_lock lk{ _mtx };
            _queue.clear();
            _frames_dropped = 0;
        }

        _is_started.store(true, std::memory_order_relaxed);
        _thread = std::jthread{ [this](std::stop_token stop) { this->_worker(std::move(stop)); } };

        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("frame_recorder::start failed: {}", e.what());
        return false;
    }

    void frame_recorder::stop() noexcept
    {
        // Stop accepting frames first, so the worker sees a queue that can only shrink.
        if (!_is_started.exchange(false, std::memory_order_relaxed)) { return; }

        _thread.request_stop(); // the worker drains what is queued, then exits
        if (_thread.joinable()) { _thread.join(); }

        _writer.close();
    }

    void frame_recorder::_worker(std::stop_token stop)
    {
        for (;;) {
            std::shared_ptr<hw::sensor_frame> frame;
            {
                std::unique_lock lk{ _mtx };
                _queue_cv.wait(lk, stop, [this] { return !_queue.empty(); });

                // Empty here means the stop was requested and the queue is drained.
                if (_queue.empty()) { return; }

                frame = std::move(_queue.front());
                _queue.pop_front();
            }

            // Encoded outside the lock, or the observer callback would block behind it on push.
            [[maybe_unused]] const bool succeeded = _writer.write_frame(
                _stream_id,
                frame->color_image(),
                frame->timestamp()
            );
        }
    }

    void frame_recorder::on_sensor_frame_update(const std::shared_ptr<hw::sensor_frame>& new_sensor_frame)
    {
        if (!_is_started.load(std::memory_order_relaxed)) { return; }
        if (!new_sensor_frame || new_sensor_frame->color_image().empty()) { return; }

        {
            std::scoped_lock lk{ _mtx };
            if (_queue.size() >= _queue_depth) {
                // Encoding is behind. Dropping keeps memory bounded; the count makes the loss visible.
                ++_frames_dropped;
                return;
            }
            _queue.push_back(new_sensor_frame);
        }
        _queue_cv.notify_one();
    }

    void frame_recorder::on_sensor_stream_reset()
    {
        // A seek or a new source does not end a recording; the owner decides when to stop.
    }

    void frame_recorder::on_sensor_stream_end()
    {
        this->stop();
    }

    recording_stats_t frame_recorder::stats() const noexcept
    {
        recording_stats_t stats = _writer.stats(); // safe to read while the worker writes

        std::scoped_lock lk{ _mtx };
        stats.frames_dropped = _frames_dropped;
        return stats;
    }

} // namespace io
