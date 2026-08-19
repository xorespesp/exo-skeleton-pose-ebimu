#pragma once
#include "recording_writer.hh"

#include "hw/sensor_frame_observer.hh"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <thread>

namespace io
{
    // Writes observed sensor frames to a recording.
    //
    // Encoding a 1080p frame costs several milliseconds, enough to starve a 30 fps stream,
    // so the observer callback only queues and a worker thread does the encoding.
    //
    // The queue is bounded: when encoding falls behind, 
    // frames are dropped and counted in stats() rather than buffered until memory runs out.
    class frame_recorder final : public hw::sensor_frame_observer
    {
    public:
        static constexpr size_t kDefaultQueueDepth = 8;

        explicit frame_recorder(const recording_options_t& options = {}, size_t queue_depth = kDefaultQueueDepth);
        ~frame_recorder() override;

        // Opens the file and registers one camera stream. 
        // Recording begins with the next observed frame.
        [[nodiscard]] bool start(const std::filesystem::path& path, const camera_stream_info_t& camera) noexcept;

        // Drains whatever is queued, then finalizes the file. Idempotent.
        void stop() noexcept;

        bool is_started() const noexcept { return _is_started.load(std::memory_order_relaxed); }
        recording_stats_t stats() const noexcept;
        const std::filesystem::path& path() const noexcept { return _writer.path(); }

        void on_sensor_frame_update(const std::shared_ptr<hw::sensor_frame>& new_sensor_frame) override;
        void on_sensor_stream_reset() override;
        void on_sensor_frame_geometry_changed() override;
        void on_sensor_stream_end(hw::stream_end_reason_t reason) override;

    private:
        void _worker(std::stop_token stop);

    private:
        const size_t _queue_depth;

        recording_writer _writer;
        stream_id_t _stream_id{ 0 };

        mutable std::mutex _mtx; // guards _queue and _frames_dropped (the writer is the worker's alone)
        std::condition_variable_any _queue_cv;
        std::deque<std::shared_ptr<hw::sensor_frame>> _queue;
        uint64_t _frames_dropped{ 0 };

        std::atomic_bool _is_started{ false };
        std::jthread _thread;
    };

} // namespace io
