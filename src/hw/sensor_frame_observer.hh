#pragma once
#include "sensor_frame.hh"

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>

namespace hw
{
    // Callbacks may be invoked from the provider's worker thread.
    class sensor_frame_observer
    {
    public:
        sensor_frame_observer() = default;
        sensor_frame_observer(sensor_frame_observer&) = delete;
        sensor_frame_observer(sensor_frame_observer&&) = delete;
        sensor_frame_observer& operator=(sensor_frame_observer&) = delete;
        sensor_frame_observer& operator=(sensor_frame_observer&&) = delete;
        virtual ~sensor_frame_observer() = default;

        // Called when a new sensor frame is available.
        // NOTE: This method may be called from a worker thread.
        virtual void on_sensor_frame_update(const std::shared_ptr<sensor_frame>& new_sensor_frame) = 0;

        // Called when the stream position jumps: a source being installed, a seek, or a recording
        // starting over at its end. Raised where no frame of the old position can follow, so an
        // observer drops what it carries between frames here and needs no test of its own.
        // NOTE: This method may be called from a worker thread.
        virtual void on_sensor_stream_reset() = 0;

        // Called when the delivered images start covering a different part of the sensor, so pixel
        // coordinates stop meaning what they meant. `on_sensor_stream_reset()` goes out with it.
        // NOTE: This method may be called from a worker thread.
        virtual void on_sensor_frame_geometry_changed() {}

        // Called when the stream terminates (e.g., at the end of playback).
        // NOTE: This method may be called from a worker thread.
        virtual void on_sensor_stream_end() = 0;
    };

    // Latches only the latest frame for pull-style consumers.
    class sensor_frame_observer_nonbuffered : public sensor_frame_observer
    {
    public:
        bool is_terminated() const
        {
            return _termination_flag;
        }

        bool has_sensor_frame() const
        {
            std::scoped_lock lk{ _mtx };
            return _last_sensor_frame.has_value();
        }

        inline bool try_get_sensor_frame(std::shared_ptr<sensor_frame>& new_sensor_frame)
        {
            std::scoped_lock lk{ _mtx };
            if (!_last_sensor_frame.has_value()) { return false; }
            new_sensor_frame = std::move(_last_sensor_frame.value());
            _last_sensor_frame.reset();
            return true;
        }

        void on_sensor_frame_update(const std::shared_ptr<sensor_frame>& new_sensor_frame) override
        {
            std::scoped_lock lk{ _mtx };
            _last_sensor_frame = new_sensor_frame;
        }

        void on_sensor_stream_reset() override
        {
            _termination_flag = false;
        }

        void on_sensor_stream_end() override
        {
            _termination_flag = true;
        }

    private:
        std::optional<std::shared_ptr<sensor_frame>> _last_sensor_frame;
        std::atomic_bool _termination_flag = false;
        mutable std::mutex _mtx;
    };

} // namespace hw
