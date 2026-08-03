#pragma once
#include "calibration.hh"
#include "sensor_frame_source.hh"
#include "sensor_frame_observer.hh"
#include "source_config.hh"

#include <Eigen/Core>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace hw
{
    // Owns a camera backend and runs a background polling thread 
    // that pulls framesets from it and pushes their frames to observers.
    // SDK-agnostic: the concrete backend is created only in the .cc.
    class sensor_frame_provider final {
    public:
        sensor_frame_provider() = default;
        ~sensor_frame_provider();

        sensor_frame_provider(const sensor_frame_provider&) = delete;
        sensor_frame_provider& operator=(const sensor_frame_provider&) = delete;
        sensor_frame_provider(sensor_frame_provider&&) = delete;
        sensor_frame_provider& operator=(sensor_frame_provider&&) = delete;

        void add_observer(std::shared_ptr<sensor_frame_observer> observer);
        void remove_observer(const std::shared_ptr<sensor_frame_observer>& observer);

        bool is_opened() const;
        [[nodiscard]] bool open(const source_config_t& config) noexcept;
        void close();

        source_backend_t get_source_backend() const { return _source_backend; }
        const std::string& get_source_name() const { return _source_name; }

        // Describes the images observers actually receive, not the raw sensor:
        // an ROI shifts the principal point and shrinks the resolution these report.
        const calibration_t& get_calibration() const { return _calib; }
        Eigen::Vector2i get_color_frame_resolution() const { return _color_frame_resolution; }
        frame_format_t get_color_format() const { return _color_format; }
        std::optional<roi_t> get_color_roi() const { return _color_roi; } // nullopt: whole frames

        float get_current_update_rate() const { return _update_rate.load(); } // EMA-smoothed fps

        // Position of the newest frame in this source's stream, restarting at zero on every open.
        // NOTE: This value is NOT an identity; for that, see `sensor_frame::id()`.
        uint32_t get_current_frame_seq() const { return _frame_seq.load(); }

        bool is_paused() const { return _paused.load(); }
        void play();
        void pause();

        // Recording sources only (no-op / 0 otherwise).
        void seek_recording_to_begin();
        void seek_recording_to_end();
        void seek_recording_timeline(timestamp_t timestamp);

        std::chrono::nanoseconds get_recording_length() const;
        timestamp_t get_first_record_timestamp() const;
        timestamp_t get_last_record_timestamp() const;

        float get_update_speed() const { return _speed.load(); }
        void  set_update_speed(float factor);

        bool is_auto_repeat_enabled() const;
        void set_auto_repeat(bool enable);

    private:
        void _install_source(
            std::unique_ptr<sensor_frame_source> source,
            source_backend_t source_backend,
            std::string source_name,
            const std::optional<roi_t>& requested_roi
        );

        void _start_thread();
        void _stop_thread();
        void _polling_thread_proc();

        std::vector<std::shared_ptr<sensor_frame_observer>> _snapshot_observers() const;
        void _notify_sensor_frame_update(const std::shared_ptr<sensor_frame>& frame);
        void _notify_sensor_stream_reset();
        void _notify_sensor_stream_end();

    private:
        std::unique_ptr<sensor_frame_source> _source;
        record_player_source* _player{ nullptr }; // non-owning; set only for recording sources
        mutable std::mutex _source_mtx;

        std::thread _thread;
        std::atomic<bool> _running{ false };
        std::atomic<bool> _paused{ false };
        std::atomic<bool> _need_repace{ false }; // request playback pacing anchor reset

        std::vector<std::shared_ptr<sensor_frame_observer>> _observers;
        mutable std::mutex _observers_mtx;

        // Cached at open(); immutable while streaming.
        calibration_t _calib{};
        Eigen::Vector2i _color_frame_resolution{ Eigen::Vector2i::Zero() };
        source_backend_t _source_backend{};
        std::string _source_name;
        frame_format_t _color_format{};
        std::optional<roi_t> _color_roi; // ROI in force

        std::atomic<uint32_t> _frame_seq{ 0 };
        std::atomic<float> _update_rate{ 0.0f };
        std::atomic<float> _speed{ 1.0f };
    }; // class

} // namespace hw
