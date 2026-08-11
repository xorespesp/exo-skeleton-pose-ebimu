#pragma once
#include "calibration.hh"
#include "frame_format.hh"
#include "roi.hh"
#include "sensor_frameset.hh"
#include "timestamp.hh"

#include <chrono>
#include <memory>
#include <optional>

namespace hw
{
    // Abstract camera backend interface. SDK-agnostic.
    class sensor_frame_source {
    public:
        virtual ~sensor_frame_source() = default;

        virtual bool is_valid() const = 0;
        virtual void close() = 0;

        virtual const calibration_t& get_calibration() const = 0;
        virtual frame_format_t get_frame_format() const = 0;

        // Narrow delivered images to `roi`, given in full-frame pixels. The source is the
        // authority on its own frame, so it clips `roi` to that frame, and a sensor's readout
        // window may quantize it further. Returns whatever came of that.
        //
        // Empty means no ROI is in force afterwards, whether because this source does not narrow
        // at all or because `roi` left nothing of the frame. Nothing narrows on a source's
        // behalf, so an empty answer means the request is dropped. Implement this wherever the
        // backend can, be it in the sensor's readout window or ahead of the colour conversion.
        virtual std::optional<roi_t> try_set_roi(const roi_t& /*roi*/) { return std::nullopt; }

        // Blocking. Returns the frames of one capture, already converted and narrowed.
        // Empty on EOF / timeout / disconnect.
        [[nodiscard]] virtual std::optional<sensor_frameset> fetch_next_sensor_frameset() = 0;
    };

    // Recording playback backends.
    class record_player_source : public sensor_frame_source {
    public:
        virtual std::chrono::nanoseconds get_recording_length() const = 0;
        virtual timestamp_t get_first_record_timestamp() const = 0;
        virtual timestamp_t get_last_record_timestamp() const = 0;

        virtual void seek_begin() = 0;
        virtual void seek_end() = 0;
        virtual void seek_timestamp(timestamp_t timestamp) = 0;
    };

} // namespace hw
