#pragma once
#include "color_marker_detector.hh"
#include "joint_measurement.hh"
#include "tag_detector.hh"

#include "hw/calibration.hh"
#include "hw/frame_format.hh"
#include "hw/timestamp.hh"

#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

namespace pose
{
    // Hands a producer's latest value to a consumer across a thread boundary. A publish overwrites
    // whatever has not been taken, so a slow consumer gets the newest value and misses the ones
    // between; a take gets one whole value or nothing, never a mix of two.
    template <typename Payload>
    class latest_value_latch
    {
    public:
        void publish(Payload payload, hw::timestamp_t at)
        {
            std::scoped_lock lk{ _mtx };
            _payload = std::move(payload);
            _at = at;
            _unread = true;
        }

        // False when nothing has been published since the last take, leaving the outputs untouched.
        bool try_take(Payload& out, hw::timestamp_t& at)
        {
            std::scoped_lock lk{ _mtx };
            if (!_unread) { return false; }
            _unread = false;
            out = _payload;
            at = _at;
            return true;
        }

        // Reads the newest frame without claiming it, which is what a readout does. `fn` runs under
        // the lock, so it returns a copy of what it wants rather than a handle to it.
        template <typename Fn>
        auto read(Fn&& fn) const
        {
            std::scoped_lock lk{ _mtx };
            return fn(_payload);
        }

    private:
        mutable std::mutex _mtx;
        Payload _payload{};
        hw::timestamp_t _at{};
        bool _unread{ false }; // a frame has been published that no take has claimed
    };

    // ---------------------------------------------------------------------------
    // Marker tracker: one marker technology's whole path from a frame to measurements
    // ---------------------------------------------------------------------------
    //
    // Everything that differs between marker technologies sits behind this interface: what a frame
    // has to carry, how a detection is made and drawn, how a detection becomes a named joint, and
    // what a rest-pose capture means for it. A holder drives one without naming its kind, and
    // reaches a technology's own readouts by asking for that type.
    //
    // Two threads share one tracker, and the sections below say which member belongs to which:
    // one runs wherever frames arrive, the other on the thread that steps the estimator. An
    // implementation carries what has to cross in one guarded member and publishes it at the end
    // of a frame, so a reader gets a whole frame's worth or nothing.
    class marker_tracker_base
    {
    public:
        virtual ~marker_tracker_base() = default;

        marker_tracker_base(const marker_tracker_base&) = delete;
        marker_tracker_base& operator=(const marker_tracker_base&) = delete;

        // --- frame thread ---------------------------------------------------------

        // Take one frame in. What was found is published for the estimator thread to collect, which
        // is why nothing comes back here.
        //
        // `annotated`, when non-null, is a BGR copy of this same frame to draw the detections over.
        // Drawing happens here, where the detections are still the ones this frame produced.
        //
        // A frame whose pixel layout this tracker cannot read publishes nothing, which reads
        // downstream as a frame that measured nothing.
        virtual void process_frame(
            const cv::Mat& image,
            hw::frame_format_t format,
            hw::timestamp_t timestamp,
            cv::Mat* annotated
        ) = 0;

        // --- estimator thread -----------------------------------------------------

        // The newest published frame's measurements, each already bound to the joint it belongs to,
        // along with the moment that frame was captured. False when nothing has been published
        // since the last call, leaving the outputs untouched.
        //
        // The time comes back with the measurements so the two can never describe different frames,
        // which is what an estimator's dt is read from.
        //
        // A tracker that cannot produce one of the two dimensionalities always answers false for it.
        virtual bool try_get_2d_measurements(std::vector<joint_2d_measurement_t>& out, hw::timestamp_t& timestamp) = 0;
        virtual bool try_get_3d_measurements(std::vector<joint_3d_measurement_t>& out, hw::timestamp_t& timestamp) = 0;

        // Markers found in the last published frame. Against `is_tracking()` this says where a
        // stalled run broke: nothing found is a detector problem, found but not tracking is a
        // naming problem.
        virtual std::size_t last_detection_count() const = 0;

        // Whether the tracker is following markers it has already identified. A tracker whose
        // markers state their own identity is always following them.
        virtual bool is_tracking() const { return true; }

        // The rig's rest pose was just captured, with an operator watching the annotated frame and
        // vouching for what it shows. A tracker with a reference of its own latches it here.
        virtual void on_rest_pose_captured() {}
        virtual void on_rest_pose_cleared() {}

        // The stream jumped to another position, so whatever this tracker followed from frame to
        // frame describes where the stream no longer is. A technology whose markers state their
        // own identity carries nothing across frames and has nothing to do here.
        virtual void on_stream_reset() {}

    protected:
        marker_tracker_base() = default;
    };

    // ---------------------------------------------------------------------------
    // AprilTag
    // ---------------------------------------------------------------------------
    //
    // A tag states its own id, so binding is a table lookup with no state across frames.
    class apriltag_tracker final : public marker_tracker_base
    {
    public:
        // `intrinsics` are what turn the per-tag pose solve on, and with it the 3D measurements a
        // frontal estimator consumes. Left empty, detection still yields tag centers.
        apriltag_tracker(
            const tag_detector::options_t& opt,
            double tag_size_m,
            std::optional<hw::intrinsic_t> intrinsics
        );
        ~apriltag_tracker() override;

        // Estimator thread. The detector is rebuilt from these before the next detect().
        void set_options(const tag_detector::options_t& opt);
        tag_detector::options_t options() const;

        // Printed black-square edge length [m]. Reaches the pose solve and the metric scale that
        // rides on each image-plane measurement.
        void set_tag_size_m(double v);
        double tag_size_m() const;

        // The delivered images cover a different part of the sensor, so the principal point the
        // pose solve reads has moved with them. Empty leaves tag centers, with no pose solved.
        void set_intrinsics(const std::optional<hw::intrinsic_t>& intrinsics);

        // Estimator thread. The last published frame's tags, for a diagnostic trace.
        std::vector<tag_detection_t> last_detections() const;

        void process_frame(
            const cv::Mat& image, 
            hw::frame_format_t format,
            hw::timestamp_t timestamp, 
            cv::Mat* annotated
        ) override;

        bool try_get_2d_measurements(std::vector<joint_2d_measurement_t>& out, hw::timestamp_t& timestamp) override;
        bool try_get_3d_measurements(std::vector<joint_3d_measurement_t>& out, hw::timestamp_t& timestamp) override;

        std::size_t last_detection_count() const override;

    private:
        std::optional<tag_detector> _detector; // frame thread; rebuilt when `_dirty`
        std::uint64_t _seen_tag_mask{ 0 };     // frame thread; bit t = id t was in the previous frame

        mutable std::mutex _mtx; // guards the tuning below, which both threads reach
        std::optional<hw::intrinsic_t> _intrinsics;
        tag_detector::options_t _opt;
        double _tag_size_m;
        bool _dirty{ true }; // forces a build on the first frame, then after each stage

        latest_value_latch<std::vector<tag_detection_t>> _latch;
    };

    // ---------------------------------------------------------------------------
    // Colour markers
    // ---------------------------------------------------------------------------
    //
    // A marker carries no identity of its own, so a second stage names it from where it sits among
    // its neighbours. That stage carries state across frames and a reference latched at rest-pose
    // capture, both of which live on the estimator thread beside the calibration they follow.
    class color_marker_tracker final : public marker_tracker_base
    {
    public:
        color_marker_tracker(
            const color_marker_detector::options_t& detector_opt,
            const color_marker_assigner::options_t& assigner_opt
        );
        ~color_marker_tracker() override;

        // Estimator thread. The detector is rebuilt from these before the next detect().
        void set_detector_options(const color_marker_detector::options_t& opt);
        color_marker_detector::options_t detector_options() const;

        // Estimator thread. Read a copy, edit it, hand it back; the next frame uses it.
        color_marker_assigner::options_t assigner_options() const noexcept { return _assigner.options(); }
        void set_assigner_options(const color_marker_assigner::options_t& opt) { _assigner.options() = opt; }

        const color_marker_assigner::stats_t& assigner_stats() const noexcept { return _assigner.stats(); }

        // Estimator thread. The last published frame's blobs and why others were dropped, which is
        // what a tuning panel shows.
        std::vector<marker_detection_t> last_detections() const;
        marker_reject_stats_t reject_stats() const;

        // What the classifier decided, per pixel: which ones passed (`mask`) and how surely
        // (`score_image`). Reading them turns their capture on, since each is a frame's worth of
        // pixels copied on the frame thread and a run is not usually being tuned.
        //
        // Both are empty until a frame has been published with capture on, and empty again from
        // the first frame after it goes off, so nothing holds a frame's pixels for a closed view.
        void set_publish_debug_images(bool on);
        cv::Mat mask() const;
        cv::Mat score_image() const;

        void process_frame(
            const cv::Mat& image, 
            hw::frame_format_t format,
            hw::timestamp_t timestamp, 
            cv::Mat* annotated
        ) override;

        bool try_get_2d_measurements(std::vector<joint_2d_measurement_t>& out, hw::timestamp_t& timestamp) override;
        bool try_get_3d_measurements(std::vector<joint_3d_measurement_t>&, hw::timestamp_t&) override { return false; } // no 3D measurements

        std::size_t last_detection_count() const override;
        bool is_tracking() const override { return _assigner.stats().locked; }

        void on_rest_pose_captured() override;
        void on_rest_pose_cleared() override;
        void on_stream_reset() override;

    private:
        std::optional<color_marker_detector> _detector; // frame thread; rebuilt when `_dirty`
        bool _warned_gray{ false };                     // frame thread; this stream was refused once

        color_marker_assigner _assigner; // estimator thread

        mutable std::mutex _mtx; // guards the tuning below, which both threads reach
        color_marker_detector::options_t _opt;
        bool _dirty{ true }; // forces a build on the first frame, then after each stage

        bool _publish_debug_images{ false }; // a tuning view is reading the per-pixel decisions

        // One frame of detection, published together so a readout cannot pair this frame's blobs
        // with the previous frame's mask.
        struct color_frame_t
        {
            std::vector<marker_detection_t> blobs;
            marker_reject_stats_t rejects{};
            cv::Mat mask, score; // empty while `_publish_debug_images` is off
        };
        latest_value_latch<color_frame_t> _latch;
    };

} // namespace pose
