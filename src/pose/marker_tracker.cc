#include "marker_tracker.hh"

#include <spdlog/spdlog.h>

#include <format>
#include <utility>

namespace pose
{
    namespace
    {
        // Tag ids beyond this are still detected and bound; they just fall out of the
        // appeared/disappeared bookkeeping, which tracks visibility in a 64-bit mask.
        constexpr int kMaxLoggedTagId = 63;

        // "3(r_ankle)": a log line names the joint a tag stands for, not just the raw id. An id the
        // rig does not bind is worth saying out loud, since a stray tag in view or the wrong family
        // printed both read as silence otherwise.
        std::string tag_label(int tag_id)
        {
            const auto j = tag_id_to_joint_id(tag_id);
            return j.has_value()
                ? std::format("{}({})", tag_id, get_joint_name(j.value()))
                : std::format("{}(unmapped)", tag_id);
        }

        std::string tag_list(std::uint64_t mask)
        {
            std::string s;
            for (int t = 0; t <= kMaxLoggedTagId; ++t)
            {
                if ((mask & (1ull << t)) == 0) { continue; }
                if (!s.empty()) { s += ", "; }
                s += tag_label(t);
            }
            return s.empty() ? std::string{ "none" } : s;
        }
    } // namespace

    // ---------------------------------------------------------------------------
    // apriltag_tracker
    // ---------------------------------------------------------------------------

    apriltag_tracker::apriltag_tracker(
        const tag_detector::options_t& opt,
        const double tag_size_m,
        std::optional<hw::intrinsic_t> intrinsics)
        : _intrinsics{ std::move(intrinsics) }
        , _opt{ opt }
        , _tag_size_m{ tag_size_m }
    { }

    apriltag_tracker::~apriltag_tracker() = default;

    void apriltag_tracker::set_options(const tag_detector::options_t& opt)
    {
        std::scoped_lock lk{ _mtx };
        _opt = opt;
        _dirty = true;
    }

    tag_detector::options_t apriltag_tracker::options() const
    {
        std::scoped_lock lk{ _mtx };
        return _opt;
    }

    void apriltag_tracker::set_tag_size_m(const double v)
    {
        std::scoped_lock lk{ _mtx };
        _tag_size_m = v;
        _dirty = true;
    }

    double apriltag_tracker::tag_size_m() const
    {
        std::scoped_lock lk{ _mtx };
        return _tag_size_m;
    }

    void apriltag_tracker::set_intrinsics(const std::optional<hw::intrinsic_t>& intrinsics)
    {
        std::scoped_lock lk{ _mtx };
        _intrinsics = intrinsics;
        _dirty = true;
    }

    std::vector<tag_detection_t> apriltag_tracker::last_detections() const
    {
        return _latch.read([](const std::vector<tag_detection_t>& tags) { return tags; });
    }

    std::size_t apriltag_tracker::last_detection_count() const
    {
        return _latch.read([](const std::vector<tag_detection_t>& tags) { return tags.size(); });
    }

    void apriltag_tracker::process_frame(
        const cv::Mat& image,
        hw::frame_format_t /*format*/,
        const hw::timestamp_t timestamp,
        cv::Mat* annotated)
    {
        // The detector reads luminance, which every layout a source delivers carries, so no frame
        // is refused here.
        tag_detector::options_t opt;
        double tag_size_m{};
        std::optional<hw::intrinsic_t> intrinsics;
        bool rebuild;
        {
            std::scoped_lock lk{ _mtx };
            opt = _opt;
            tag_size_m = _tag_size_m;
            intrinsics = _intrinsics;
            rebuild = !_detector.has_value() || _dirty;
            _dirty = false;
        }

        if (rebuild)
        {
            // The detector is built and used on this thread alone; the other one only ever stages a
            // request under the lock above.
            _detector.emplace(opt, tag_size_m, intrinsics);
            spdlog::debug("tracker: tag detector built (tag {:.3f} m, decimate {:.2f}, sigma {:.2f}, "
                          "refine {}, iters {}, threads {}, pose {})",
                tag_size_m, opt.quad_decimate, opt.quad_sigma, opt.refine_edges,
                opt.num_iters, opt.num_threads,
                !intrinsics.has_value()
                    ? "off"
                    : (opt.pose_method == tag_detector::pose_method_t::homography ? "homography" : "OI"));
        }

        std::vector<tag_detection_t> found = _detector.value().detect(image);
        if (annotated != nullptr) { draw_tag_detections(*annotated, found); }

        // Which ids are in view is this technology's own diagnostic, so it is reported here rather
        // than by a holder that would have to ask what kind it was holding. Logged as edges, so a
        // steady stream stays silent.
        std::uint64_t mask = 0;
        for (const auto& det : found) {
            if (det.id < 0 || det.id > kMaxLoggedTagId) { continue; }
            mask |= (1ull << det.id);
        }
        if (mask != _seen_tag_mask)
        {
            if (const std::uint64_t appeared = mask & ~_seen_tag_mask) {
                spdlog::debug("tracker: tag(s) detected: {}", tag_list(appeared));
            }
            if (const std::uint64_t lost = _seen_tag_mask & ~mask) {
                spdlog::debug("tracker: tag(s) lost: {}", tag_list(lost));
            }
            _seen_tag_mask = mask;
        }

        _latch.publish(std::move(found), timestamp);
    }

    bool apriltag_tracker::try_get_2d_measurements(
        std::vector<joint_2d_measurement_t>& out,
        hw::timestamp_t& timestamp)
    {
        // Bound outside the latch: a table lookup per tag is no reason to hold up the frame thread.
        std::vector<tag_detection_t> tags;
        if (!_latch.try_take(tags, timestamp)) { return false; }

        out = bind_2d_measurements(tags, this->tag_size_m());
        return true;
    }

    bool apriltag_tracker::try_get_3d_measurements(
        std::vector<joint_3d_measurement_t>& out,
        hw::timestamp_t& timestamp)
    {
        std::vector<tag_detection_t> tags;
        if (!_latch.try_take(tags, timestamp)) { return false; }

        out = bind_3d_measurements(tags);
        return true;
    }

    // ---------------------------------------------------------------------------
    // color_marker_tracker
    // ---------------------------------------------------------------------------

    color_marker_tracker::color_marker_tracker(
        const color_marker_detector::options_t& detector_opt,
        const color_marker_assigner::options_t& assigner_opt)
        : _assigner{ assigner_opt }
        , _opt{ detector_opt }
    { }

    color_marker_tracker::~color_marker_tracker() = default;

    void color_marker_tracker::set_detector_options(const color_marker_detector::options_t& opt)
    {
        std::scoped_lock lk{ _mtx };
        _opt = opt;
        _dirty = true;
    }

    color_marker_detector::options_t color_marker_tracker::detector_options() const
    {
        std::scoped_lock lk{ _mtx };
        return _opt;
    }

    std::vector<marker_detection_t> color_marker_tracker::last_detections() const
    {
        return _latch.read([](const color_frame_t& frame) { return frame.blobs; });
    }

    marker_reject_stats_t color_marker_tracker::reject_stats() const
    {
        return _latch.read([](const color_frame_t& frame) { return frame.rejects; });
    }

    std::size_t color_marker_tracker::last_detection_count() const
    {
        return _latch.read([](const color_frame_t& frame) { return frame.blobs.size(); });
    }

    void color_marker_tracker::set_publish_debug_images(const bool on)
    {
        std::scoped_lock lk{ _mtx };
        _publish_debug_images = on;
    }

    cv::Mat color_marker_tracker::mask() const
    {
        return _latch.read([](const color_frame_t& frame) { return frame.mask; });
    }

    cv::Mat color_marker_tracker::score_image() const
    {
        return _latch.read([](const color_frame_t& frame) { return frame.score; });
    }

    void color_marker_tracker::process_frame(
        const cv::Mat& image,
        const hw::frame_format_t format,
        const hw::timestamp_t timestamp,
        cv::Mat* annotated)
    {
        // Colour classification needs the colour. A gray source cannot supply it, and a recording
        // replays whatever layout it was written with, so the mismatch surfaces here rather than at
        // open. Said once per stream: repeating it every frame would bury the rest of the log.
        if (format == hw::frame_format_t::gray8)
        {
            if (!_warned_gray) {
                _warned_gray = true;
                spdlog::error("tracker: the source delivers {} frames, which carry no color for the "
                              "color marker detector to classify",
                    hw::frame_format_to_str(format));
            }
            return;
        }

        color_marker_detector::options_t opt;
        bool rebuild;
        bool publish_debug;
        {
            std::scoped_lock lk{ _mtx };
            opt = _opt;
            rebuild = !_detector.has_value() || _dirty;
            _dirty = false;
            publish_debug = _publish_debug_images;
        }

        if (rebuild)
        {
            _detector.emplace(opt);

            const color_model_t& m = opt.model;
            spdlog::debug("tracker: color marker detector built (model {}, area {:.0f}-{:.0f} px, "
                          "fill {:.2f}, aspect {:.2f}, score {:.2f}, open {}, close {})",
                m.valid ? std::format("a*{:+.1f} b*{:+.1f} d<{:.1f}", m.mean_ab.x(), m.mean_ab.y(), m.max_distance)
                        : "not fitted",
                opt.min_area_px, opt.max_area_px, opt.min_fill, opt.max_aspect, opt.min_score,
                opt.open_kernel_px, opt.close_kernel_px);

            if (!m.valid) {
                spdlog::warn("tracker: the color model has not been fitted, so nothing will be "
                             "detected; sample a marker in the debugger's Color Model panel "
                             "and press Fit");
            }
        }

        color_frame_t frame;
        frame.blobs = _detector.value().detect(image);
        frame.rejects = _detector.value().reject_stats();
        if (annotated != nullptr) { draw_marker_detections(*annotated, frame.blobs); }

        // The detector writes over these on the next frame, so a reader on the other thread needs
        // its own copy. Taken only while a view is asking, which is what keeps a frame's worth of
        // pixels off the normal path.
        if (publish_debug) {
            frame.mask = _detector.value().mask().clone();
            frame.score = _detector.value().score_image().clone();
        }

        _latch.publish(std::move(frame), timestamp);
    }

    bool color_marker_tracker::try_get_2d_measurements(
        std::vector<joint_2d_measurement_t>& out,
        hw::timestamp_t& timestamp)
    {
        // The assigner runs on this thread and names outside the latch: naming walks the whole leg
        // and would hold up the detection thread.
        color_frame_t frame;
        if (!_latch.try_take(frame, timestamp)) { return false; }

        out = _assigner.assign(frame.blobs);
        return true;
    }

    void color_marker_tracker::on_rest_pose_captured()
    {
        // The reference is measured through the same camera as everything checked against it, so
        // however the camera was mounted cancels out and only the leg's own motion is left to vary.
        if (_assigner.capture_reference()) {
            spdlog::info("tracker: marker geometry reference captured");
        } else {
            spdlog::warn("tracker: the whole leg is not assigned, so no marker geometry reference "
                         "was captured and that check stays inactive");
        }
    }

    void color_marker_tracker::on_rest_pose_cleared()
    {
        _assigner.clear_reference(); // captured together, so dropped together
    }

    void color_marker_tracker::on_stream_reset()
    {
        // A marker carries no identity of its own, so the assigner names it from where its
        // neighbours were last frame. Across a jump they were somewhere else entirely.
        _assigner.reset();
    }

} // namespace pose
