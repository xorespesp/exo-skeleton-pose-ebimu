#pragma once
#include "joint_measurement.hh"

#include "hw/calibration.hh" // hw::intrinsic_t
#include "utils/serializable.hh"

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <string_view>
#include <vector>

namespace pose
{
    // One pose solution for a tag from orthogonal iteration.
    struct tag_pose_t {
        Eigen::Isometry3d transform{};     // tag -> camera
        double obj_err{ 0.0 };             // object-space error of this solution (lower = better fit)
        std::array<cv::Point2f, 4> axes{}; // pixel-projected {origin, X, Y, Z} of this solution
    };

    struct tag_detection_t {
        int id{ 0 };                          // decoded tag id
        int hamming{ 0 };                     // corrected error bits
        float decision_margin{ 0.0f };        // decode confidence (higher = better)
        cv::Point2f center{};                 // tag center in pixels
        std::array<cv::Point2f, 4> corners{}; // corner pixels, counter-clockwise
        std::array<tag_pose_t, 2> pose_candidates{}; // raw orthogonal-iteration solutions, obj_err ascending
        int num_pose_candidates{ 0 };         // valid entries in pose_candidates (0 without intrinsics, else 1 or 2)
        std::optional<tag_pose_t> pose;       // pose the selector chose (tag->camera); empty without intrinsics
    };

    // ---------------------------------------------------------------------------
    // Pose-candidate selector
    // ---------------------------------------------------------------------------
    //
    // Orthogonal iteration can yield two poses (planar ambiguity). The detector applies a selector
    // once while building each detection and keeps only the chosen pose in tag_detection_t::pose.
    // Any callable works (no registration) and may be stateful (keyed on tag id), but must depend
    // only on past detections, as it runs at detection time.
    using tag_pose_candidate_selector_fn = std::function<const tag_pose_t*(int tag_id, std::span<const tag_pose_t> candidates)>;

    namespace selectors
    {
        // Default policy: lowest object-space error (best geometric fit). Stateless.
        [[nodiscard]] inline const tag_pose_t* min_error(int /*tag_id*/, std::span<const tag_pose_t> candidates) noexcept
        {
            if (candidates.empty()) { return nullptr; }
            return &*std::ranges::min_element(candidates, {}, &tag_pose_t::obj_err);
        }
    } // namespace selectors

    // Apriltag detector wrapping the C apriltag library.
    class tag_detector {
    public:
        // How a tag's 3D pose (tag->camera) is estimated from its corners.
        enum class pose_method_t {
            // Orthogonal iteration: refines the pose to minimize object-space error and returns BOTH
            // planar-ambiguity solutions. Most accurate rotation; costs num_iters x 2 per tag.
            orthogonal_iteration,
            // Homography decomposition (closed form, from the detector's already-computed homography):
            // a single perspective-correct pose. Rotation is coarser, but the translation (hence depth)
            // is comparable and far cheaper. One candidate; no planar-ambiguity pair.
            homography,
        };

        // Tuning options for the detector.
        struct options_t {
            float quad_decimate{ 2.0f }; // 1.0 = full resolution (best corner accuracy)
            float quad_sigma{ 0.0f };    // Gaussian blur sigma for quad detection (0 = none)
            bool refine_edges{ true };   // align quad edges to image gradients (better accuracy)
            int num_iters{ 20 };         // orthogonal-iteration steps per tag
            int num_threads{ 4 };        // detection worker threads
            pose_method_t pose_method{ pose_method_t::orthogonal_iteration }; // tag->camera pose estimator

            DECLARE_SERIALIZABLE_FIELDS(
                v("quad_decimate", o.quad_decimate);
                v("quad_sigma",    o.quad_sigma);
                v("refine_edges",  o.refine_edges);
                v("num_iters",     o.num_iters);
                v("num_threads",   o.num_threads);
                v("pose_method",   o.pose_method);
            )
        };

        // TODO: make the tag family selectable (currently fixed to tagStandard41h12).
        explicit tag_detector(
            const options_t& opt = {},
            double tag_size_m = 0.05,
            std::optional<hw::intrinsic_t> intrinsics = std::nullopt,
            tag_pose_candidate_selector_fn pose_selector = selectors::min_error
        );
        ~tag_detector();

        tag_detector(const tag_detector&) = delete;
        tag_detector& operator=(const tag_detector&) = delete;

        // Accepts BGR / BGRA / grayscale. Fills pose + axes when intrinsics were supplied.
        std::vector<tag_detection_t> detect(const cv::Mat& image);

    private:
        struct context_t;

        options_t _opt;
        double _tag_size_m;
        std::optional<hw::intrinsic_t> _intrinsics;
        tag_pose_candidate_selector_fn _pose_selector;
        std::unique_ptr<context_t> _ctx;
    };

    constexpr std::string_view pose_method_to_str(tag_detector::pose_method_t m)
    {
        return (m == tag_detector::pose_method_t::homography) ? "homography" : "orthogonal_iteration";
    }

    // nullopt if `str` names neither method.
    constexpr std::optional<tag_detector::pose_method_t> pose_method_from_str(std::string_view str)
    {
        using m_t = tag_detector::pose_method_t;
        if (str == pose_method_to_str(m_t::orthogonal_iteration)) { return m_t::orthogonal_iteration; }
        if (str == pose_method_to_str(m_t::homography)) { return m_t::homography; }
        return std::nullopt;
    }

    // ---------------------------------------------------------------------------
    // Tag detections -> estimator measurements
    // ---------------------------------------------------------------------------
    //
    // A tag states its own identity, so binding it to the rig is a table lookup and needs no state
    // across frames. These are free functions: the detector decodes tags and knows nothing of the
    // rig, and that stays true with the binding sitting beside it rather than inside it.
    //
    // Both drop what they cannot bind, so the returned span is exactly what an estimator can use.

    // Mean edge length of a tag quad [px]. Against the tag's printed size it gives a metric scale.
    [[nodiscard]] inline double mean_edge_px(const std::array<cv::Point2f, 4>& corners)
    {
        double sum = 0.0;
        for (int k = 0; k < 4; ++k) {
            const cv::Point2f d = corners[(k + 1) % 4] - corners[k];
            sum += std::sqrt(static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y);
        }
        return sum / 4.0;
    }

    // Camera-space position of a detected tag: the chosen pose's translation, falling back to the
    // first candidate. A tag's pose candidates share one translation, so the choice does not reach
    // the position. Empty without intrinsics, since no pose was solved then.
    [[nodiscard]] inline std::optional<Eigen::Vector3d> detection_position(const tag_detection_t& det)
    {
        if (det.pose.has_value()) { return det.pose->transform.translation(); }
        if (det.num_pose_candidates > 0) { return det.pose_candidates[0].transform.translation(); }
        return std::nullopt;
    }

    // Tag centers as image-plane measurements. `tag_size_m` is the printed black-square edge length,
    // which turns each quad's pixel size into that measurement's metric scale.
    // Tags outside the rig's table are dropped.
    [[nodiscard]] inline std::vector<joint_2d_measurement_t> bind_2d_measurements(
        std::span<const tag_detection_t> detections,
        double tag_size_m)
    {
        std::vector<joint_2d_measurement_t> out;
        out.reserve(detections.size());
        for (const auto& det : detections)
        {
            const auto joint = tag_id_to_joint_id(det.id);
            if (!joint.has_value()) { continue; }

            joint_2d_measurement_t m{};
            m.joint_id = joint.value();
            m.center_px = Eigen::Vector2d{ det.center.x, det.center.y };

            // A degenerate quad would divide into a wild scale, so it votes on the center alone.
            if (const double edge_px = mean_edge_px(det.corners); edge_px > 1e-6) {
                m.meters_per_pixel = tag_size_m / edge_px;
            }
            out.push_back(m);
        }
        return out;
    }

    // Tag poses as rig-space position measurements. A frontal camera's frame is the rig frame, so
    // the translation passes through. Tags outside the rig's table, and tags whose pose went
    // unsolved, are dropped.
    [[nodiscard]] inline std::vector<joint_3d_measurement_t> bind_3d_measurements(
        std::span<const tag_detection_t> detections)
    {
        std::vector<joint_3d_measurement_t> out;
        out.reserve(detections.size());
        for (const auto& det : detections)
        {
            const auto joint = tag_id_to_joint_id(det.id);
            if (!joint.has_value()) { continue; }
            const auto p = detection_position(det);
            if (!p.has_value()) { continue; }

            out.push_back(joint_3d_measurement_t{ .joint_id = joint.value(), .position = p.value() });
        }
        return out;
    }

    // Draw tag outlines, ids, and 3D axes (when present).
    void draw_tag_detections(
        cv::Mat& bgr,
        std::span<const tag_detection_t> detections
    );

} // namespace pose
