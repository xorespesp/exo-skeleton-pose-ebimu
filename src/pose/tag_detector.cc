#include "tag_detector.hh"

#include <apriltag.h>
#include <apriltag_pose.h>
#include <tagStandard41h12.h>

#include <opencv2/imgproc.hpp>

#include <array>
#include <format>
#include <string>
#include <utility>

namespace pose
{
    namespace
    {
        Eigen::Isometry3d to_isometry(const apriltag_pose_t& p)
        {
            // matd_t stores its 3x3 (R) / 3x1 (t) doubles row-major and contiguous.
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            transform.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(p.R->data);
            transform.translation() = Eigen::Map<const Eigen::Vector3d>(p.t->data);
            return transform;
        }

        cv::Point2f project(const hw::intrinsic_t& k, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p_tag)
        {
            const Eigen::Vector3d p_cam = pose * p_tag;
            return {
                static_cast<float>(k.fx * p_cam.x() / p_cam.z() + k.cx),
                static_cast<float>(k.fy * p_cam.y() / p_cam.z() + k.cy)
            };
        }

    } // namespace

    struct tag_detector::context_t
    {
        std::unique_ptr<::apriltag_detector, void(*)(::apriltag_detector*)> detector{
            ::apriltag_detector_create(), 
            &::apriltag_detector_destroy 
        };
        std::unique_ptr<::apriltag_family, void(*)(::apriltag_family*)> family{
            ::tagStandard41h12_create(), 
            &::tagStandard41h12_destroy 
        };
    };

    tag_detector::tag_detector(
        const options_t& opt,
        double tag_size_m,
        std::optional<hw::intrinsic_t> intrinsics,
        tag_pose_candidate_selector_fn pose_selector)
        : _opt{ opt }
        , _tag_size_m{ tag_size_m }
        , _intrinsics{ std::move(intrinsics) }
        , _pose_selector{ std::move(pose_selector) }
        , _ctx{ std::make_unique<context_t>() }
    {
        ::apriltag_detector_add_family(_ctx->detector.get(), _ctx->family.get());

        _ctx->detector->quad_decimate = _opt.quad_decimate;
        _ctx->detector->quad_sigma = _opt.quad_sigma;
        _ctx->detector->nthreads = _opt.num_threads;
        _ctx->detector->refine_edges = _opt.refine_edges;
    }

    tag_detector::~tag_detector() = default;

    std::vector<tag_detection_t> tag_detector::detect(const cv::Mat& image)
    {
        cv::Mat gray;
        if (image.channels() == 3) { cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY); }
        else if (image.channels() == 4) { cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY); }
        else { gray = image; }
        if (!gray.isContinuous()) { gray = gray.clone(); }

        ::image_u8_t im{ gray.cols, gray.rows, static_cast<int32_t>(gray.step[0]), gray.data };

        const std::unique_ptr<::zarray_t, void(*)(::zarray_t*)> raw{
            ::apriltag_detector_detect(_ctx->detector.get(), &im),
            &::apriltag_detections_destroy
        };

        std::vector<tag_detection_t> detections;
        detections.reserve(::zarray_size(raw.get()));
        for (int i = 0; i < ::zarray_size(raw.get()); ++i) {
            ::apriltag_detection_t* d = nullptr;
            ::zarray_get(raw.get(), i, &d);

            tag_detection_t& det = detections.emplace_back();
            det.id = d->id;
            det.hamming = d->hamming;
            det.decision_margin = d->decision_margin;
            det.center = { static_cast<float>(d->c[0]), static_cast<float>(d->c[1]) };
            for (size_t k = 0; k < det.corners.size(); ++k) {
                det.corners[k] = { static_cast<float>(d->p[k][0]), static_cast<float>(d->p[k][1]) };
            }

            if (!_intrinsics.has_value()) { continue; }

            const hw::intrinsic_t& intr = _intrinsics.value();
            ::apriltag_detection_info_t info{ d, _tag_size_m, intr.fx, intr.fy, intr.cx, intr.cy };

            const double len = _tag_size_m * 0.5;
            const auto make_tag_pose = [&intr, len](const ::apriltag_pose_t& p, double err) {
                const Eigen::Isometry3d tf = to_isometry(p);
                return tag_pose_t{
                    tf,
                    err,
                    std::array<cv::Point2f, 4>{
                        project(intr, tf, { 0.0, 0.0, 0.0 }),
                        project(intr, tf, { len, 0.0, 0.0 }),
                        project(intr, tf, { 0.0, len, 0.0 }),
                        project(intr, tf, { 0.0, 0.0, len }),
                    }
                };
            };

            std::array<tag_pose_t, 2> pose_cands_buff;
            size_t num_pose_cands{};

            if (_opt.pose_method == pose_method_t::homography)
            {
                // Closed-form pose from the detector's homography: one perspective-correct solution,
                // no planar-ambiguity pair. obj_err is not defined for this path, so report 0.
                ::apriltag_pose_t p{};
                ::estimate_pose_for_tag_homography(&info, &p);
                pose_cands_buff[0] = make_tag_pose(p, 0.0);
                num_pose_cands = 1;
                ::matd_destroy(p.R);
                ::matd_destroy(p.t);
            }
            else
            {
                // Run orthogonal iteration directly (estimate_tag_pose is just this + a min-error pick),
                // keeping both planar-ambiguity solutions and their errors for the selector.
                double err1{ 0.0 }, err2{ 0.0 };
                ::apriltag_pose_t p1{}, p2{};
                ::estimate_tag_pose_orthogonal_iteration(
                    &info,
                    &err1, &p1,
                    &err2, &p2,
                    _opt.num_iters
                );

                // NOTE: Orthogonal iteration always returns solution 1; the second exists only when a
                // distinct minimum was found (p2.R != null), else err2 == HUGE_VAL and p2.t is unset.
                pose_cands_buff[0] = make_tag_pose(p1, err1);
                num_pose_cands = 1;
                ::matd_destroy(p1.R);
                ::matd_destroy(p1.t);

                if (p2.R != nullptr) {
                    pose_cands_buff[1] = make_tag_pose(p2, err2);
                    num_pose_cands = 2;
                    ::matd_destroy(p2.R);
                    ::matd_destroy(p2.t);
                }

                // Order by object-space error (ascending) so candidate [0] is the best geometric fit.
                if (num_pose_cands == 2 && pose_cands_buff[1].obj_err < pose_cands_buff[0].obj_err) {
                    std::swap(pose_cands_buff[0], pose_cands_buff[1]);
                }
            }

            // Expose the raw candidates so downstream consumers can re-select;
            // the detector still picks one via its own selector.
            det.pose_candidates = pose_cands_buff;
            det.num_pose_candidates = static_cast<int>(num_pose_cands);

            const std::span<const tag_pose_t> pose_cands_span{ pose_cands_buff.data(), num_pose_cands };
            const tag_pose_t* const selected_pose = _pose_selector
                ? _pose_selector(det.id, pose_cands_span)
                : selectors::min_error(det.id, pose_cands_span);

            if (selected_pose != nullptr) {
                det.pose = *selected_pose;
            } else {
                // No pose was selected. leave empty.
            }
        }

        return detections;
    }

    void draw_tag_detections(cv::Mat& bgr, std::span<const tag_detection_t> detections)
    {
        for (const auto& d : detections) {
            for (int k = 0; k < 4; ++k) {
                cv::line(bgr, d.corners[k], d.corners[(k + 1) % 4], cv::Scalar{ 0, 255, 0 }, 2);
            }
            cv::circle(bgr, d.corners[0], 4, cv::Scalar{ 0, 0, 255 }, cv::FILLED); // first corner
            cv::putText(bgr, std::format("{}", d.id), d.center,
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar{ 0, 255, 255 }, 2);

            if (d.pose.has_value()) {
                const auto& a = d.pose->axes;
                cv::line(bgr, a[0], a[1], cv::Scalar{ 0, 0, 255 }, 2); // X red
                cv::line(bgr, a[0], a[2], cv::Scalar{ 0, 255, 0 }, 2); // Y green
                cv::line(bgr, a[0], a[3], cv::Scalar{ 255, 0, 0 }, 2); // Z blue
                cv::putText(
                    bgr,
                    std::format("{:.2f}m", d.pose->transform.translation().norm()),
                    d.corners[2],
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar{ 255, 255, 255 }, 2
                );
            }
        }
    }

} // namespace pose
