#include "color_marker_detector.hh"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <numbers>
#include <string>

namespace pose
{
    namespace
    {
        // OpenCV 는 8비트 Lab 영상에서 a*, b* 를 128 만큼 올려서 저장한다(음수를 담기 위해).
        // 원래 값으로 되돌릴 때 이 값을 뺀다.
        constexpr double kLabShift = 128.0;

        // 표본이 한 점에 몰리면 공분산이 0 이 되어 역행렬을 구할 수 없다.
        // 최소한의 퍼짐을 더해 그 상황을 막는다. 1.0 은 a*b* 눈금 한 칸이다.
        constexpr double kMinVariance = 1.0;

        double circle_diameter_from_area(double area_px)
        {
            return 2.0 * std::sqrt(area_px / std::numbers::pi);
        }

        // 2x2 대칭 행렬의 역행렬. 공식이 짧아서 직접 쓴다.
        //
        //   | a  b |⁻¹        1     |  d  -b |
        //   | b  d |     =  -------  | -b   a |
        //                   ad - b²
        //
        // 분모(행렬식)가 0 이면 역행렬이 없다. 여기서는 호출 전에 대각선에 최소 퍼짐을 더해
        // 두므로 그럴 일이 없지만, 수치 오차를 대비해 단위 행렬로 물러난다.
        Eigen::Matrix2d invert_symmetric_2x2(double a, double b, double d)
        {
            const double det = a * d - b * b;
            Eigen::Matrix2d inv;
            if (!(std::abs(det) > 1e-12)) {
                inv = Eigen::Matrix2d::Identity();
                return inv;
            }
            inv <<  d / det, -b / det,
                   -b / det,  a / det;
            return inv;
        }
    } // namespace

    // ---------------------------------------------------------------------------
    // color_sampler
    // ---------------------------------------------------------------------------

    void color_sampler::add(const cv::Mat& bgr, cv::Point center, int radius)
    {
        if (bgr.empty() || bgr.channels() < 3 || radius <= 0) { return; }

        // 클릭 지점이 영상 가장자리면 원이 밖으로 나간다. 겹치는 부분만 남긴다.
        const cv::Rect box =
            cv::Rect(center.x - radius, center.y - radius, 2 * radius + 1, 2 * radius + 1)
            & cv::Rect(0, 0, bgr.cols, bgr.rows);
        if (box.empty()) { return; }

        cv::Mat lab;
        cv::cvtColor(bgr(box), lab, cv::COLOR_BGR2Lab);

        const int r2 = radius * radius;
        for (int y = 0; y < lab.rows; ++y)
        {
            const cv::Vec3b* row = lab.ptr<cv::Vec3b>(y);
            const int dy = box.y + y - center.y;
            for (int x = 0; x < lab.cols; ++x)
            {
                // 잘라 온 것은 사각형이므로, 원 밖의 모서리는 여기서 버린다.
                const int dx = box.x + x - center.x;
                if (dx * dx + dy * dy > r2) { continue; }

                const double a = static_cast<double>(row[x][1]) - kLabShift;
                const double b = static_cast<double>(row[x][2]) - kLabShift;

                // 평균과 공분산은 합만 있으면 나오므로 픽셀을 통째로 쌓아 두지 않는다.
                ++_n;
                _sum_a += a;   _sum_b += b;
                _sum_aa += a * a;  _sum_ab += a * b;  _sum_bb += b * b;

                if (_points.size() < kMaxPoints) {
                    _points.emplace_back(static_cast<float>(a), static_cast<float>(b));
                }
            }
        }
    }

    void color_sampler::clear()
    {
        _n = 0;
        _sum_a = _sum_b = _sum_aa = _sum_ab = _sum_bb = 0.0;
        _points.clear();
    }

    std::optional<color_model_t> color_sampler::fit(double max_distance) const
    {
        if (_n < kMinSamples) { return std::nullopt; }

        const double n = static_cast<double>(_n);
        const double mean_a = _sum_a / n;
        const double mean_b = _sum_b / n;

        // 표본 공분산. n 이 아니라 n-1 로 나누는 쪽이 참값에 치우침 없는 추정이다.
        double var_a  = (_sum_aa - n * mean_a * mean_a) / (n - 1.0);
        double var_b  = (_sum_bb - n * mean_b * mean_b) / (n - 1.0);
        double cov_ab = (_sum_ab - n * mean_a * mean_b) / (n - 1.0);

        var_a += kMinVariance;
        var_b += kMinVariance;

        color_model_t model;
        model.mean_ab = Eigen::Vector2d{ mean_a, mean_b };
        model.cov_ab << var_a, cov_ab,
                        cov_ab, var_b;
        model.max_distance = max_distance;
        model.valid = true;
        return model;
    }

    // ---------------------------------------------------------------------------
    // color_marker_detector
    // ---------------------------------------------------------------------------

    color_marker_detector::color_marker_detector(const options_t& opt)
        : _opt{ opt }
    {
        this->rebuild_lut();
    }

    void color_marker_detector::set_model(const color_model_t& model)
    {
        _opt.model = model;
        this->rebuild_lut();
    }

    void color_marker_detector::rebuild_lut()
    {
        _lut.fill(0);
        if (!_opt.model.valid || _opt.model.max_distance <= 0.0) { return; }

        const Eigen::Vector2d& mu = _opt.model.mean_ab;
        const double threshold = _opt.model.max_distance;

        // 아래에서 잴 마할라노비스 거리는 공분산의 역행렬을 쓴다.
        //
        //   d² = (p - 평균)ᵀ · 공분산⁻¹ · (p - 평균)
        //
        // 역행렬을 곱한다는 것은 "표본이 퍼진 만큼 나눈다" 는 뜻이다. 표본이 넓게 퍼진 방향으로는
        // 같은 거리라도 작게 세어지고 좁은 방향으로는 크게 세어져서, 결과적으로 기울어진 타원을
        // 원으로 편 자리에서 재게 된다. 공분산을 그대로 곱하면 정반대가 되어 버린다.
        //
        // 이 행렬은 모델 하나에 대해 고정이고 아래 65536 칸이 전부 같은 것을 쓰므로, 칸마다
        // 다시 구하지 않고 루프 밖에서 한 번만 구한다.
        const Eigen::Matrix2d& cov = _opt.model.cov_ab;
        const Eigen::Matrix2d inv = invert_symmetric_2x2(cov(0, 0), cov(0, 1), cov(1, 1));

        // 가능한 (a, b) 조합이 256x256 뿐이므로 전부 미리 계산해 둔다.
        // 이 표 덕분에 픽셀마다 하는 일이 조회 한 번으로 줄어든다.
        for (int ai = 0; ai < 256; ++ai)
        {
            for (int bi = 0; bi < 256; ++bi)
            {
                const Eigen::Vector2d d{ ai - kLabShift - mu.x(), bi - kLabShift - mu.y() };

                // 마할라노비스 거리: 표본이 퍼진 모양(타원)을 원으로 펴서 잰 거리.
                const double dist = std::sqrt(std::max(0.0, d.dot(inv * d)));

                // 모델 중심에서 1, 임계 거리에서 0 으로 내려가는 점수.
                // 마스크(점수 > 0)와 서브픽셀 가중치를 이 값 하나로 함께 쓴다.
                const double w = 1.0 - dist / threshold;
                _lut[static_cast<std::size_t>(ai) * 256 + static_cast<std::size_t>(bi)] =
                    (w > 0.0) ? static_cast<std::uint8_t>(std::lround(w * 255.0)) : std::uint8_t{ 0 };
            }
        }
    }

    void color_marker_detector::_build_score_image(const cv::Mat& bgr)
    {
        cv::cvtColor(bgr, _lab, cv::COLOR_BGR2Lab);
        _score.create(_lab.size(), CV_8UC1);

        for (int y = 0; y < _lab.rows; ++y)
        {
            const cv::Vec3b* src = _lab.ptr<cv::Vec3b>(y);
            std::uint8_t* dst = _score.ptr<std::uint8_t>(y);
            for (int x = 0; x < _lab.cols; ++x)
            {
                // src[x] 는 (L, a, b). L(밝기)은 쓰지 않는다. 그림자와 노출 변동을
                // 무시하려고 일부러 버리는 것이다.
                dst[x] = _lut[static_cast<std::size_t>(src[x][1]) * 256 + static_cast<std::size_t>(src[x][2])];
            }
        }
    }

    std::vector<marker_detection_t> color_marker_detector::detect(const cv::Mat& bgr)
    {
        _rejects = marker_reject_stats_t{};
        std::vector<marker_detection_t> out;

        // 색으로 판정하는 검출기이므로 컬러 프레임이 아니면 할 수 있는 일이 없다.
        // 색 모델을 아직 적합하지 않았을 때도 마찬가지다.
        if (bgr.empty() || bgr.channels() < 3 || !_opt.model.valid) {
            _score.release();
            _mask.release();
            return out;
        }

        this->_build_score_image(bgr);

        // 점수가 0 보다 크면 모델 안쪽이다. 여기서 이진 마스크가 된다.
        cv::threshold(_score, _mask, 0.0, 255.0, cv::THRESH_BINARY);

        // 열림(open) = 침식 후 팽창. 작은 점 노이즈는 침식에서 사라지고,
        // 살아남은 덩어리는 팽창으로 원래 크기를 되찾는다.
        if (_opt.open_kernel_px > 1) {
            const cv::Mat k = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(_opt.open_kernel_px, _opt.open_kernel_px));
            cv::morphologyEx(_mask, _mask, cv::MORPH_OPEN, k);
        }
        // 닫힘(close) = 팽창 후 침식. 덩어리 안의 구멍을 메운다.
        // 마커에 광택이 있으면 정반사로 한가운데가 하얗게 날아가 구멍이 생기는데, 그것을 메워야
        // 무게중심이 도넛 모양 때문에 흔들리지 않는다.
        if (_opt.close_kernel_px > 1) {
            const cv::Mat k = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(_opt.close_kernel_px, _opt.close_kernel_px));
            cv::morphologyEx(_mask, _mask, cv::MORPH_CLOSE, k);
        }

        // 서로 붙어 있는 흰 픽셀들을 하나의 덩어리로 묶어 번호를 매기고,
        // 덩어리마다 면적과 외접 사각형을 함께 받는다.
        const int num_labels = cv::connectedComponentsWithStats(
            _mask, _labels, _stats, _centroids, 8, CV_32S);

        for (int i = 1; i < num_labels; ++i) // 0 번은 배경이라 건너뛴다
        {
            const double area = static_cast<double>(_stats.at<int>(i, cv::CC_STAT_AREA));
            if (area < _opt.min_area_px) { ++_rejects.too_small; continue; }
            if (area > _opt.max_area_px) { ++_rejects.too_large; continue; }

            const int bx = _stats.at<int>(i, cv::CC_STAT_LEFT);
            const int by = _stats.at<int>(i, cv::CC_STAT_TOP);
            const int bw = _stats.at<int>(i, cv::CC_STAT_WIDTH);
            const int bh = _stats.at<int>(i, cv::CC_STAT_HEIGHT);

            // 채움율: 덩어리가 외접 사각형을 얼마나 채우는가.
            // 꽉 찬 원이면 pi/4 = 0.785 정도가 나온다. 이보다 많이 낮으면 속이 비었거나
            // 모양이 찌그러진 것이다.
            const double fill = area / static_cast<double>(bw * bh);
            if (fill < _opt.min_fill) { ++_rejects.not_filled; continue; }

            // 종횡비: 원이면 1 이다. 노출이 길면 스윙하는 마커가 줄무늬로 늘어나 이 값이 커진다.
            const double aspect = static_cast<double>(std::max(bw, bh))
                                / static_cast<double>(std::max(1, std::min(bw, bh)));
            if (aspect > _opt.max_aspect) { ++_rejects.too_long; continue; }

            // 서브픽셀 중심. 픽셀을 0/1 로만 세지 않고 소속 점수를 가중치로 쓴다.
            // 경계에 걸친 어중간한 픽셀이 부분적으로만 반영되므로, 마커가 반 픽셀 움직였을 때
            // 중심도 반 픽셀만 움직인다. 정수 단위로 튀지 않는다는 뜻이다.
            double weight_sum = 0.0, weighted_x = 0.0, weighted_y = 0.0;
            for (int y = by; y < by + bh; ++y)
            {
                const int* label_row = _labels.ptr<int>(y);
                const std::uint8_t* score_row = _score.ptr<std::uint8_t>(y);
                for (int x = bx; x < bx + bw; ++x)
                {
                    // 외접 사각형 안에 다른 덩어리가 끼어 있을 수 있으므로 번호를 확인한다.
                    if (label_row[x] != i) { continue; }
                    const double w = static_cast<double>(score_row[x]);
                    weight_sum += w;
                    weighted_x += w * x;
                    weighted_y += w * y;
                }
            }
            if (weight_sum <= 0.0) { continue; }

            // 소속 점수 평균. 색이 얼마나 확실한가를 하나의 값으로 요약한 것이다.
            const double score = weight_sum / (area * 255.0);
            if (score < _opt.min_score) { ++_rejects.low_score; continue; }

            marker_detection_t d;
            d.center = cv::Point2f(static_cast<float>(weighted_x / weight_sum),
                                   static_cast<float>(weighted_y / weight_sum));
            d.area_px = static_cast<float>(area);
            d.diameter_px = static_cast<float>(circle_diameter_from_area(area));
            d.score = static_cast<float>(score);
            d.fill = static_cast<float>(fill);
            d.aspect = static_cast<float>(aspect);
            out.push_back(d);
        }

        // 큰 것부터. 마커가 배경 잡음보다 큰 것이 보통이라 눈으로 볼 때 편하다.
        std::ranges::sort(out, std::ranges::greater{}, &marker_detection_t::area_px);
        return out;
    }

    // ---------------------------------------------------------------------------
    // color_marker_assigner
    // ---------------------------------------------------------------------------

    color_marker_assigner::color_marker_assigner(const options_t& opt)
        : _opt{ opt }
    {
        this->_rebuild_chain();
    }

    void color_marker_assigner::_rebuild_chain()
    {
        _chain.clear();
        _chain.push_back(get_root_joint());
        for (auto j = get_leg_root_joint(_opt.leg); j.has_value(); j = get_child_joint(j.value())) {
            _chain.push_back(j.value());
        }
        _chain_side = _opt.leg;

        _last_px.assign(_chain.size(), std::nullopt);
        _reference_px.clear();
        _locked = false;
        _lost_frames = 0;
    }

    void color_marker_assigner::reset()
    {
        // 기준 거리는 여기서 건드리지 않는다. 그것은 rest pose 와 짝지어 잡고 짝지어 버리는
        // 값이라 `clear_reference()` 가 맡고, 프레임이 튄 것만으로 뼈 길이가 달라지지는 않는다.
        _last_px.assign(_chain.size(), std::nullopt);
        _locked = false;
        _lost_frames = 0;
        _stats = stats_t{};
    }

    void color_marker_assigner::clear_reference()
    {
        _reference_px.clear();
    }

    bool color_marker_assigner::capture_reference()
    {
        if (_chain.size() < 2) { return false; }

        // 전 구간이 배정돼 있어야 기준이 된다. 한 칸이라도 비면 그 뼈의 기준이 없어서,
        // 이후 그 뼈만 검증에서 빠지는 어중간한 상태가 된다.
        for (const auto& px : _last_px) {
            if (!px.has_value()) { return false; }
        }

        _reference_px.resize(_chain.size() - 1);
        for (std::size_t i = 0; i + 1 < _chain.size(); ++i) {
            _reference_px[i] = (_last_px[i + 1].value() - _last_px[i].value()).norm();
        }
        return true;
    }

    std::vector<joint_2d_measurement_t> color_marker_assigner::assign(
        const std::span<const marker_detection_t> detections)
    {
        if (_opt.leg != _chain_side) { this->_rebuild_chain(); }

        const std::size_t slots = _chain.size();
        _stats = stats_t{};
        _stats.candidates = static_cast<int>(detections.size());
        _stats.has_reference = !_reference_px.empty();

        std::vector<joint_2d_measurement_t> out;
        if (slots < 2 || detections.empty()) {
            ++_lost_frames;
            if (_lost_frames > _opt.lost_frames_before_full_search) { this->_unlock(); }
            _stats.locked = _locked;
            _stats.lost_frames = _lost_frames;
            return out;
        }

        // 이번 프레임에 슬롯마다 어느 덩어리가 붙었나. 비어 있으면 그 관절은 이번에 안 나온다.
        std::vector<const marker_detection_t*> picked(slots, nullptr);

        if (!_locked)
        {
            // --- 처음 배정: 다리를 따라 늘어선 순서로 ---
            // 슬롯 수만큼은 있어야 순서를 셀 수 있다. 그보다 많으면 색이 가장 확실한 것들만
            // 남기는데, 크기와 모양까지 통과한 배경 물체를 가려내는 마지막 잣대가 색이라서다.
            if (detections.size() < slots) {
                ++_lost_frames;
                if (_lost_frames > _opt.lost_frames_before_full_search) { this->_unlock(); }
                _stats.locked = false;
                _stats.lost_frames = _lost_frames;
                return out;
            }

            std::vector<const marker_detection_t*> best;
            best.reserve(detections.size());
            for (const auto& d : detections) { best.push_back(&d); }
            std::ranges::partial_sort(best, best.begin() + static_cast<std::ptrdiff_t>(slots),
                std::ranges::greater{}, [](const marker_detection_t* d) { return d->score; });
            best.resize(slots);

            // 다리는 화면에서 위아래로 늘어서 있으므로 y 오름차순이 곧 골반부터 발까지다.
            std::ranges::sort(best, {}, [](const marker_detection_t* d) { return d->center.y; });

            for (std::size_t i = 0; i < slots; ++i) { picked[i] = best[i]; }
        }
        else
        {
            // --- 추적: 직전 위치에서 가장 가까운 덩어리 ---
            // 슬롯 순서대로 집어 가며 이미 쓴 덩어리는 건너뛴다. 두 슬롯이 같은 덩어리를
            // 두고 다투는 일은 마커 간격이 반경보다 넓은 한 생기지 않는다.
            std::vector<bool> used(detections.size(), false);
            const double radius2 = _opt.search_radius_px * _opt.search_radius_px;

            for (std::size_t i = 0; i < slots; ++i)
            {
                if (!_last_px[i].has_value()) { continue; }
                const Eigen::Vector2d& prev = _last_px[i].value();

                double best_d2 = radius2;
                std::size_t best_k = detections.size();
                for (std::size_t k = 0; k < detections.size(); ++k)
                {
                    if (used[k]) { continue; }
                    const Eigen::Vector2d c{ detections[k].center.x, detections[k].center.y };
                    if (const double d2 = (c - prev).squaredNorm(); d2 < best_d2) {
                        best_d2 = d2;
                        best_k = k;
                    }
                }

                if (best_k == detections.size()) { ++_stats.out_of_radius; continue; }
                used[best_k] = true;
                picked[i] = &detections[best_k];
            }
        }

        // --- 뼈 길이비 검증 ---
        // 기준이 있고 검사가 켜져 있을 때만 돈다. 한 군데라도 밴드를 벗어나면 이번 프레임의
        // 배정을 통째로 버린다. 한 칸 밀림은 사슬 전체를 어긋나게 하므로 부분만 살릴 수 없다.
        if (_opt.enable_bone_length_check && !_reference_px.empty())
        {
            const double lo = 1.0 - _opt.bone_length_tolerance;
            const double hi = 1.0 + _opt.bone_length_tolerance;

            for (std::size_t i = 0; i + 1 < slots; ++i)
            {
                if (!picked[i] || !picked[i + 1]) { continue; } // 한쪽이 비면 잴 것이 없다
                if (_reference_px[i] < 1e-6) { continue; }

                const Eigen::Vector2d p{ picked[i]->center.x, picked[i]->center.y };
                const Eigen::Vector2d q{ picked[i + 1]->center.x, picked[i + 1]->center.y };
                const double ratio = (q - p).norm() / _reference_px[i];
                if (ratio < lo || ratio > hi) { _stats.bad_geometry = true; break; }
            }

            if (_stats.bad_geometry) {
                ++_lost_frames;
                if (_lost_frames > _opt.lost_frames_before_full_search) { this->_unlock(); }
                _stats.locked = _locked;
                _stats.lost_frames = _lost_frames;
                return out; // 직전 위치는 그대로 두어 다음 프레임이 같은 자리에서 다시 시도한다
            }
        }

        // --- 결과 확정 ---
        // 못 이은 슬롯은 직전 위치를 **그대로 둔다.** 그것이 다음 프레임의 예측 중심이라,
        // 잠깐 가려졌던 마커가 다시 나타나면 같은 자리에서 도로 붙는다. 지워 버리면 lock 이
        // 풀릴 때까지 그 관절을 영영 되찾지 못한다.
        out.reserve(slots);
        for (std::size_t i = 0; i < slots; ++i)
        {
            if (!picked[i]) { continue; }

            const Eigen::Vector2d c{ picked[i]->center.x, picked[i]->center.y };
            _last_px[i] = c;

            joint_2d_measurement_t m{};
            m.joint_id = _chain[i];
            m.center_px = c;
            if (picked[i]->diameter_px > 1e-6f) {
                m.meters_per_pixel = _opt.marker_diameter_m / picked[i]->diameter_px;
            }
            out.push_back(m);
            ++_stats.assigned;
        }

        // lock 은 전 구간이 이어진 프레임에서만 선다. 한 칸이라도 비면 계수기가 올라가는데,
        // 한 관절이 계속 안 잡힌다는 것은 배정 자체가 의심스럽다는 뜻이므로 그 상태가
        // 이어지면 lock 을 풀고 늘어선 순서부터 다시 찾는다. 그 사이에도 이어진 슬롯은
        // 계속 나가므로, 잠깐 가려진 관절 하나가 나머지를 끊지는 않는다.
        if (_stats.assigned == static_cast<int>(slots)) {
            _locked = true;
            _lost_frames = 0;
        } else {
            ++_lost_frames;
            if (_lost_frames > _opt.lost_frames_before_full_search) { this->_unlock(); }
        }

        _stats.locked = _locked;
        _stats.lost_frames = _lost_frames;
        return out;
    }

    void color_marker_assigner::_unlock()
    {
        _locked = false;
        _lost_frames = 0;
        _last_px.assign(_chain.size(), std::nullopt);
    }

    // ---------------------------------------------------------------------------
    // 자유 함수
    // ---------------------------------------------------------------------------

    cv::Mat build_ab_histogram(const cv::Mat& bgr, int step)
    {
        // 행이 a*, 열이 b* 다. 둘 다 저장값 기준(0~255)이라 실제 값은 128 을 뺀 것이다.
        cv::Mat hist = cv::Mat::zeros(256, 256, CV_32S);
        if (bgr.empty() || bgr.channels() < 3) { return hist; }

        step = std::max(1, step);

        cv::Mat lab;
        cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
        for (int y = 0; y < lab.rows; y += step)
        {
            const cv::Vec3b* row = lab.ptr<cv::Vec3b>(y);
            for (int x = 0; x < lab.cols; x += step)
            {
                hist.at<int>(row[x][1], row[x][2]) += 1;
            }
        }
        return hist;
    }

    void draw_marker_detections(cv::Mat& bgr, const std::vector<marker_detection_t>& detections)
    {
        if (bgr.empty() || bgr.channels() < 3) { return; }

        const cv::Scalar outline{ 0, 255, 0 };   // 초록: 검출된 원
        const cv::Scalar crosshair{ 0, 255, 255 }; // 노랑: 중심

        for (const marker_detection_t& d : detections)
        {
            const cv::Point c{ cvRound(d.center.x), cvRound(d.center.y) };
            const int r = std::max(3, cvRound(d.diameter_px * 0.5f));

            cv::circle(bgr, c, r, outline, 2, cv::LINE_AA);
            cv::drawMarker(bgr, c, crosshair, cv::MARKER_CROSS, 14, 1, cv::LINE_AA);

            const std::string label = std::format("{:.1f}px {:.2f}", d.diameter_px, d.score);
            cv::putText(bgr, label, cv::Point(c.x + r + 5, c.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, outline, 1, cv::LINE_AA);
        }
    }

} // namespace pose
