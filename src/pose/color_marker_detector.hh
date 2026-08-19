#pragma once
#include "joint_measurement.hh"
#include "joints_def.hh"

#include "utils/serializable.hh"

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

// ---------------------------------------------------------------------------
// 색상 마커 검출기
// ---------------------------------------------------------------------------
//
// 하지 관절에 붙인 단색 원형 마커를 영상에서 찾아 그 중심을 서브픽셀로 돌려준다.
// 설계 배경은 docs/color-marker-detection.md 참고.
//
// ## 왜 HSV 가 아니라 Lab 인가
//
// 색을 다룰 때 흔히 쓰는 HSV 는 H(색상)/S(채도)/V(명도) 각각에 최소·최대를 걸어 "상자"를
// 만든다. 그런데 실제 마커 픽셀들을 좌표평면에 찍어 보면 상자가 아니라 **비스듬히 기울어진
// 타원** 모양으로 뭉쳐 있다. 상자로는 이 모양을 담을 수 없어서, 상자를 키우면 배경이 섞여
// 들어오고 줄이면 마커 가장자리가 잘려 나간다.
//
// 그래서 CIE Lab 색공간을 쓴다. Lab 은 색을 세 축으로 나누는데
//
//   L* : 밝기          (얼마나 밝은가)
//   a* : 초록 <-> 빨강 (음수면 초록, 양수면 빨강 쪽)
//   b* : 파랑 <-> 노랑 (음수면 파랑, 양수면 노랑 쪽)
//
// 여기서 **L* 을 버리고 (a*, b*) 평면만 쓴다.** 밝기를 빼고 색감만 남기는 것이라, 그림자가
// 지거나 조명이 조금 어두워져도 같은 색으로 판정된다. 이것이 이 방식의 핵심이다.
//
// ## 다만 밝기에 완전히 불변인 것은 아니다
//
// a*, b* 를 구하는 식에 세제곱근이 들어 있다.
//
//   a* = 500 · ( f(X/Xn) − f(Y/Yn) ) ,   f(t) ≈ t^(1/3)
//
// 밝기가 k 배가 되면 X, Y, Z 가 함께 k 배가 되고, 세제곱근을 지나면서 **a*, b* 가 k^(1/3) 배로
// 이동한다.** 노출을 두 배로 올리면 약 1.26 배, 즉 mean_a 가 +62 였다면 +78 로 간다. 표준편차가
// 3~5 인 모델에서는 몇 표준편차 밖이라 마커를 통째로 놓친다.
//
// 그늘처럼 국소적이고 폭이 작은 변동은 공분산이 흡수하지만, 노출이나 게인을 바꾸는 것은 다른
// 색으로 바꾸는 것과 같다. 그래서 모델은 자기를 적합할 때 쓴 노출·게인과 같은 설정 파일에
// 들어 있고, 그 값을 고치면 모델도 다시 재야 한다.
//
// ## 마할라노비스 거리
//
// 마커 픽셀들을 (a*, b*) 평면에 점으로 찍으면 구름처럼 뭉친다. 새 픽셀 하나가 들어왔을 때
// "이게 저 구름에 속하나" 를 판정해야 하는데, 재는 방법에 따라 결과가 크게 달라진다.
//
//   방법 1. 상자      a* 는 이 범위, b* 는 저 범위. 구름이 기울어져 있으면 상자 모서리에는
//                     점이 없는데도 통과시키고, 상자를 줄이면 구름 끝이 잘린다.
//   방법 2. 자로 재기  중심에서 거리 R 이내(= 원). 구름이 한쪽으로 길면 원으로는 못 맞춘다.
//                     키우면 짧은 쪽으로 배경이 들어오고, 줄이면 긴 쪽으로 마커가 잘린다.
//   방법 3. 마할라노비스   각 방향의 편차를 그 방향의 표준편차로 나눈다. 즉 "몇 표준편차만큼
//                     떨어졌나" 를 재는 것이라 단위가 사라진다.
//
// 축이 기울지 않은 경우라면 방법 3 은 이만큼 단순하다.
//
//   d² = ( (a - ā) / σa )²  +  ( (b - b̄) / σb )²
//
// 구름이 기울어져 있으면(a* 와 b* 가 같이 움직이면) 축별로 나누는 것만으로는 부족한데,
// 공분산의 역행렬이 그 기울기까지 함께 처리한다. 그것을 행렬로 쓴 것이 아래 식이다.
//
//   d² = (p - 평균)ᵀ · 공분산⁻¹ · (p - 평균)
//
// 기하학적으로는 등거리선이 원이 아니라 **표본 구름과 같은 모양, 같은 방향의 타원**이 된다.
// 그래서 "타원을 원으로 펴서 잰 거리" 라고 이해하면 된다. 표본이 넓게 퍼진 방향으로는 멀어도
// 관대하고, 좁은 방향으로는 조금만 벗어나도 멀다고 본다.
//
// 판정 기준이 `max_distance` 하나로 끝난다는 것도 크다. HSV 상자는 H/S/V 최소·최대 여섯 개를
// 맞춰야 하지만 여기서는 **사람이 맞출 설정값이 하나뿐**이고, 그 값이 곧 "몇 표준편차까지
// 인정할 것인가" 라는 명확한 의미를 갖는다.
//
// 2차원 정규분포에서 d < k 일 확률은 1 - exp(-k²/2) 이다.
//
//   k = 2  ->  86.5%      k = 3  ->  98.9%
//
// ## 색 모델은 코드에 박혀 있지 않다
//
// 어떤 색도 상수로 들어 있지 않다. `color_model_t::valid` 는 적합 전까지 false 이고, 그동안
// detect() 는 아무것도 내지 않는다. 시험소마다 조명과 카메라가 달라 같은 마커도 다른 색으로
// 찍히므로, 모델은 반드시 현장에서 찍은 표본에서 나와야 한다.
//
// ## 룩업 테이블
//
// OpenCV 가 8비트 영상을 Lab 으로 바꾸면 a*, b* 가 각각 0~255 정수가 된다. 가능한 (a,b)
// 조합이 256×256 = 65536 가지뿐이므로, **모든 조합의 판정 결과를 미리 표로 만들어 두면**
// 픽셀마다 제곱근이나 행렬 곱을 할 필요가 없다. 표 조회 한 번이면 끝이라 HSV 임계값 비교보다
// 오히려 빠르다. 표는 색 모델이 바뀔 때만 다시 만든다.
namespace pose
{
    // 색 클래스 하나를 정의하는 모델. 마커 픽셀을 표본으로 모아 적합해서 만든다.
    struct color_model_t
    {
        Eigen::Vector2d mean_ab{ 0.0, 0.0 };                   // 표본의 (a*, b*) 평균
        Eigen::Matrix2d cov_ab{ Eigen::Matrix2d::Identity() }; // 표본이 퍼진 모양 (공분산)
        // 마할라노비스 거리가 이 값보다 가까우면 마커 색으로 본다.
        // 정규분포라면 2.0 은 표본의 약 86%, 3.0 은 약 99% 를 덮는다.
        double max_distance{ 3.0 };
        bool valid{ false }; // 적합 전에는 false. 이 상태에서는 검출이 아무것도 내지 않는다

        // NOTE: `valid` 필드는 serializable field 목록에서 제외. (해당 필드는 각 필드에 저장된 값들이 유효한지를 나타내는 상태이므로)
        DECLARE_SERIALIZABLE_FIELDS(
            v("mean_ab",      o.mean_ab);
            v("cov_ab",       o.cov_ab);
            v("max_distance", o.max_distance);
        )
    };

    // 검출된 마커 하나. (NOTE: `center` 가 최종 산출물, 나머지 필드는 디버깅용)
    struct marker_detection_t
    {
        cv::Point2f center{};        // 서브픽셀 중심 [px]
        float diameter_px{ 0.0f };   // 면적에서 환산한 겉보기 지름 [px]
        float score{ 0.0f };         // 소속 점수 평균 (0~1). 1 에 가까울수록 모델 중심에 가깝다
        float area_px{ 0.0f };       // 마스크 상 면적 [px]
        float fill{ 0.0f };          // 면적 / 외접 사각형 면적. 채워진 원이면 약 0.785
        float aspect{ 1.0f };        // 외접 사각형의 긴 변 / 짧은 변. 원이면 1
    };

    // 필터에서 걸러진 후보의 사유별 개수. (왜 안 잡혔는지를 추적하기 위한 디버깅 목적)
    struct marker_reject_stats_t
    {
        int too_small{ 0 };  // 면적이 min_area_px 미만
        int too_large{ 0 };  // 면적이 max_area_px 초과
        int not_filled{ 0 }; // 채움율 미달 (가운데가 뚫렸거나 모양이 찌그러짐)
        int too_long{ 0 };   // 종횡비 초과 (모션 블러로 늘어난 경우가 대부분)
        int low_score{ 0 };  // 색은 맞지만 모델 중심에서 먼, 자신 없는 덩어리
        int total() const { return too_small + too_large + not_filled + too_long + low_score; }
    };

    // ---------------------------------------------------------------------------
    // 표본 수집기
    // ---------------------------------------------------------------------------
    //
    // 화면에서 마커를 클릭하면 그 주변 픽셀을 여기에 모으고, 충분히 모이면 `fit()` 으로
    // 색 모델을 만든다. **여러 프레임에 걸쳐, 다리를 여러 자세로 옮겨 가며 모아야 한다.**
    // 한 자세에서만 뽑으면 실제 운용 중 겪을 조명 변동 폭이 표본에 담기지 않아서, 현장에서
    // 마커를 놓치기 시작한다.
    class color_sampler
    {
    public:
        // `bgr` 의 (center, radius) 원 안 픽셀을 표본에 더한다. 영상 밖은 알아서 잘린다.
        void add(const cv::Mat& bgr, cv::Point center, int radius);

        void clear();

        std::size_t count() const { return _n; }

        // 산점도에 그릴 표본점 (a*, b*). 표시용이라 개수가 제한된다.
        const std::vector<cv::Point2f>& points() const { return _points; }

        // 모은 표본에서 평균과 공분산을 계산해 모델을 만든다.
        // 표본이 너무 적으면(kMinSamples 미만) 비어 있는 값을 돌려준다.
        std::optional<color_model_t> fit(double max_distance) const;

        // 2차원 공분산을 추정하는 데 필요한 최소 표본 수. 지름 20 px 마커를 반지름 6 px 로
        // 한 번 클릭하면 약 110 픽셀이 모이므로, 클릭 한 번으로도 일단 모델이 나온다.
        // **이것은 하한일 뿐이다.** 실제로는 다리를 여러 자세로 옮겨 가며 수천 픽셀을 모아야
        // 운용 중 겪을 조명 변동이 공분산에 담긴다.
        static constexpr std::size_t kMinSamples = 100;

    private:
        // 평균과 공분산은 합만 있으면 구할 수 있어서 픽셀을 통째로 들고 있지 않는다.
        std::size_t _n{ 0 };
        double _sum_a{ 0.0 }, _sum_b{ 0.0 };
        double _sum_aa{ 0.0 }, _sum_ab{ 0.0 }, _sum_bb{ 0.0 };

        std::vector<cv::Point2f> _points; // 산점도 표시용 표본 (kMaxPoints 까지)
        static constexpr std::size_t kMaxPoints = 20000;
    };

    // ---------------------------------------------------------------------------
    // 검출기
    // ---------------------------------------------------------------------------
    class color_marker_detector
    {
    public:
        // 아래 필터들이 왜 필요한가: 색만 맞으면 배경의 붉은 물체나 옷도 함께 걸린다.
        // 색이 맞는 픽셀 덩어리를 찾은 뒤, 그 덩어리가 마커처럼 생겼는지 크기와 모양으로,
        // 그리고 색이 얼마나 확실한지로 한 번 더 거른다.
        // 다섯 게이트 중 하나라도 어긋나면 그 덩어리는 통째로 버려진다.
        //
        // 여기서 말하는 **외접 사각형**은 덩어리를 감싸는 가장 작은 직사각형이다. 원을 직접
        // 찾는 것이 아니라, 찾은 덩어리가 원처럼 생겼는지를 이 사각형으로 검사한다.
        // 원이라면 두 가지가 성립하기 때문이다.
        //
        //   원이면 가로와 세로가 같다            ->  종횡비  약 1.0
        //   원이면 그 사각형의 78.5% 를 채운다   ->  채움율  약 0.785 (= pi/4)
        //
        //   정사각형   종횡비 1.0   채움율 1.00   -> 채움율이 너무 높아 탈락
        //   도넛       종횡비 1.0   채움율 0.35   -> 채움율이 낮아 탈락
        //   줄무늬     종횡비 2.4   채움율 0.78   -> 종횡비가 높아 탈락
        //   원반       종횡비 1.0   채움율 0.785  -> 통과
        //
        // 교과서적인 원형도 공식(4·pi·면적/둘레²)을 쓰지 않는 이유는, 작은 덩어리에서 둘레
        // 추정이 부정확해서다. 픽셀이 계단 모양이라 둘레가 실제보다 길게 나오고 원형도가
        // 과소평가된다. 지름 20 px 수준에서는 이 오차가 크다.
        struct options_t
        {
            color_model_t model{}; // 적합 전에는 valid == false 이고, 그동안 검출은 비어 있다

            // --- 크기: 덩어리가 얼마나 "큰가" ---
            //
            // 지름 d 원의 면적은 pi·d²/4 이므로 지름 20 px 마커는 약 314 px² 이다.
            // 기본값은 지름 8.7 px ~ 87 px 에 해당하는 넓은 밴드다.
            //
            // min: 올리면 점 노이즈가 걸러지고, 내리면 멀거나 일부 가려진 마커가 살아남는다.
            // max: 색이 우연히 맞는 넓은 배경 영역(벽, 옷)을 차단한다.
            double min_area_px{ 60.0 };
            double max_area_px{ 6000.0 };

            // --- 모양: 그 픽셀들이 얼마나 "뭉쳐 있나" ---
            //
            // 채움율 = 덩어리 면적 / 외접 사각형 면적. 크기가 같아도 모양은 다를 수 있어서,
            // 면적만으로는 원반과 도넛을 구분하지 못한다. 이 값이 그 역할을 한다.
            // 광택 마커가 여기서 걸린다면 min_fill 을 내리기 전에 close_kernel_px 를 먼저
            // 올려야 한다. 구멍을 메우는 것이 근본 해결이다.
            double min_fill{ 0.55 };

            // 종횡비 = 외접 사각형의 긴 변 / 짧은 변. 원이면 1 이다.
            // 노출이 길면 스윙하는 마커가 줄무늬로 늘어나 이 값이 커지므로, 노출을 충분히
            // 줄이기 전까지는 넉넉하게 잡아야 한다.
            //   지름 20 px, 이미지 상 속도 3478 px/s 기준
            //   노출 2 ms -> 27x20 -> 1.35        노출 8 ms -> 48x20 -> 2.4
            double max_aspect{ 3.0 };

            // --- 확신: 색이 얼마나 "맞나" ---
            //
            // 덩어리 안 픽셀들의 소속 점수 평균이 이 값에 못 미치면 버린다. 크기와 모양이
            // 우연히 맞은 배경 물체는 대개 모델 타원의 가장자리에 걸쳐 있어서 여기서 걸린다.
            // 올리면 확실한 것만 남고, 마커가 그늘에 들어갔을 때 놓치기 쉬워진다.
            double min_score{ 0.15 };

            // --- 마스크 정리: 두 연산의 목적은 서로 반대다 ---
            //
            // 열림(open, 침식 후 팽창)은 마스크 **밖**의 작은 점을 지운다. 커널보다 작은 점은
            // 침식에서 사라져 돌아오지 않고, 살아남은 덩어리는 팽창으로 원래 크기를 회복한다.
            // 올리면 마스크가 깨끗해지지만 얇은 마커 테두리가 깎인다.
            int open_kernel_px{ 3 };

            // 닫힘(close, 팽창 후 침식)은 마스크 **안**의 구멍을 메운다. 광택 마커는 정반사로
            // 한가운데가 하얗게 날아가 도넛이 되는데, 그대로 두면 하이라이트가 다리 움직임을
            // 따라 이동하면서 무게중심까지 흔들린다.
            // 올리면 큰 구멍도 메우지만 가까이 붙은 덩어리끼리 합쳐진다.
            //
            // 1 이하면 해당 단계를 건너뛴다. 1x1 커널은 마스크를 바꾸지 않는다.
            int close_kernel_px{ 5 };

            DECLARE_SERIALIZABLE_FIELDS(
                v("model",           o.model);
                v("min_area_px",     o.min_area_px);
                v("max_area_px",     o.max_area_px);
                v("min_fill",        o.min_fill);
                v("max_aspect",      o.max_aspect);
                v("min_score",       o.min_score);
                v("open_kernel_px",  o.open_kernel_px);
                v("close_kernel_px", o.close_kernel_px);
            )
        };

        explicit color_marker_detector(const options_t& opt = {});

        options_t& options() noexcept { return _opt; }
        const options_t& options() const noexcept { return _opt; }

        // 색 모델을 갈아 끼우고 룩업 테이블을 다시 만든다.
        void set_model(const color_model_t& model);

        // 옵션을 직접 고친 뒤 모델이 바뀌었다면 이것을 불러 표를 갱신한다.
        void rebuild_lut();

        // 한 프레임에서 마커를 찾는다. `bgr` 은 3채널 컬러여야 한다(흑백은 빈 결과).
        // 면적이 큰 순서로 정렬해서 돌려준다.
        std::vector<marker_detection_t> detect(const cv::Mat& bgr);

        // --- 마지막 detect() 의 부산물 (표시/진단용) ---

        // 픽셀별 소속 점수 (CV_8U). 0 이면 모델 밖, 255 면 모델 중심.
        // 이 값이 서브픽셀 중심을 구할 때의 가중치로도 쓰인다.
        const cv::Mat& score_image() const noexcept { return _score; }

        // 정리까지 끝난 이진 마스크 (CV_8U, 0 또는 255).
        const cv::Mat& mask() const noexcept { return _mask; }

        const marker_reject_stats_t& reject_stats() const noexcept { return _rejects; }

    private:
        void _build_score_image(const cv::Mat& bgr);

        options_t _opt;

        // (a*, b*) 조합마다의 소속 점수를 미리 계산해 둔 표. [a][b] 순서, 256x256.
        std::array<std::uint8_t, 256 * 256> _lut{};

        // 프레임마다 재사용하는 버퍼. 매 프레임 새로 할당하지 않으려고 멤버로 들고 있다.
        cv::Mat _lab, _score, _mask;
        cv::Mat _labels, _stats, _centroids;
        marker_reject_stats_t _rejects{};
    };

    // ---------------------------------------------------------------------------
    // 관절 배정기
    // ---------------------------------------------------------------------------
    //
    // 검출기는 "화면 어딘가에 이런 덩어리가 있다" 까지만 말한다. 그 덩어리가 무릎인지 발목인지는
    // 여기서 정한다. AprilTag 은 태그마다 id 가 찍혀 있어 이 단계가 필요 없지만, 단색 원반은
    // 서로 구별할 표식이 없으므로 **놓인 자리**로 알아내야 한다.
    //
    // ## 어떻게 알아내나
    //
    //   1. 처음 (lock 전)   다리를 따라 늘어선 순서로 배정한다. 골반이 맨 위, 발이 맨 아래다.
    //   2. 그 다음 프레임들  직전 위치에서 search_radius_px 안에 있는 가장 가까운 덩어리를 잇는다.
    //   3. 놓치면            전 구간을 잇지 못한 프레임이 lost_frames_before_full_search 만큼
    //                        연속되면 lock 을 풀고 1 번부터 다시 한다.
    //
    // 2번이 배경 오검출을 거의 없앤다. 화면 구석의 붉은 물체가 크기와 모양까지 통과하더라도,
    // 직전 프레임의 무릎 자리에서 멀면 후보에 들지 못한다.
    //
    // ## 뼈 길이비 검증
    //
    // 로봇은 강체라 골반-무릎, 무릎-발목, 발목-발의 이미지 거리가 걷는 내내 거의 고정이다.
    // 배정이 한 칸 밀리면 "허벅지" 자리에 골반→발목이 들어와 거리가 두 배쯤 되므로 즉시 걸린다.
    //
    // 기준 거리는 `capture_reference()` 로 잡는데, **rest pose 를 보정하는 순간**에 부르도록
    // 되어 있다. 그 순간이 조작자가 화면을 보며 배정이 맞는지 직접 확인하는 유일한 시점이라서다.
    // 기준을 그 프레임에서 뽑으므로 카메라가 비스듬히 설치돼 생긴 왜곡은 기준과 측정 양쪽에
    // 똑같이 실려 상쇄된다. 남는 것은 다리 자세에 따라 변하는 부분뿐이고, 그 폭은 한 칸 밀림이
    // 만드는 차이보다 훨씬 작다.
    class color_marker_assigner
    {
    public:
        struct options_t
        {
            // 마커를 붙인 다리. 색은 어느 다리인지 말해 주지 않으므로 설치가 정해 준다.
            joint_side_t leg{ joint_side_t::left };

            // 인쇄한 원반의 지름 [m]. 겉보기 지름과 함께 미터 스케일이 된다.
            double marker_diameter_m{ 0.018 };

            // 직전 위치에서 이 반경 안의 덩어리만 후보로 본다.
            // 올리면 빠르게 움직여도 따라가고, 배경 오검출이 들어올 여지가 생긴다.
            // 마커가 한 프레임에 움직이는 거리보다 넉넉해야 한다.
            double search_radius_px{ 60.0 };

            // 강체 가정에 기댄 배정 검증. 마커가 휘는 부위에 붙었거나 카메라 거리가 변하는
            // 설치에서는 정상 배정을 계속 거부하게 되므로, 원인을 가를 때 이것을 끈다.
            bool enable_bone_length_check{ true };

            // 기준 거리 대비 허용 폭. 0.35 면 0.65 ~ 1.35 배까지 인정한다.
            // 한 칸 밀림은 0.5 배나 2.0 배쯤으로 나타나므로 이 밴드 밖이다.
            // 카메라가 눈에 띄게 비스듬하면 자세에 따른 변동이 커지므로 0.45 정도로 올린다.
            double bone_length_tolerance{ 0.35 };

            // 전 구간을 잇지 못한 프레임이 이만큼 연속되면 lock 을 풀고 처음부터 다시 찾는다.
            int lost_frames_before_full_search{ 10 };

            DECLARE_SERIALIZABLE_FIELDS(
                v("leg",                            o.leg);
                v("marker_diameter_m",              o.marker_diameter_m);
                v("search_radius_px",               o.search_radius_px);
                v("enable_bone_length_check",       o.enable_bone_length_check);
                v("bone_length_tolerance",          o.bone_length_tolerance);
                v("lost_frames_before_full_search", o.lost_frames_before_full_search);
            )
        };

        // 한 프레임의 배정 결과 요약. (왜 관절이 안 잡혔는지에 대한 디버깅 목적)
        struct stats_t
        {
            int candidates{ 0 };     // 검출기가 넘긴 덩어리 수
            int assigned{ 0 };       // 그중 관절에 배정된 수
            int out_of_radius{ 0 };  // 예측 반경 안에 후보가 없어 이번에 못 이은 슬롯 수
            // 전 구간을 잇지 못한 채 지난 연속 프레임 수. 재탐색에 들어가면서 0 으로 돌아가므로
            // 0 에서 lost_frames_before_full_search 사이를 오간다.
            int lost_frames{ 0 };
            bool locked{ false };    // 배정이 잡혀 추적 중인지
            bool bad_geometry{ false }; // 이번 프레임이 뼈 길이비 검증에서 버려졌는지
            bool has_reference{ false }; // 기준 거리가 잡혀 있는지
        };

        explicit color_marker_assigner(const options_t& opt = {});

        options_t& options() noexcept { return _opt; }
        const options_t& options() const noexcept { return _opt; }

        // 한 프레임의 덩어리들을 관절에 배정한다. 배정된 것만 돌려주므로,
        // 가려진 관절은 결과에서 빠지고 그 처리는 추정기의 홀드가 맡는다.
        std::vector<joint_2d_measurement_t> assign(std::span<const marker_detection_t> detections);

        // 가장 최근에 이어진 위치들의 뼈 거리를 기준으로 삼는다. 전 구간이 이어져 있어야 하며,
        // 아니면 false 를 돌려주고 기존 기준을 그대로 둔다.
        bool capture_reference();
        void clear_reference();

        // lock 과 직전 위치를 버리고 다음 프레임부터 늘어선 순서로 다시 찾는다.
        // 영상이 다른 지점으로 건너뛰어 예측 위치가 더는 다음 프레임을 설명하지 못할 때 부른다.
        // 기준 거리는 남는다. 그것은 `clear_reference()` 로만 버린다.
        void reset();

        const stats_t& stats() const noexcept { return _stats; }

        // 이 배정기가 따라가는 마커 사이트 사슬 (골반 -> 무릎 -> 발목 -> 발). 슬롯 하나가 원반
        // 하나다. 한 사이트에 겹쳐 있는 관절들(골반 마커 위의 hip 분절)은 별도 슬롯 없이 그
        // 사이트의 측정을 함께 받는다.
        std::span<const joint_id_t> chain() const noexcept { return _chain; }

    private:
        void _rebuild_chain();
        void _unlock(); // 다음 프레임이 늘어선 순서부터 다시 찾도록 되돌린다

        options_t _opt;

        std::vector<joint_id_t> _chain;
        joint_side_t _chain_side{ joint_side_t::midline }; // _chain 을 만들 때 쓴 다리

        // 슬롯별 직전 배정 위치. 다음 프레임의 예측 중심이 된다.
        std::vector<std::optional<Eigen::Vector2d>> _last_px;

        // 이웃한 두 슬롯 사이의 기준 거리 [px]. 크기는 사슬 길이 - 1.
        std::vector<double> _reference_px;

        bool _locked{ false };
        int _lost_frames{ 0 };
        stats_t _stats{};
    };

    // 화면 전체의 (a*, b*) 분포를 세어 256x256 히스토그램(CV_32S)으로 돌려준다.
    // 산점도를 그려 마커 색과 배경 색이 얼마나 떨어져 있는지 눈으로 보기 위한 것이다.
    // `step` 은 몇 픽셀 건너 셀지로, 크게 잡으면 빨라지고 분포 모양은 거의 그대로다.
    cv::Mat build_ab_histogram(const cv::Mat& bgr, int step = 4);

    // 검출 결과를 영상 위에 그린다(원 테두리와 중심 십자, 지름 표기).
    void draw_marker_detections(cv::Mat& bgr, const std::vector<marker_detection_t>& detections);

} // namespace pose
