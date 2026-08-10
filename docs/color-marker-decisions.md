# 색 마커 경로 결정 기록

**이 문서만 예외적으로 이력을 담는다.** 저장소의 다른 문서와 주석은 "지금 코드가 무엇을 하는가"
만 쓰지만, 여기는 **무엇을 고려했고 왜 안 골랐는지**를 남긴다. 그럴듯해 보이지만 이미 검토하고
버린 안이 여럿이라, 기록이 없으면 같은 제안이 반복된다.

현재 설계가 무엇인지는 [color-marker-detection.md](color-marker-detection.md) 를 본다.
알고리즘 설명은 [color-marker-algorithm.md](color-marker-algorithm.md) 에 있다.

---

## 1. 되돌린 결정

**아래 방향으로 다시 제안하지 말 것.** 각 행의 근거가 지금도 유효하다.

| 항목 | 검토했던 안 | 채택 | 왜 |
|---|---|---|---|
| 마커 색 개수 | 2색 교대 `A,B,A,B` | **1색** | 대상이 1-DOF 힌지 강체 로봇이라 마커 간 이미지 거리가 걷는 내내 불변. 색 패턴이 하던 배정 검증을 뼈 길이비가 대신한다. 색을 늘리면 시험소마다 적합할 모델이 하나 늘고, 두 클래스의 Lab 구름이 겹치면 검증 장치가 고장 원인이 된다 |
| 검출기 다중 클래스 | `models` 벡터 + 클래스 LUT 2장 + 블롭별 다수결 | **단일 모델** | N=1 에서 downstream(joints_def 색 열, config 배열, 배정기의 클래스 일치)이 전부 놀았다. 늘리는 건 나중에 additive 로 가능 |
| 초기 배정 정렬축 | 네 점의 주축(PCA, roll 불변) | **영상 y 오름차순** | 카메라가 비스듬하다는 얘기는 **뼈 길이 관점**이었고 순서는 y 로 재는 게 맞다. 없는 문제를 푼 것 |
| 트래커 타입 접근 | `apriltag_tracker()` / `color_marker_tracker()` 캐시 포인터 | **`dynamic_cast`** | 캐시 포인터는 `_tracker` 가 이미 아는 사실의 복제본이고, 종류마다 멤버와 접근자가 늘어 추상화 목적과 어긋난다 |
| 측정 차원 결정 시점 | `process_frame` 시점 | **take 시점** (`try_get_2d/3d_measurements`) | 앞에서 정하면 트래커가 "둘 중 하나는 항상 빔" 포크를 안고 측정값을 저장하게 된다. 차원은 소비자(추정기)의 성질이다 |
| `detection_frame_t` | 두 기술을 제네릭하게 담는 컨테이너 | **제거** | 트래커가 자기 래치를 갖고, GUI 는 `last_detections()` / `reject_stats()` 로 직접 읽는다 |
| annotate | 별도 가상 함수 | **`process_frame()` 의 out 인자** | "detect 직후 같은 스레드에서" 라는 글로 안 쓰인 전제를 구조로 강제한다 |
| `detection_noun()` | 인터페이스 가상 함수 | **제거** | 로그 단어 하나가 인터페이스 멤버로 앉아 있었다 |
| 함수명 | `detect()` / `take_2d/3d()` | **`process_frame()` / `try_get_2d/3d_measurements()`** | `detect` 는 결과를 돌려줄 것처럼 읽히는데 void. `try_get` 은 코드베이스가 이미 쓰는 말 |
| 색 캘리브레이션 위치 | 프로파일 인라인 → 별도 캘리브 파일(`model_file` 경로) | **프로파일 안 `calibration` 블록** | 별도 파일의 근거는 "다른 도구(vzcam-test)가 쓴다" 였는데, 측정이 디버거로 들어오면서 재는 앱과 프로파일을 읽고 쓰는 앱이 같아졌다 |
| `fitted_under` | source / exposure / gain / samples / note / frame_resolution | **`frame_resolution` 만** | 앞 셋은 바로 위 `camera` 블록의 복사본이고, 한 파일이면 대조할 두 벌이 아예 안 생긴다. `samples`·`note` 는 필요 없다고 판단 |
| `camera.vz` 하위 블록 | 벤더 전용 노브를 백엔드별 팔에 넣기 | **안 함** | WB·BlackLevel 을 config 로 빼지 않기로 하면서 남는 VZ 전용 필드가 `intrinsics_file` 하나뿐이라 구조를 바꿀 값이 없다 |
| `camera_config_t` 전면 교체 | `hw::source_config_t` variant 를 그대로 직렬화 | **안 함** | 둘은 계층이 아니라 **authored 형태와 resolved 형태**다. `intrinsics_file`(경로) vs `intrinsic`(파싱된 행렬), 정수 us vs 소수 us. 합치면 3x3 행렬이 프로파일로 돌아오고, hw 내부 리팩토링이 설정 파일 포맷을 깨게 된다 |
| 화이트밸런스·블랙레벨 | config 옵션으로 노출 | **백엔드가 이름으로 고정** | 복잡도. `vz_frame_source::open()` 이 `LightSourcePreset` 등 카메라가 스스로 이름 붙인 상태로 눌러 둔다 |
| 화이트밸런스 비율 | 세 채널을 1.0 항등으로 | **`LightSourcePreset` 프리셋** | Bayer 센서는 초록에 훨씬 세게 반응해서, 비율을 평평하게 하면 화면이 초록으로 치우친다. 방을 닮은 비율은 벤더가 말할 값이다 |
| `BlackLevel` | 0 으로 고정 | **고정하지 않고 open 로그에 찍음** | 카메라가 중립이라 부르는 값을 안 내놓는다. 여기서 지어내면 같은 카메라를 보는 다른 도구와 화면이 달라진다 |
| provider 읽기 seam | `get_capture_conditions()` 로 노브 상태를 읽어 기록 | **철회** | 문제는 읽을 수 없다는 게 아니라 **앱이 정하지 않는다는 것**이었다. 정하게 만들면 기록할 게 없다 |
| JSON 주석 키 | 샘플 파일 상단의 `_note` | **제거** | `save_config()` 가 스키마에서 다시 만들기 때문에 저장 한 번에 날아간다. 권할 게 아니라 함정 |

## 2. 되돌리지 않은 지적

사용자가 제기했고 논의 끝에 **현행 유지**로 남은 것들이다.

- **"`take_` 를 observer 로 옮기면?"** → 유지. observer 가 측정 타입으로 갈라져야 하고(템플릿 또는
  클래스 분할), 배정기가 프레임 스레드로 가면 `capture_reference()` 가 스테이징이 된다.
- **"GUI 때문에 근본 구조에 코드스멜을 남기나?"** → 아니다. 스레드 간 래치는 GUI 가 아니라
  **검출이 프레임 스레드, 추정이 루프 스레드**라서 생긴다. GUI 표시를 전부 지워도
  `process_frame` / `try_get_*` 분리와 `_published` 래치는 남는다.

## 3. 남아 있는 구멍 (의도적으로 안 고침)

| 항목 | 상태 | 판단 |
|---|---|---|
| `marker_tracker_base::reset()` 이 `latest_value_latch` 를 안 비움 | seek 직후 옛 프레임 1장이 옛 타임스탬프로 소비된다. 색 배정은 최대 `lost_frames_before_full_search`(기본 10) 프레임 공백 후 자동 복구 | **트래커 안에서는 못 닫힌다.** 워커가 `_source_mtx` 를 놓은 뒤 옵저버를 부르므로, 래치를 비운 직후 seek 이전 프레임이 다시 들어올 수 있다. 제대로 닫으려면 `sensor_frame_provider` 에 seek epoch 을 두고 워커가 stale 프레임을 버려야 한다(약 5줄). 실기 검증에서 거슬리면 별건으로 |
| `_do_save_config` 가 해석된 절대 `intrinsics_file` 경로를 써넣음 | 프로파일을 GUI 로 한 번 저장하면 상대 경로가 절대 경로가 된다 | 사용자 판단: 수동으로 고친다 |
| `color_sampler::_points` | 표본점을 최대 2만 개 모으지만 읽는 곳이 없다 | a\*b\* 산점도 UI 가 보류 중이라 그 UI 를 붙이면 바로 쓰인다 |
| `assign()` 미lock 경로의 후보 부족 시 계수기 | 두 조기 return 의 `_unlock()` 처리를 맞춰 순환하게 해 둠 | 해결됨 |

보류한 기능(색 다중 클래스, von Kries 조명 보정, a\*b\* 산점도, 트레이스 블롭 원시 데이터)은
`color-marker-detection.md` §11 의 표에 근거와 함께 있다.

## 4. 검증 상태

**실기 검증은 아직 안 했다.** 순서는 `color-marker-detection.md` §11 의 단계 계획을 따른다.

1. 기존 AprilTag sagittal 녹화를 열어 회귀 확인
2. 마커 시트(`tools/marker-sheet/markers.pdf`) 부착, VZ 로 짧게 녹화 (**`raw` 코덱**, ROI 로 다리
   영역만). JPEG 는 크로마 서브샘플링 때문에 색 튜닝용으로 부적합하고, 디버거가 경고를 띄운다
3. 디버거로 프로파일을 열어 샘플링 → Fit → Save Config As
4. **수직 순서 가정 확인** — 보행 주기 어디서도 발목이 무릎 위로 올라가지 않는지, 뼈 길이비가
   밴드 안인지. 배정이 안 잡히면 `Bone length check` 를 껐다 켜 원인을 가른다
5. rest pose 캡처 후 sagittal angles 플롯 확인
6. `serve` + webview
7. 성능: 5 MP 컬러에서 검출 ms 계측, 30 fps 예산(33 ms) 넘으면 `camera.roi` 로 자르기

`configs/` 는 현재 비어 있다. 프로파일은 `--dump-config` 로 뼈대를 뽑아 만든다.
