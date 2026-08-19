# 하지 관절 각도 측정 컨벤션 (Lower Limb Angle Conventions)

본 문서는 이 프로젝트의 프로토콜이 실어 나르는 하지(Hip, Knee, Ankle) 시상면(Sagittal Plane) 관절 각도의 컨벤션을 정의한다.  
세 각도 모두 생체역학·보행 분석에서 쓰이는 표준 컨벤션을 그대로 따르며, 본 문서는 그 표준이 어느 필드에 어떤 부호로 실리는지를 확정한다.

> `proto/exo_pose_proto.fbs` 의 `JointPose` 가 아래 세 가지 각도 컨벤션을 필드 하나씩으로 싣는다.  
> (`sagittal_segment_angle`, `sagittal_clinical_angle`, `sagittal_included_angle`)  
> 프로토콜 상의 표현 단위는 radian이지만, 본 문서에서는 읽기 편하도록 degree로 설명한다.

## 1. 각도 표현 컨벤션의 종류

### A. Segment Angle
- **정의:** 외부 공간의 고정된 전역 좌표계(Global 좌표축, 주로 중력 벡터)를 기준으로 단일 분절(뼈)이 기울어진 절대적인 각도.
- **용도:** IMU 센서 데이터의 원시(Raw) 방향 값 또는 월드 트랜스폼(World Transform)을 나타낼 때 사용.

### B. Clinical Joint Angle (Neutral Zero Method)
- **정의:** 인접한 두 분절 사이의 상대 각도. 해부학적으로 똑바로 선 자세를 0°(Neutral Zero Method)로 기준 삼아, 자세의 굽힘 정도(Deviation)를 측정.
- **용도:** 생체역학 논문, 보행 분석(Gait Analysis) 결과 시각화, 의료 데이터 분석.

### C. Included Angle (Intersegmental Angle)
- **정의:** 인접한 두 뼈의 장축 사이 각도를 나타내는, 부호 있는 연속량. 두 뼈가 직선일 때 180°이며, 굴곡(Flexion)하면 180°보다 작아지고 신전(Extension)하면 180°를 넘어 연속적으로 커진다.
- **용도:** 기구학적 연산 및 기하학적 기준 지표. 연속량이라 굴곡/신전 방향이 값 자체에 보존된다.
> **NOTE:**
> - 두 뼈가 직선인 것과 중립 자세인 것은 다르다. Hip/Knee는 중립에서 두 뼈가 직선이라 `180°`지만, Ankle의 중립은 발이 정강이와 직각이라 `90°` 다. (Ankle에서 `180°`는 발끝을 정강이와 일직선이 되도록 편 극단 자세를 의미)
> - 순수 기하학적 내각(0°~180° 범위)이 필요하면 180°를 넘는 값을 `360° − θ_inc`로 접는다. (e.g: Hip 15° Extension의 `195°`는 기하학적 내각으로는 `165°`)

---

## 2. 전역 좌표계 및 기준 축 정의 (Kinematic Convention)

수식 적용을 위한 2D 시상면(Sagittal plane) 좌표계 기준이다.
- **기준 축 (0°):** 수직 아래 방향 (Gravity Down Vector)
- **회전 부호:** 앞쪽(Anterior)으로 기울면 `(+)`, 뒤쪽(Posterior)으로 기울면 `(-)`
- **분절 방향 벡터:**
    - Thigh/Shank/Foot는 근위(Proximal)에서 원위(Distal)로 향하는 벡터로 잰다. (Hip→Knee, Knee→Ankle, Ankle→Toe)
    - Pelvis는 몸통 축의 하향 연장선 방향으로 잰다.
    - 몸통이 앞으로 기울면 그 하향 연장선은 뒤로 넘어가므로 `θ_p`가 음수가 되고, `θ_hip = θ_t − θ_p`가 굴곡 `(+)`으로 옳게 나온다. 방향을 반대로 잡으면 Hip의 부호가 뒤집힌다.
- **Segments 기호:**
    - `θ_p`: Pelvis 분절 각도
    - `θ_t`: Thigh 분절 각도
    - `θ_s`: Shank 분절 각도
    - `θ_f`: Foot 분절 각도 (서 있을 때 앞으로 직각이므로 중립값이 `+90°`)

---

## 3. 분절 각도 기반 변환 공식 (Conversion Formulas)

2절의 분절 각도(`θ_p`, `θ_t`, `θ_s`, `θ_f`)에서 Clinical Joint Angle과 Included Angle을 계산하는 공식이다.  
관절 각도는 `θ_hip` / `θ_knee` / `θ_ankle`, Included Angle은 `θ_inc`로 적는다.

| Joint | Segments | Segment -> Clinical | Clinical -> Included | Segment -> Included |
| :--- | :--- | :--- | :--- | :--- |
| **Hip** | Pelvis (`θ_p`), Thigh (`θ_t`) | `θ_hip = θ_t - θ_p` | `θ_inc = 180° - θ_hip` | `θ_inc = 180° - θ_t + θ_p` |
| **Knee** | Thigh (`θ_t`), Shank (`θ_s`) | `θ_knee = θ_t - θ_s` | `θ_inc = 180° - θ_knee` | `θ_inc = 180° - θ_t + θ_s` |
| **Ankle** | Shank (`θ_s`), Foot (`θ_f`) | `θ_ankle = θ_f - θ_s - 90°` | `θ_inc = 90° - θ_ankle` | `θ_inc = 180° - θ_f + θ_s` |

> **부호 의미:**
> Hip/Knee는 양수(+)가 Flexion, 음수(-)가 Extension이다.
> Ankle은 양수(+)가 Dorsiflexion, 음수(-)가 Plantarflexion이다.

---

## 4. 상태별 수치 매핑표 (State Numerical Examples)

앞 절의 공식을 실제 예시 자세에 적용한 매핑표.  
각 관절의 주요 움직임에 따른 세 각도의 수치 변화이며, 직관적인 비교를 위해 상위 분절(근위부)이 `0°`로 고정되어 있다고 가정한다.  
각 관절의 중립 자세(Neutral)는 리그의 T-Pose에 해당한다.

| Joint | Posture & Movement | Segment Angle | Clinical Joint Angle | Included Angle |
| :--- | :--- | :--- | :--- | :--- |
| **Hip** | 올곧게 선 자세 (Neutral) | `θ_p = 0°`, `θ_t = 0°` | **`0°`** | **`180°`** |
| | 다리를 앞으로 직각 들어올림<br>*(90° Flexion, High Knee / Leg Raise)* | `θ_p = 0°`, `θ_t = +90°` | `+90°` (Flexion) | `90°` |
| | 다리를 뒤로 15° 뻗음<br>*(15° Extension, Standing Glute Kickback)* | `θ_p = 0°`, `θ_t = -15°` | `-15°` (Extension) | `195°` |
| **Knee** | 올곧게 선 자세 (Neutral) | `θ_t = 0°`, `θ_s = 0°` | **`0°`** | **`180°`** |
| | 무릎을 뒤로 직각 굽힘<br>*(90° Flexion, Standing Leg Curl)* | `θ_t = 0°`, `θ_s = -90°` | `+90°` (Flexion) | `90°` |
| | 발뒤꿈치를 엉덩이까지 당겨 깊게 접음<br>*(135° Max Flexion, Standing Leg Curl)* | `θ_t = 0°`, `θ_s = -135°`| `+135°` (Flexion) | `45°` |
| **Ankle** | 올곧게 선 자세 (Neutral) | `θ_s = 0°`, `θ_f = +90°` | **`0°`** | **`90°`** |
| | 발끝을 위로 20° 당김<br>*(20° Dorsiflexion, Toe Raise)* | `θ_s = 0°`, `θ_f = +110°` | `+20°` (Dorsiflexion) | `70°` |
| | 까치발을 45° 듦<br>*(45° Plantarflexion, Calf Raise)* | `θ_s = 0°`, `θ_f = +45°` | `-45°` (Plantarflexion) | `135°` |

### 4.1. 복합 자세 예시 (Multi-Joint Example)

위 표는 상위 분절을 `0°`로 고정한 단일 관절 움직임이라, 세 컨벤션이 갈라지는 지점이 드러나지 않는다. 좀 더 복합적인 자세를 예시로 디테일하게 설명한다.  
몸통은 올곧게 선 채로 한쪽 다리를 ㄱ 자로 들어 올리되, 허벅지는 수평 앞으로, 정강이는 무릎에서 수직으로 늘어뜨리고, 발끝은 `45°` 편(Plantarflexion) 자세 기준으로 설명한다.

```
   Anterior ←────────────────────────────→ Posterior

                      │  Torso (vertical)
                      │
          Knee ●──────● Hip
               │
               │
         Ankle ●
              ╱
         Toe ●          (들어 올린 다리 기준, 디딤 다리는 생략)
```

**Segment Angle** (전역 기준. 수직 아래가 `0°`, 앞으로 기울면 `+`)

| 분절 (방향 벡터) | 직립 자세 Segment Angle | 현 자세 Segment Angle | 변화량 (현 자세 − 직립 자세) |
| :--- | :--- | :--- | :--- |
| Pelvis (`θ_p`): 몸통 축의 하향 연장선 | `0°` | `0°` | `0 - 0 = 0°` |
| Thigh (`θ_t`): Hip→Knee | `0°` | `+90°` | `90 - 0 = +90°` |
| Shank (`θ_s`): Knee→Ankle | `0°` | `0°` | `0 - 0 = 0°` |
| Foot (`θ_f`): Ankle→Toe | `+90°` | `+45°` | `45 - 90 = -45°` |

**세 컨벤션 비교**

| Joint | Segment Angle (부모, 자식) | Clinical Joint Angle | Included Angle |
| :--- | :--- | :--- | :--- |
| **Hip** | `θ_p = 0°`, `θ_t = +90°` | `θ_t - θ_p = +90°` (Flexion) | `180° - θ_hip = 90°` |
| **Knee** | `θ_t = +90°`, `θ_s = 0°` | `θ_t - θ_s = +90°` (Flexion) | `180° - θ_knee = 90°` |
| **Ankle** | `θ_s = 0°`, `θ_f = +45°` | `θ_f - θ_s - 90° = -45°` (Plantarflexion) | `90° - θ_ankle = 135°` |

> 위 두 표의 식은 [3절](#3-분절-각도-기반-변환-공식-conversion-formulas)의 변환 공식을 그대로 적용한 것이다.

**위 예시에서 주목할 점**

**1. Segment Angle 하나로는 관절 각도를 알 수 없다.**
- `θ_s = 0°`는 직립에서도 이 자세에서도 같은 값이지만 무릎은 각각 `0°`와 `+90°`다.
- 정강이가 가만히 있어도 부모인 허벅지가 `+90°`로 돌면 무릎은 굽는다.
- Clinical은 절대 자세가 아니라 부모 분절과의 차이이므로, 반드시 부모와 짝지어 읽어야 한다.

**2. 부모가 멈춰 있을 때에 한해 Clinical이 자기 분절의 변화량과 일치한다.**
- 발목은 부모인 정강이가 `0°`에 머물러 있어 `θ_ankle`이 `θ_f`의 중립(`+90°`) 대비 변화량 `-45°`와 같아졌다.
- 무릎처럼 부모가 함께 도는 관절에서는 이 일치가 깨진다.

**3. Hip과 Knee가 같은 `+90°`지만 뺄셈의 상대가 다르다.**
- Hip은 `θ_t - θ_p`(몸통 기준), Knee는 `θ_t - θ_s`(허벅지 기준)다. 이 자세에서 `θ_p`와 `θ_s`가 나란히 `0°`라 값이 겹쳤을 뿐이다.

**4. Included Angle은 그림과 직접 대응한다.**
- Hip `90°`, Knee `90°`는 두 뼈가 직각으로 만난다는 뜻이고, Ankle `135°`는 중립의 직각에서 발끝을 `45°` 더 편 상태다.

**5. Segment는 체인을 따라 누적된 값, Clinical은 그 누적을 뺀 값이다.**
- 순운동학으로 자세를 만들 때 쓰는 것은 Clinical(로컬) 쪽이고, IMU처럼 전역 자세를 직접 재는 센서가 내놓는 것은 Segment 쪽이다. 둘을 섞으면 부모의 회전이 두 번 실린다.
