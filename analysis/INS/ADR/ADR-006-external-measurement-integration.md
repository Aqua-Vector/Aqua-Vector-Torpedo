# ADR-006: 외부 측정 통합 — LiDAR + NHC + 초기 캘리브

## Status
**Accepted** — 2026-05-06

## Context

ADR-005에서 9-state ESKF 채택. 칼만 필터가 의미 있으려면 **외부 측정**이 필수.
ZUPT가 거의 불가능한 환경(어뢰가 운영 중 정지하지 않음)에서 어떤 측정 source를
어떻게 통합할지 결정한다.

운영 환경:
- 5m × 5m 평면, RC카 시연
- 운영 시간 ~5~30초
- IMU 단독 시 30초 후 위치 오차 ~25cm
- 발사 전 정지 가능 (초기 캘리브용)
- 발사 후 도달까지 정지 없음 (ZUPT 부재)
- 발사대 LiDAR 가용

## Decision

**4-layer 측정 통합 채택**:

| 우선순위 | Source | 주기 | 정확도 | 역할 |
|---|---|---|---|---|
| 1 | **LiDAR 위치 측정** | 100Hz | ±5cm | 주 측정 (continuous correction) |
| 2 | **초기 캘리브** | 1회 (발사 전) | - | bias seeding + 자세 초기화 |
| 3 | **NHC** | 매 사이클 (보조) | σ~0.1 m/s | LiDAR 끊김 시 fallback |
| 4 | 도착 후 정지 | 1회 (시연 종료) | - | 사후 검증 (실시간 X) |

## Rationale

### Layer 1: LiDAR 위치 측정 (주)

#### 측정 모델

발사대 LiDAR가 어뢰 위치를 추적하여 어뢰에 RS-485로 전송:

```
z_lidar = [px_lidar, py_lidar]ᵀ ∈ ℝ²

H_lidar = [I_xy   0    0]                      (2×9)
              2×3  2×3 2×3

R_lidar = diag(σ²)  with σ = 0.05 m            (2×2)

= ⎡ 0.0025    0   ⎤  m²
  ⎣   0    0.0025 ⎦
```

#### 활용 근거

- **100Hz 매 사이클 측정**: ZUPT 부재 환경에서 누적 오차 reset 역할 대체
- **±5cm 정확도**: IMU 30초 drift(~25cm) 대비 압도적 개선
- **자세 간접 보정**: P 비대각 통해 위치 측정으로 자세 오차도 보정 (ADR-005 참조)

#### 운영 환경 적합성

- 발사 전 LiDAR가 장애물 매핑 (정적 환경 1회 스캔)
- 발사 후 LiDAR가 어뢰 추적으로 임무 전환
- 멘토님 시스템 설계 기반 (장애물은 정적이므로 발사 후 LiDAR 자원 가용)

### Layer 2: 초기 캘리브레이션 (발사 전)

#### 절차

발사 전 어뢰가 RC카 위에 정지 상태로 놓인 상태에서 5초 측정:

```
[1] Bias 초기값 추정
- 정지 상태의 IMU 평균을 bias로 사용
- ADR-001 EMA의 초기값으로 seeding
- 5초 = 4165 샘플 (833Hz) 평균

[2] 자세 초기화
- 가속도로 pitch/roll 추정:
    pitch = atan2(-ax, sqrt(ay² + az²))
    roll  = atan2(ay, az)
  (정지 시 가속도 = 중력 방향)
- yaw 초기값:
    옵션 A: 발사대가 알려줌 (LiDAR로 어뢰 방향 측정)
    옵션 B: 0으로 고정 (시연 시 어뢰 방향 사전 정렬)
- Quaternion 변환하여 q_nominal 초기화

[3] 위치/속도 초기화
- p = 발사 위치 (발사대가 설정)
- v = 0 (정지)

[4] P 행렬 초기화
- 위치: σ_p = 5cm (LiDAR 정확도)
- 속도: σ_v = 1 cm/s (정지 검증)
- 자세: σ_θ = pitch/roll 1°, yaw 5° (불확실성 큼)
```

#### 활용 근거

- ADR-001의 EMA bias는 시간 상수 200초 → 시연 시간 30초 이내 수렴 안 함
- 초기 5초 평균으로 bias 빠르게 시작점 확보
- 자세 초기화 없이 시작하면 ESKF 첫 사이클부터 큰 오차

### Layer 3: NHC (Non-Holonomic Constraint)

#### 직관

RC카는 **옆방향(left-right)으로 미끄러지지 않음** — 차량 운동학의 제약.

```
v_body[lateral] ≈ 0
```

이를 측정으로 활용:
- 측정값 z_nhc = 0
- 실제 v_body[lateral]가 0에서 벗어나면 칼만이 이를 보정

#### 측정 모델

```
v_body = R(q)ᵀ · v_nav

z_nhc = v_body[y]   (body의 좌우 방향)
      = (R(q)ᵀ · v_nav)[1]
      ≈ 0
```

H 행렬은 자세에 따라 변함 (Jacobian 계산):
```
H_nhc[3:6] = R(q)ᵀ[1, :]    ← 속도에 대한 부분
H_nhc[6:9] = -[v_body]× [1]  ← 자세 오차의 영향
H_nhc[0:3] = 0               ← 위치는 영향 X
```

R_nhc:
```
σ_nhc = 0.1 m/s   (RC카 미끄러짐 가능성 마진)
R_nhc = σ_nhc² = 0.01 m²/s²
```

#### 활용 근거

- LiDAR 측정 사이 추가 보정 (lateral velocity drift 방지)
- LiDAR 끊김 시 fallback (혼자만으론 약하지만 발산 늦춤)
- 측정 비용 0 (계산만으로 활용)

#### 활성화 조건

- 어뢰가 정지 가까울 때는 비활성 (속도 너무 작으면 노이즈 영향)
- |v| > 0.1 m/s 일 때만 활성

### Layer 4: 도착 후 정지

#### 활용

시연 종료 후 어뢰 정지 시:
- INS 추정 v ≈ 0 인지 확인 (수렴 검증)
- 최종 위치 vs LiDAR 측정 비교 (정확도 평가)
- 자소서/발표 데이터 (시연 결과 정량 분석)

#### 비활용 (실시간 보정 X)

- 시연이 끝났으므로 보정 의미 없음
- 사후 분석용으로만 사용

## LiDAR 끊김 시 처리

LiDAR 다중 객체 추적에서 일시 끊김 가능 (ADR-005 환경 분석 참조).

### Timeout 정책

```
정상: LiDAR 측정 100Hz 주기 (10ms)
허용: 50ms (5사이클) 끊김까지 정상 동작
경고: 50~200ms 끊김 — NHC만으로 진행
위험: 200ms 이상 끊김 — 발사대 통신 fault 처리
```

### Fallback 전략

```cpp
if (lidar_packet_age < 50ms) {
    // 정상: LiDAR update + NHC
    eskf_update_lidar(z_lidar);
    eskf_update_nhc();
} else if (lidar_packet_age < 200ms) {
    // 경고: NHC만
    eskf_update_nhc();
    // P가 점차 커짐 → 다음 LiDAR 측정 시 강하게 보정
} else {
    // 위험: 발사대 통신 fault
    // 단순 IMU 적분만 (predict only)
    // 상태 플래그를 발사대로 전송
}
```

### 검증

```
LiDAR 정상 30초:        위치 오차 ~5cm
LiDAR 1초 끊김 (NHC):   추가 +10cm (총 ~15cm)
LiDAR 5초 끊김:         추가 ~25cm (drift 그대로)
시연 요구 50cm:          1초 끊김까지 만족
```

## Communication Packet Design (제민이 합의 필요)

### 발사대 → 어뢰 (Downlink, 100Hz, RS-485)

```cpp
struct __attribute__((packed)) DownlinkPacket {
    // 헤더
    uint8_t  sync;                  // 0xAA
    uint32_t timestamp_us;          // 발사대 기준 시간
    uint16_t seq;                   // 시퀀스 번호
    
    // 표적 정보 (목표)
    float    target_x, target_y;    // 표적 위치 (m)
    
    // 어뢰 위치 측정 (NEW — ESKF update용)
    float    torpedo_x, torpedo_y;  // LiDAR 측정값
    
    // 상태 플래그
    uint8_t  cmd_code;              // 명령 (정지/추적/회피 등)
    uint8_t  lidar_status;          // 0=정상, 1=의심, 2=끊김
    
    // CRC
    uint16_t crc16;
};
// 총: 1 + 4 + 2 + 8 + 8 + 1 + 1 + 2 = 27 byte
```

### 어뢰 → 발사대 (Uplink, 100Hz, RS-485)

```cpp
struct __attribute__((packed)) UplinkPacket {
    // 헤더
    uint8_t  sync;                  // 0xBB
    uint32_t timestamp_us;          // 어뢰 기준 시간
    uint16_t seq;
    
    // ESKF 추정 결과
    float    p_estimate[3];         // 추정 위치 (m)
    float    v_estimate[3];         // 추정 속도 (m/s)
    float    yaw_estimate;          // 추정 yaw (rad)
    
    // 상태
    uint8_t  status_flags;          // bit0=ESKF_OK, bit1=LiDAR_OK, ...
    uint8_t  reserved;
    
    // CRC
    uint16_t crc16;
};
// 총: 1 + 4 + 2 + 28 + 1 + 1 + 2 = 39 byte
```

### 통신 부담 분석

```
Downlink: 27 byte × 100Hz = 2700 byte/s = 21,600 bps
Uplink:   39 byte × 100Hz = 3900 byte/s = 31,200 bps
합계:     52,800 bps ≈ 53 kbps

RS-485 460,800 bps의 11.5% 사용
여유 88.5% (ADR-007 시간 예산과 일관)
```

## Alternatives Considered

### 대안 A: LiDAR 측정만 사용

- 가장 단순
- **비채택 이유**: LiDAR 끊김 시 fallback 없음, 위험

### 대안 B: NHC만 사용 (LiDAR 표시용)

- 칼만 부담 적음
- **비채택 이유**: NHC만으론 위치 drift 못 잡음, ESKF 의미 약화

### 대안 C: 4-layer 통합 ✅ 채택

- 다중 보정으로 robust
- LiDAR 끊김 시에도 fallback
- 자소서 어필 강력

### 대안 D: GPS 추가 통합

- 5m×5m 실내 시연이라 GPS 무관
- 향후 야외 어뢰로 확장 시 검토 (ADR-XXX)

## Consequences

### 긍정적 영향

1. **다중 보정 layer**: 단일 측정 실패에 robust
2. **LiDAR 끊김 대응**: NHC fallback으로 부드러운 degradation
3. **초기 수렴 빠름**: 캘리브로 ESKF 첫 사이클부터 안정
4. **자소서 어필**: "측정 다중화 + Graceful degradation"
5. **시연 안정성**: 5cm → 50cm 마진 10배

### 부정적 영향 / Trade-off

1. **NHC 가정**: RC카가 미끄러지면 NHC 부정확
   - 완화책: σ_nhc=0.1m/s 마진, 활성 조건 |v|>0.1m/s
2. **통신 패킷 커짐**: Downlink 27 byte (vs 기존 ~15 byte)
   - 완화책: RS-485 460,800 bps 여유 88.5%
3. **제민이와 합의 필요**: 패킷 구조 변경
   - 완화책: ADR로 명시 + PR로 합의
4. **초기 캘리브 5초 필요**: 발사 직전 5초 정지 절차
   - 완화책: 시연 시나리오에 자연스럽게 포함

## Implementation Notes

### Update Step Dispatcher

```cpp
class EskfUpdateScheduler {
public:
    void on_imu_sample(ImuSample s, EskfState& x) {
        // 매 사이클 NHC 시도 (조건부)
        if (x.v.norm() > 0.1f) {
            eskf_update_nhc(x);
        }
    }
    
    void on_lidar_packet(DownlinkPacket pkt, EskfState& x) {
        if (pkt.lidar_status == 0 /* 정상 */) {
            Vec2 z = {pkt.torpedo_x, pkt.torpedo_y};
            eskf_update_lidar(x, z);
            last_lidar_us_ = now_us();
        }
        // status != 0 이면 update 건너뜀
    }
    
    bool is_lidar_fresh() const {
        return (now_us() - last_lidar_us_) < 50000;  // 50ms
    }
    
private:
    uint64_t last_lidar_us_ = 0;
};
```

### 초기 캘리브 절차

```cpp
void initial_calibration(IImu& imu, EskfState& x, float duration_sec = 5.0f) {
    Vec3 a_sum{0, 0, 0}, w_sum{0, 0, 0};
    int count = 0;
    
    uint64_t t_start = now_us();
    while ((now_us() - t_start) < duration_sec * 1e6) {
        ImuSample s;
        if (imu.read(s)) {
            a_sum += s.a;
            w_sum += s.w;
            count++;
        }
    }
    
    Vec3 a_mean = a_sum / count;
    Vec3 w_mean = w_sum / count;
    
    // Bias 초기값 (자이로는 평균값, 가속도는 중력 빼고)
    x.b_g = w_mean;
    x.b_a = a_mean - Vec3{0, 0, 9.81f};  // (이미 nav→body 변환 가정)
    
    // 자세 초기화 (가속도로 pitch/roll)
    float pitch = atan2(-a_mean.x, sqrt(a_mean.y*a_mean.y + a_mean.z*a_mean.z));
    float roll  = atan2(a_mean.y, a_mean.z);
    float yaw   = 0.0f;  // 또는 발사대로부터 받음
    
    x.q = euler_to_quaternion(roll, pitch, yaw);
    
    // 위치/속도
    x.p = Vec3{0, 0, 0};  // 발사 위치
    x.v = Vec3{0, 0, 0};
    
    // P 초기값
    x.P = Matrix9::Zero();
    x.P.block<3,3>(0, 0) = Matrix3::Identity() * 0.05f * 0.05f;     // pos σ=5cm
    x.P.block<3,3>(3, 3) = Matrix3::Identity() * 0.01f * 0.01f;     // vel σ=1cm/s
    x.P.block<3,3>(6, 6) = Matrix3::Identity() * 0.0017f * 0.0017f; // att σ=0.1°
}
```

### NHC Update 의사코드

```cpp
void eskf_update_nhc(EskfState& x) {
    // body frame 속도 = R^T * v_nav
    Matrix3 R = quaternion_to_rotation(x.q);
    Vec3 v_body = R.transpose() * x.v;
    
    // 측정 잔차: v_body[y] should be 0
    float y_innov = 0.0f - v_body.y();
    
    // H 행렬 (자세에 따라 변함)
    Matrix1x9 H = Matrix1x9::Zero();
    H.block<1,3>(0, 3) = R.transpose().row(1);          // ∂v_body[y]/∂v_nav
    H.block<1,3>(0, 6) = -skew_symmetric(v_body).row(1); // ∂v_body[y]/∂δθ
    
    // R 행렬 (스칼라)
    float R_nhc = 0.1f * 0.1f;  // 0.01 m²/s²
    
    // Kalman update
    float S = (H * x.P * H.transpose())(0, 0) + R_nhc;
    Matrix9x1 K = x.P * H.transpose() / S;
    
    Vec9 delta_x = K * y_innov;
    x.P = (Matrix9::Identity() - K * H) * x.P;
    
    // Reset / inject (LiDAR update와 동일)
    inject_error_to_nominal(x, delta_x);
}
```

## Verification Plan

Day 4-5 구현 후:

1. **단위 테스트 — 초기 캘리브**: FakeImu 정지 → 5초 캘리브 → bias/자세 수렴 확인
2. **단위 테스트 — LiDAR update**: FakeImu + 가짜 LiDAR 측정 → P 감소, 위치 보정 확인
3. **단위 테스트 — NHC**: 운동 중 lateral velocity drift 발생 → NHC가 보정
4. **시뮬 — LiDAR 끊김**: 5초 끊김 시 fallback 동작 → 발산 안 함
5. **MATLAB 시뮬**: 시연 시나리오 (30초 운동) → 위치 오차 < 50cm
6. **ZYBO 통합**: 실제 환경 검증

## References

### 이론
- Sola, "Quaternion kinematics for the error-state Kalman filter", arXiv 2017
- Dissanayake et al., "The aiding of a low-cost strapdown inertial measurement unit using vehicle model constraints for land vehicle applications", IEEE TRA, 2001 (NHC)
- Kong et al., "INS/GNSS Integration with NHC", 2019

### 선행 ADR
- ADR-001: Bias EMA 시간 상수 (초기 캘리브와 연동)
- ADR-002: Hamilton Quaternion 자세 표현
- ADR-005: 9-state ESKF (이 ADR의 update step source)
- ADR-007: 833Hz IMU + 100Hz 메인 (시간 예산)

### 시스템 설계
- 멘토님의 LiDAR 임무 분리 설계 (발사 전 장애물 매핑 / 발사 후 어뢰 추적)
- 5m × 5m RC카 시연 환경
- ADR-005 환경 분석 (ZUPT 부재, LiDAR 풍부)
