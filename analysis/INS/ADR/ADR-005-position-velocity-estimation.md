# ADR-005: 위치/속도 추정 — 9-state Error-State Kalman Filter

## Status
**Accepted** — 2026-05-06

## Context

어뢰 INS의 핵심 알고리즘으로 위치/속도/자세를 통합 추정해야 한다.
ADR-001(bias 별도 EMA), ADR-002(Quaternion), ADR-007(833Hz IMU + 100Hz 메인)에 이어
**칼만 필터 기반 통합 추정 알고리즘**을 결정한다.

운영 환경:
- 5m × 5m 평면, RC카 시연
- 운영 시간 ~5~30초 (발사 → 도달)
- ZUPT 거의 불가 (발사 전/후 정지만)
- 발사대 LiDAR가 어뢰 위치 측정 가능 (100Hz, ±5cm) — ADR-006 참조
- IMU drift: 30초 내 위치 오차 ~25cm

## Decision

**9-state Error-State Kalman Filter (ESKF) 채택**.

### State 구성

```
Nominal state (10차원):
  x_nom = [p, v, q]
    p ∈ ℝ³ : 위치 (nav frame, m)
    v ∈ ℝ³ : 속도 (nav frame, m/s)
    q ∈ ℝ⁴ : 자세 quaternion (Hamilton, body→nav)

Error state (9차원):
  δx = [δp, δv, δθ]ᵀ ∈ ℝ⁹
    δp : 위치 오차
    δv : 속도 오차
    δθ : 자세 오차 (작은 회전 벡터)

Bias: 칼만 외부에서 EMA로 별도 추정 (ADR-001)
```

## Rationale

### 왜 ESKF (vs 단순 Strapdown / EKF)?

| 방식 | 장점 | 단점 | 채택? |
|---|---|---|---|
| 단순 Strapdown 적분 | 단순 | 외부 측정 융합 X, drift 누적 | ✗ |
| 자세 EKF + 별도 위치 적분 | 부분적 칼만 | 통합 추정 X, P 행렬 분리 어려움 | ✗ |
| EKF (full state) | 통합 | quaternion 자유도 문제, 큰 오차 시 발산 | ✗ |
| **9-state ESKF** | **통합 + 안정** | 약간 단순화 | **✅** |
| 15-state full ESKF | 정석 | bias 칼만 통합 → ADR-001과 충돌, 1주 부담 | ✗ |

### 왜 9-state (vs 15-state)?

#### 1. ADR-001과의 일관성

ADR-001에서 가속도/자이로 bias를 **별도 EMA**로 추정 결정 (τ=15초/200초).
15-state ESKF는 bias를 칼만 안에서 다시 추정 → **두 시스템이 같은 것을 두 번 추정**하는 모순.
9-state는 bias state 제외 → ADR-001과 자연스럽게 정합.

#### 2. 환경 적합성 — 정량 분석

**운영 시간 vs bias 시간 상수 비율**:
```
운영 시간: 30초 (시연)
가속도 bias τ: 15초 → 비율 2배 (변동 가능)
자이로 bias τ: 200초 → 비율 0.15배 (거의 안 변함)
```

→ 자이로 bias 변동 거의 없음. 가속도는 살짝 변동 가능하지만:
- LiDAR 100Hz 측정으로 위치 보정 → 누적 영향 흡수
- ZUPT 부재 환경에서도 LiDAR가 그 역할 대신

#### 3. 정확도 비교

| 방식 | 위치 오차 (LiDAR 정상) | 위치 오차 (LiDAR 끊김) |
|---|---|---|
| 9-state + EMA bias | ~5 cm | ~+10 cm |
| 15-state full | ~3 cm | ~+8 cm |
| **차이** | **~2 cm** | **~2 cm** |

시연 정확도 요구 50cm. **둘 다 충분히 만족**.

#### 4. 수치 안정성

P 행렬 크기:
- 9-state: 9×9 (81 원소)
- 15-state: 15×15 (225 원소)

15-state는 양정치 깨질 위험 ↑. Joseph form, Cholesky 추가 필요.
9-state는 작은 행렬로 수치 안정.

#### 5. 1주 일정 적합성

| 단계 | 9-state | 15-state |
|---|---|---|
| 구현 | 3~4일 | 4~6일 |
| 디버깅 (P 발산) | 1~2일 | 3~5일 |
| **1주 가능성** | **90%** | **50%** |

ESKF 처음 짜는 환경에서 9-state가 안전.

### 점진적 확장 가능성

향후 운영 검증에서 9-state로 부족하면 15-state로 확장:
- bias state 3개 추가만으로 자연스럽게 확장
- 인터페이스는 동일 (ESKF 추상)
- ADR-XXX로 별도 결정

→ "단순함 + 확장성" 균형.

### 환경 분석 — ZUPT 부재의 의미

원래 INS에서는 ZUPT(Zero Velocity Update)가 누적 오차 reset의 핵심.
우리 환경:
- 발사 → 도달까지 어뢰 계속 운동
- ZUPT 0번 (발사 전 초기 캘리브만)

대안: **발사대 LiDAR 위치 측정**을 ZUPT의 역할로 활용 (ADR-006).
- 100Hz 매 사이클 측정
- ZUPT보다 더 자주 보정
- ESKF의 진짜 가치 발휘

## Mathematical Foundation

### Nominal State 진행 (연속 시간)

```
ṗ = v
v̇ = R(q)·(a_imu - b_a) - g
q̇ = 0.5·q ⊗ (ω_imu - b_g)
```

이산화 (10ms):
```
p[k+1] = p[k] + v[k]·dt + 0.5·a_nav[k]·dt²
v[k+1] = v[k] + a_nav[k]·dt
q[k+1] = q[k] ⊗ Δq(ω_corrected · dt)
```

### Error State 진행

연속 시간:
```
δṗ = δv
δv̇ = -R(q)·[a]× · δθ + R(q)·n_a
δθ̇ = -[ω]× · δθ + n_g
```

행렬 형태:
```
       δp     δv          δθ
     ┌───────────────────────────┐
δṗ  │   0      I             0  │
δv̇  │   0      0    -R(q)·[a]×  │     F (9×9)
δθ̇  │   0      0       -[ω]×    │
     └───────────────────────────┘
```

이산화 (1차 근사):
```
Φ = I + F · dt          (9×9)
δx[k+1] = Φ · δx[k]
P[k+1] = Φ · P · Φᵀ + Q
```

### Process Noise Q

ADR-001 Allan Variance 측정값 활용:

```
VRW = 5.89e-4 m/s²/√Hz  (가속도)
ARW = 5.82e-5 rad/√s   (자이로)

Q_v = VRW² · dt = 3.47e-9 m²/s²  (이산)
Q_θ = ARW² · dt = 3.39e-11 rad²  (이산)
```

```
       δp     δv         δθ
     ┌─────────────────────────────────┐
δp  │   0      0            0          │
δv  │   0    Q_v·I          0          │     Q (9×9)
δθ  │   0      0          Q_θ·I        │
     └─────────────────────────────────┘
```

위치 부분 0: 위치는 직접 측정 안 함, 속도에서 적분.

### Measurement Update (LiDAR, ADR-006)

LiDAR 측정 z = [px, py]ᵀ (5m×5m 평면 2D):

```
H = [I_xy    0    0]                           (2×9)
        2×3  2×3  2×3

R = diag(σ_lidar²)  with σ_lidar = 0.05 m      (2×2)

y = z - H · x_nominal[xy]
S = H·P·Hᵀ + R
K = P·Hᵀ·S⁻¹
δx = K · y
P_new = (I - K·H) · P
```

### Reset (Injection)

```
p_nominal ← p_nominal + δp
v_nominal ← v_nominal + δv
q_nominal ← q_nominal ⊗ q(δθ)         (quaternion 곱)
δx ← 0
P ← G · P · Gᵀ                         (G ≈ I)
```

## Alternatives Considered

### 대안 A: 단순 Strapdown 적분 (칼만 X)

- IMU만 적분, LiDAR 측정은 표시용으로만
- 5m × 5m / 30초 시연: drift ~25cm, 요구 50cm → 만족
- **비채택 이유**: 발사대 측정을 적극 활용 못함, 자소서 임팩트 약함

### 대안 B: 자세 EKF + 단순 위치 적분

- 자세만 칼만, 위치/속도는 별도 적분
- **비채택 이유**: P 행렬 분리로 통합 추정 어려움, 측정 융합 비효율

### 대안 C: 15-state full ESKF

- bias도 칼만으로 통합 추정
- 정석 INS 표준
- **비채택 이유**: 
  - ADR-001 EMA bias와 충돌
  - 1주에 처음 짜기 위험 (P 발산 디버깅)
  - 정확도 차이 미미 (~2cm)
  - 시연 요구 50cm 9-state로도 충분

### 대안 D: 9-state ESKF ✅ 채택

- ADR-001 일관 + 환경 적합 + 1주 안전
- 향후 15-state 확장 가능

## Consequences

### 긍정적 영향

1. **통합 추정**: 위치 + 속도 + 자세를 하나의 칼만으로
2. **외부 측정 활용**: LiDAR 측정으로 누적 오차 보정 (ADR-006)
3. **수치 안정**: 9×9 행렬, P 양정치 유지 쉬움
4. **ADR-001 정합**: bias EMA와 칼만이 분리되어 모순 없음
5. **확장 가능**: 향후 15-state로 자연스럽게 확장
6. **자소서 어필**: "환경 분석 후 9-state 채택" → 시니어 사고

### 부정적 영향 / Trade-off

1. **정확도 약간 손해**: 15-state 대비 ~2cm
   - 완화책: 시연 요구 50cm 충분히 만족
2. **bias 변동 못 잡음**: 운영 중 bias 변동은 EMA 시간 상수로만 따라감
   - 완화책: LiDAR 측정으로 위치 오차 흡수, 짧은 운영 시간 (30초)
3. **LiDAR 끊김 시 robust 약함**: 15-state 대비
   - 완화책: NHC 보조 측정 (ADR-006)
4. **튜닝 필요**: Q 행렬 초기값(Allan 측정)에서 시작, 실제 동작 보고 조정
5. **F 매 사이클 계산**: R(q), [a]×, [ω]× 매번 갱신
   - 완화책: 시간 예산 8.3% (ADR-007), CPU 충분

## Implementation Notes

### Predict Step 의사코드

```cpp
void eskf_predict(EskfState& x, ImuSample s, BiasEstimate b, float dt) {
    // 1. Bias 보정
    Vec3 a_corr = s.a - b.b_a;
    Vec3 w_corr = s.w - b.b_g;
    
    // 2. F 행렬 매 사이클 계산
    Matrix3 R = quaternion_to_rotation(x.q);
    Matrix3 a_skew = skew_symmetric(a_corr);
    Matrix3 w_skew = skew_symmetric(w_corr);
    
    Matrix9 F = Matrix9::Zero();
    F.block<3,3>(0, 3) = Matrix3::Identity();
    F.block<3,3>(3, 6) = -R * a_skew;
    F.block<3,3>(6, 6) = -w_skew;
    
    // 3. Phi = I + F*dt
    Matrix9 Phi = Matrix9::Identity() + F * dt;
    
    // 4. P 진행
    x.P = Phi * x.P * Phi.transpose() + x.Q;
    
    // 5. Nominal state 비선형 적분
    Vec3 a_nav = R * a_corr - Vec3{0, 0, 9.81f};
    x.p += x.v * dt + 0.5f * a_nav * dt * dt;
    x.v += a_nav * dt;
    
    Vec3 dtheta = w_corr * dt;
    Quaternion dq = small_angle_quaternion(dtheta);
    x.q = quaternion_multiply(x.q, dq);
    x.q.normalize();
}
```

### Update Step 의사코드

```cpp
void eskf_update_lidar(EskfState& x, Vec2 z_lidar) {
    // H is constant, R is constant — precomputed
    static const Matrix2x9 H = make_lidar_H();
    static const Matrix2 R = make_lidar_R();
    
    // 1. 측정 잔차
    Vec2 y = z_lidar - x.p.head<2>();
    
    // 2. Kalman update
    Matrix2 S = H * x.P * H.transpose() + R;
    Matrix9x2 K = x.P * H.transpose() * S.inverse();
    
    Vec9 delta_x = K * y;
    x.P = (Matrix9::Identity() - K * H) * x.P;
    
    // 3. Reset / Inject
    x.p += delta_x.segment<3>(0);
    x.v += delta_x.segment<3>(3);
    
    Vec3 dtheta = delta_x.segment<3>(6);
    Quaternion dq_correction = small_angle_quaternion(dtheta);
    x.q = quaternion_multiply(x.q, dq_correction);
    x.q.normalize();
    
    // delta_x is implicitly 0 from here (not stored)
}
```

### 클래스 구조

```cpp
namespace torpedo {

struct EskfState {
    // Nominal state
    Vec3 p;
    Vec3 v;
    Quaternion q;
    
    // Error covariance (9x9)
    Matrix9 P;
    
    // Process noise (constant after init)
    Matrix9 Q;
};

class EskfEstimator {
public:
    void init(const InitParams& params);
    void predict(ImuSample s, BiasEstimate b, float dt);
    void update_lidar(Vec2 z_lidar);
    void update_nhc();  // Non-Holonomic Constraint, ADR-006
    
    EskfState state() const;
private:
    EskfState x_;
};

} // namespace torpedo
```

### 5계층에서의 위치

```
Layer: Domain (Layer 3)
  include/torpedo/domain/estimator/eskf.hpp
  src/domain/estimator/eskf.cpp
```

## Verification Plan

Day 4-5 구현 후:
1. **단위 테스트**: FakeImu (정지 상태) → ESKF predict → P 단조 증가 확인
2. **시뮬 테스트**: FakeImu (등속 직선) → ESKF predict → 위치 적분 정확성
3. **시뮬 테스트**: FakeLidar 측정 주입 → ESKF update → P 감소 확인
4. **MATLAB 시뮬레이터**: 시연 시나리오 시뮬, 위치 오차 추적
5. **ZYBO 통합**: 실제 IMU + 발사대 LiDAR 측정으로 동작 확인

수렴 기준:
- LiDAR 정상: 위치 오차 < 5cm (정상 동작)
- LiDAR 끊김 1초: 위치 오차 < 15cm
- 시연 요구: < 50cm (충분히 만족)

## References

### 이론
- Sola, "Quaternion kinematics for the error-state Kalman filter", arXiv 2017
- Madyastha et al., "EKF vs ESKF for Aircraft Attitude Estimation", 2011
- Trawny & Roumeliotis, "Indirect Kalman Filter for 3D Attitude Estimation", 2005

### 선행 ADR
- ADR-001: Bias 추정 시간 상수 분리 (EMA, 칼만 외부)
- ADR-002: Hamilton Quaternion 자세 표현
- ADR-007: 833Hz IMU + 100Hz 메인 + 6축 Median + 8샘플 평균

### 관련 ADR
- ADR-006: 외부 측정 통합 (LiDAR + NHC) — 동시 결정
- ADR-008 (예정): 만약 9-state로 부족 시 15-state 확장 검토

### 측정 데이터
- Allan Variance 1시간 측정: `~/zybo_dev/allan/imu_1h.csv`
- VRW: 5.33e-4 ~ 7.37e-4 m/s²/√Hz (데이터시트 0.91~1.25×)
- ARW: 0.18 ~ 0.22 deg/√h (데이터시트 0.60~0.73×)
