# ADR-001: 가속도/자이로 Bias 추정 시간 상수 분리

## Status
**Accepted** — 2026-04-30

## Context

수중 어뢰 INS 알고리즘 설계의 첫 단계로, IMU 센서의 시간 가변 bias를
실시간 추정·보정하는 `BiasEstimator` 모듈을 설계해야 한다.

설계에 앞서 사용 IMU(ISM330DHCX)의 노이즈 특성을 정량 분석하여,
bias 추정에 사용할 적절한 시간 상수(τ)의 근거를 확보한다.

## Decision

`BiasEstimator`를 가속도용·자이로용 **2개 모듈로 분리**하고,
각 모듈은 다음 시간 상수의 EMA로 bias를 추정한다:

| 모듈 | 시간 상수 (τ) | 비고 |
|---|---|---|
| `AccelBiasEstimator` | **15 초** | ax, ay 10초 / az 21초의 절충값 |
| `GyroBiasEstimator` | **200 초** | gy 84초 / gx 168초 / gz 670초 의 보수적 평균 |

EMA 갱신은 **정지 상태(ZUPT 검출 ON)에서만** 수행.

## Rationale

### 측정 환경
- 보드: ZYBO Z7 + ISM330DHCX
- SPI 833Hz 설정, 실측 평균 fs = **782.5 Hz**
- 측정 시간: 60분 정지 상태
- 총 샘플: 2,769,945개
- 분석: MATLAB Sensor Fusion Toolbox `allanvar` (octave taus)

### 측정 결과 (Allan Deviation 최저점 = Bias Instability 시간 상수)
- 가속도 ax  τ_optimal = 10.5 초,  BI = 3.61e-4 m/s²
- 가속도 ay  τ_optimal = 10.5 초,  BI = 3.68e-4 m/s²
- 가속도 az  τ_optimal = 20.9 초,  BI = 5.61e-4 m/s²
- 자이로 gx  τ_optimal = 167  초,  BI = 7.14e-6 rad/s
- 자이로 gy  τ_optimal =  84  초,  BI = 1.02e-5 rad/s
- 자이로 gz  τ_optimal = 670  초,  BI = 8.71e-6 rad/s

### 데이터시트 정합성

| 항목 | 데이터시트 | 실측 | 비율 | 평가 |
|---|---|---|---|---|
| Accel VRW | 5.89e-4 m/s²/√Hz | 5.33e-4 ~ 7.37e-4 | 0.91~1.25× | ✓ 정상 |
| Gyro ARW | 0.30 deg/√h | 0.18 ~ 0.22 | 0.60~0.73× | ✓ 양호 |

→ 칩 정상 동작 + 측정 신뢰성 확인.

### 핵심 발견

**가속도와 자이로의 최적 시간 상수가 한 자릿수 차이.**
- 가속도: 10~20초
- 자이로: 80~670초

이는 두 센서의 bias drift 메커니즘 차이에 기인:
- 가속도계: 온도/응력 영향이 빠름 (수십 초 단위에서 안정점)
- 자이로: 양자화/적분 노이즈 특성으로 장시간 평균에서 안정

→ 두 센서를 동일 윈도우로 처리하면 둘 다 sub-optimal.
→ 분리 처리가 정량적으로 정당화됨.

### EMA 시간 상수 → 계수 변환

연속시간 EMA의 시간 상수 τ는 이산 EMA 계수 α와 다음 관계:
α = 1 - exp(-Δt / τ)

샘플 주기 Δt = 1/782.5 ≈ 1.278 ms 기준:
- AccelBiasEstimator (τ = 15s):  α ≈ 8.5 × 10⁻⁵
- GyroBiasEstimator  (τ = 200s): α ≈ 6.4 × 10⁻⁶

## Alternatives Considered

### 대안 A: 단일 윈도우로 두 센서 통합
- 단일 τ 선택 시 한쪽이 반드시 sub-optimal
- 5초 → 가속도엔 짧고 자이로엔 매우 짧음
- 1분 → 가속도엔 random walk 영역 진입
- 5분 → 가속도 정확도 손상

→ Allan plot의 시간 상수 분리가 명확하므로 통합은 정량적으로 부적절.

### 대안 B: 캘리브레이션 1회만 (bias 학습 X)
- 시작 시 평균값을 bias로 고정
- 시간에 따른 drift 보정 못 함
- ISM330DHCX의 BI(가속도 0.04 mg, 자이로 0.5 mdps)는 작지만,
  운용 시간 30분~1시간 동안 무시 못 할 수준

→ 동적 학습 필요.

### 대안 C: 칼만 필터 state로 통합 추정
- ESKF의 state vector에 bias 포함하여 통합 추정
- 더 정밀하지만 복잡도 큼
- 1단계에서는 모듈 분리가 단순하고 검증 용이

→ 향후 ESKF 도입 시 채택 검토 (ADR-XXX로 별도 문서화).

## Consequences

### 긍정적 영향

1. **정량적 설계 근거 확보** — 모든 시간 상수가 측정 데이터에서 유도됨
2. **각 센서 최적 처리** — Bias Instability 최저점 시간 상수 활용
3. **ESKF 확장 시 Q/R 행렬 즉시 활용** — Allan BI 값을 그대로 noise 모델에 반영
4. **재현성** — 측정 절차(`raw_logger.cpp` + `allan_analysis.m`) 표준화로 향후 다른 IMU 칩에도 적용 가능

### 부정적 영향 / Trade-off

1. **모듈 2개 관리** — `AccelBiasEstimator`, `GyroBiasEstimator` 분리 필요
   - 완화책: 공통 인터페이스 `IBiasEstimator`로 통일
2. **τ 값의 환경 의존성** — 측정 환경(온도, 진동) 변하면 τ_optimal 변화
   - 완화책: 운용 환경에서 재측정 권장 (특히 RC카 통합 시)
3. **EMA의 응답 지연** — τ=200초 자이로는 bias 변화에 느리게 반응
   - 완화책: ZUPT 게이팅 + 초기 캘리브 단계에서 빠르게 수렴

## Implementation Notes

```cpp
// include/torpedo/domain/bias_estimator.hpp
namespace torpedo {

class IBiasEstimator {
public:
    virtual ~IBiasEstimator() = default;
    virtual void update(float x, float y, float z, bool is_stationary) = 0;
    virtual void get(float& bx, float& by, float& bz) const = 0;
};

class AccelBiasEstimator : public IBiasEstimator {
    static constexpr float TAU_SEC = 15.0f;
    // alpha = 1 - exp(-dt / tau)는 init() 또는 생성자에서 fs 받아 계산
    float alpha_;
    float bx_=0, by_=0, bz_=0;
public:
    AccelBiasEstimator(float fs);
    void update(float ax, float ay, float az, bool stationary) override;
    void get(float& bx, float& by, float& bz) const override;
};

class GyroBiasEstimator : public IBiasEstimator {
    static constexpr float TAU_SEC = 200.0f;
    // 동일한 인터페이스, 다른 τ
    // ...
};

} // namespace torpedo
```

## References

- 측정 데이터: `~/zybo_dev/allan/imu_1h.csv` (2026-04-30)
- 분석 스크립트: `~/zybo_dev/allan/allan_analysis.m`
- 결과 그래프: `allan_variance_6axis.png`
- 데이터시트: ISM330DHCX Datasheet, STMicroelectronics, Rev. 5
- IEEE Std 952-1997: Allan Variance 표준 정의
- N. El-Sheimy et al., "Analysis and Modeling of Inertial Sensors Using Allan Variance," IEEE TIM, 2008