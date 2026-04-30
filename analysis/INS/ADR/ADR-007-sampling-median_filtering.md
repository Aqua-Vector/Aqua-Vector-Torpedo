# ADR-007: 샘플링 + 필터링 전략

## Status
**Accepted** — 2026-04-30

## Context

INS 알고리즘 입력단의 샘플링 주파수, 메인 루프 주기, 다운샘플링 방식,
필터링 정책을 결정해야 한다. 시간 예산 + 노이즈 + spike 특성을 정량 분석.

## Decision

| 항목 | 값 | 근거 |
|---|---|---|
| **IMU ODR** | 833 Hz | LPF 91Hz, 100Hz 정수 8배수 |
| **메인 루프** | 100 Hz (10ms) | RS-485 송신 동기화 |
| **다운샘플링** | 8샘플 단순 평균 | √8 ≈ 2.83× 노이즈 감소 |
| **Median Filter** | **6축 모두 3-sample** | 운동 환경 spike 검증 |
| **추가 SW LPF** | 없음 | 칩 LPF + 평균 + Median으로 충분 |

## Rationale

### IMU ODR 선택 (833Hz)

ISM330DHCX의 이산 ODR 옵션 중 선택:
- 12.5 / 26 / 52 / 104 / 208 / 416 / **833** / 1666 / 3333 / 6666 Hz

**선택 근거**:
1. **LPF 적절성**: 내장 LPF cutoff = ODR/9 ≈ 91Hz
2. **메인 루프 정수배**: 833 / 100 = 8.33 ≈ 8 샘플/주기
3. **Allan 측정 검증**: VRW 0.91~1.25× 데이터시트 (정상)

### 메인 루프 100Hz

RS-485 송신 동기화. 시간 예산 분석:

| 작업 | 이론 | 비율 |
|---|---|---|
| SPI 8샘플 읽기 | 600 μs | 6% |
| UART 24B 송신 | 210 μs | 2% |
| INS + Median + ESKF | 30 μs | 0.3% |
| **합계 (이론)** | **840 μs** | **8.4%** |

ARM Cortex-A9 @ 667MHz에서 알고리즘은 전체의 5% 미만.
실측 마진 3~5배 고려해도 3~4ms로 10ms 예산 내.

### 다운샘플링 = 8샘플 단순 평균

- 노이즈 √8 ≈ 2.83× 감소
- VRW 0.7e-3 → 0.25e-3 m/s² (Allan 기준)
- 칩 내장 LPF (91Hz)와 결합하여 운동(< 20Hz) 통과

### Median Filter — 6축 모두 적용 (3-sample)

**측정 결과 기반 정량 근거**:

#### 측정 1: 정지 1시간 (Allan + spike 분석)

| 축 | spike (>50σ) | max σ |
|---|---|---|
| ax | 0.003% (74건) | 136σ |
| ay | 0.002% (44건) | 116σ |
| az | 0.003% (76건) | 164σ |
| gx | 0.0001% (2건) | ~10σ |
| gy | 0% | 정상 |
| gz | 0% | 정상 |

→ **가속도에 spike 검출**, 자이로는 정지 시 거의 0.

#### 측정 2: 운동 5분 (정지/운동/정지 3구간)

**정지 1 (운동 전)** — SPI 환경 검증:
```
ax std: 0.001 g, spike: 0건
gz std: 0.05 dps, spike: 0건
```
→ SPI 통신 자체는 깨끗.

**운동 구간**:
```
gx max: 251 dps (105.6σ), spike >50σ: 8건
```
→ 운동 중 자이로 spike 검출.

**정지 2 (운동 후)** — 잔류 진동:
```
gx std: 0.59 dps (정지 1의 8.7배)
gy std: 0.51 dps (정지 1의 8.4배)
gz std: 0.63 dps (정지 1의 12.6배)
자이로 spike >50σ: 7건
```
→ 운동 후 자이로 잔류 진동.

#### 종합 결론

- **가속도**: 정지 1시간에서 spike 0.003% (max 164σ) → Median 필요
- **자이로**: 정지에선 0%이지만 **운동 중 gx 8건 + 정지 2에서 7건** 발견 → Median 필요
- **SPI 자체는 깨끗** (정지 1 = 0건). spike는 물리적 충격/운동 시 발생.
- **6축 동일 처리**: 코드 일관성 + 운동 환경 일관 대응

#### 비용 분석

- 위상 지연: 3-sample = 3.6ms @ 833Hz, 0.36ms @ 다운샘플 후 100Hz
- 메모리: 6축 × 3샘플 × float = 72 byte
- CPU: O(1) 정렬 (3-element swap), 무시 가능

#### 적용 위치

파이프라인 순서:
```
IMU raw → Median(3) → 평균 다운샘플(8) → INS
              ↑                  ↑
              spike 제거          노이즈 감소
```

Median 먼저, 평균 나중. 이유: spike 1건이 평균에 들어가면
8샘플 평균 결과를 1/8만큼 오염. Median으로 먼저 제거 후 평균.

### 소프트웨어 LPF 미채택

칩 내장 LPF (91Hz) + 8샘플 평균 + Median이 충분.
추가 SW LPF는:
- 위상 지연 추가 (응답 느려짐)
- 이득 미미

향후 90~200Hz 대역 진동 문제 시 ADR-XXX로 별도 검토.

## Alternatives Considered

### 대안 A: Median 미적용 (way 1)

- 단순한 시작
- **비채택 이유**: 가속도 정지 1시간 spike 0.003%, 운동 중 자이로 spike 검출

### 대안 B: 가속도만 Median (way 2)

- 자이로 정지 시 0% 신뢰
- **비채택 이유**: 운동 환경 측정에서 자이로 gx 8건 (105σ) 발견.
  운동 중 자이로 spike 무시 어려움.

### 대안 C: 6축 Median ✅ 채택 (way 3)

- 가속도 + 자이로 모두 측정 근거 있음
- 코드 일관성

### 대안 D: 5-sample Median

- 더 강한 spike 제거
- **비채택 이유**: 위상 지연 5/833 = 6ms로 증가, 효용 미미

## Consequences

### 긍정적 영향

1. **시간 예산 92% 여유** — ESKF, 추가 알고리즘 자유
2. **노이즈 √8 감소** — VRW 0.7e-3 → 0.25e-3 m/s²
3. **단발 spike 차단** — 정지/운동 환경 모두 대응
4. **운동 환경 검증 완료** — RC카 통합 시 신뢰성

### 부정적 영향 / Trade-off

1. **다운샘플링 위상 지연** — 8샘플 평균 = 4.5ms (사이클의 45%)
   - 100Hz에서 운동 응답 영향 미미
2. **Median 위상 지연** — 1샘플 (1.2ms @ 833Hz)
   - 무시 가능
3. **칩 LPF 의존** — 다른 IMU 교체 시 ADR 재검토 필요
4. **연속 spike 한계** — 3-sample Median은 1샘플 spike만 제거.
   2샘플 이상 연속 outlier는 그대로 통과.
   - 현재 데이터에선 연속 outlier 흔적 없음

## Implementation Notes

```cpp
// MedianFilter3 — O(1) 갱신, 3-element 정렬
struct MedianFilter3 {
    float buf[3] = {0, 0, 0};
    int idx = 0;
    int count = 0;
    
    float update(float x) {
        buf[idx] = x;
        idx = (idx + 1) % 3;
        if (count < 3) { count++; return x; }
        
        float a = buf[0], b = buf[1], c = buf[2];
        if (a > b) std::swap(a, b);
        if (b > c) std::swap(b, c);
        if (a > b) std::swap(a, b);
        return b;
    }
};

// 메인 루프
void main_loop() {
    constexpr int N_AVG = 8;
    MedianFilter3 mf_ax, mf_ay, mf_az;
    MedianFilter3 mf_gx, mf_gy, mf_gz;
    
    while (running) {
        auto t_start = now_us();
        
        // 8샘플 읽기 + Median 필터 + 평균
        float ax_sum = 0, ay_sum = 0, az_sum = 0;
        float gx_sum = 0, gy_sum = 0, gz_sum = 0;
        
        for (int i = 0; i < N_AVG; ++i) {
            ImuSample s;
            imu.read(s);
            
            // Median 적용 (1샘플 spike 제거)
            ax_sum += mf_ax.update(s.ax);
            ay_sum += mf_ay.update(s.ay);
            az_sum += mf_az.update(s.az);
            gx_sum += mf_gx.update(s.gx);
            gy_sum += mf_gy.update(s.gy);
            gz_sum += mf_gz.update(s.gz);
        }
        
        // 평균 (노이즈 √8 감소)
        ImuSample avg = {
            .ax = ax_sum / N_AVG, .ay = ay_sum / N_AVG, .az = az_sum / N_AVG,
            .gx = gx_sum / N_AVG, .gy = gy_sum / N_AVG, .gz = gz_sum / N_AVG,
        };
        
        // INS 처리
        attitude.update(avg, dt);
        navigator.predict(avg, attitude.get(), dt);
        telemetry.send(navigator.state());
        
        sleep_until(t_start + 10000);
    }
}
```

## References

- ADR-001: Bias 시간 상수 (선행)
- ADR-002: Quaternion 자세 표현 (선행)
- 측정 1 (정지 1시간): `~/zybo_dev/allan/imu_1h.csv`
- 측정 2 (운동 5분): `~/zybo_dev/allan/imu_motion.csv`
- 분석 도구: `spike_check.cpp`, `segment_check.cpp`
- 시간 예산: 본 ADR Section "메인 루프 100Hz"