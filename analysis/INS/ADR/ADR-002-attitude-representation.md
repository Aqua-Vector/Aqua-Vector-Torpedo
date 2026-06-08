# ADR-002: 자세 표현 — Quaternion 채택

## Status
**Accepted** — 2026-04-30

## Context

INS 알고리즘은 IMU 자이로 측정값을 적분하여 자세(attitude)를 추정한다.
자세를 어떤 수학적 형식으로 저장·전파할지 결정해야 한다.

## Decision

**Hamilton 쿼터니언 q = (w, x, y, z)** 으로 자세 저장.
외부 인터페이스는 사용 편의를 위해 Euler 각(roll, pitch, yaw)도 제공.

## Rationale

### 비교 표

| 항목 | Euler | DCM (3×3) | **Quaternion** |
|---|---|---|---|
| 메모리 (float) | 3 | 9 | 4 |
| 짐벌락 | 있음 | 없음 | 없음 |
| 적분 안정성 | 낮음 | 중간 | 높음 |
| 보간 | 어려움 | 어려움 | SLERP 가능 |
| 좌표 변환 | 행렬 변환 필요 | 직접 곱 | 회전 공식 |
| 직관성 | 좋음 | 중간 | 낮음 |
| 정규화 | 불필요 | 매번 필요 | 매번 필요 (단순) |

### 우리 환경 적용

1. **2D 평면 운동** — 주로 yaw, 작은 roll/pitch
   - 그러나 RC카 회전·가감속 시 작지만 non-zero pitch/roll 존재
   - Euler로도 가능하지만 마진 없음
   
2. **향후 3D 확장** — 수중 어뢰 본 모델은 6DoF 운동
   - 처음부터 Quaternion 쓰면 코드 재사용 가능
   - Euler → Quaternion 마이그레이션 비용 회피

3. **표준 정합성** — Mahony, Madgwick, ESKF 모든 표준 알고리즘이 Quaternion
   - 논문 코드 직접 적용 가능
   - 향후 ESKF 도입 시 자연스러움

4. **수치 안정성** — 833Hz × 1시간 = 300만 적분
   - Quaternion 미분 dq/dt = 0.5 q ⊗ ω 는 선형
   - Euler는 비선형 (sin/cos), 작은 오차 누적 가능

## Alternatives Considered

### 대안 A: Euler 각 (roll, pitch, yaw)
- **장점**: 직관적, 디버깅 쉬움, 메모리 적음 (3개)
- **단점**: 짐벌락, 적분 비선형, 3D 확장 시 재작성 필요
- **검토 결과**: 작은 마진 + 3D 확장 가능성 때문에 비채택

### 대안 B: DCM (Direction Cosine Matrix)
- **장점**: 좌표 변환 직관적, 짐벌락 없음
- **단점**: 메모리 9개, 정규화 시 직교성 유지 어려움 (3×3 행렬의 9개 자유도 vs 회전의 3개 자유도, 6개 제약)
- **검토 결과**: 메모리 비효율 + 정규화 복잡으로 비채택

### 대안 C: Hamilton Quaternion ✅ 채택
- 표준 + 효율 + 확장성

## Consequences

### 긍정적 영향
1. **표준 알고리즘 직접 적용** — Mahony, Madgwick, ESKF 모두 Quaternion 기반
2. **수치 안정성** — 짐벌락 없음, 선형 미분 방정식
3. **메모리 효율** — DCM 대비 절반
4. **3D 확장 비용 0** — 동일 코드 재사용
5. **SLERP 가능** — 자세 보간 필요 시 (예: 시뮬레이터 데이터 생성)

### 부정적 영향 / Trade-off
1. **직관 떨어짐** — 4D 공간 추상화, 디버깅 시 인간이 이해하기 어려움
   - **완화책**: 외부 인터페이스에서 Euler로 변환해 출력
2. **연산 약간 증가** — Hamilton product (16번 곱셈, 12번 덧셈)
   - **완화책**: ZYBO Cortex-A9 충분 (833Hz에서 부담 없음)
3. **정규화 필요** — \|q\|=1 유지를 위해 매 적분 후 정규화
   - **완화책**: 단순 sqrt 한 번 (10ns 수준)

## Implementation Notes

### 데이터 구조

```cpp
// include/torpedo/domain/attitude/quaternion.hpp
namespace torpedo {

struct Quaternion {
    float w, x, y, z;     // Hamilton 규약: w가 스칼라
    
    // 단위 쿼터니언 (자세 회전 없음)
    static Quaternion identity() { return {1, 0, 0, 0}; }
    
    // 정규화 (단위 길이로)
    void normalize() {
        float n = std::sqrt(w*w + x*x + y*y + z*z);
        if (n > 1e-9f) { w/=n; x/=n; y/=n; z/=n; }
    }
    
    // Hamilton product (q1 ⊗ q2)
    Quaternion operator*(const Quaternion& q) const;
    
    // Euler 변환 (디버깅용)
    void to_euler(float& roll, float& pitch, float& yaw) const;
    
    // 자이로 적분 (작은 각도 근사)
    // dq = 0.5 * q ⊗ (0, ωx, ωy, ωz)
    static Quaternion from_gyro(float gx, float gy, float gz, float dt);
};

} // namespace torpedo
```

### 외부 출력 인터페이스

`ImuSample` 같은 사용자 대상 출력은 **Euler로 변환**:

```cpp
struct AttitudeOutput {
    float roll_deg, pitch_deg, yaw_deg;   // 디버그/표시용
    Quaternion q_internal;                  // 알고리즘 입력용
};
```

## References

- Diebel, "Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors", 2006
- Sola, "Quaternion kinematics for the error-state Kalman filter", 2017
- Hamilton, "On Quaternions" (1843) — 원조 정의
- ADR-001: Bias 추정 시간 상수 (선행 결정)