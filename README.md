# Aqua-Vector-Torpedo
## 수중 어뢰 유도 시스템 (어뢰 노드)

3노드 분산 수중 어뢰 유도 시스템에서 **어뢰 노드**를 담당하는 임베디드 애플리케이션입니다.

IMU 기반 INS(관성 항법) 연산부터 통제소·구동부와의 실시간 통신,
PN(비례항법) 유도, 조타 명령 실행까지
어뢰가 표적까지 자율 유도되는 전 과정을 ZYBO 보드 위에서 수행하도록 설계했습니다.

## 기술 스택
- **Platform**: ZYBO Z7-10 (Zynq-7000, ARM Cortex-A9) / PetaLinux
- **Language**: C++17

- **Architecture**:
    - 계층형 설계 (Sensor → HAL → Domain → Guidance/Control → Core)
    - 인터페이스 기반 의존성 주입 (IImu, IPwmChannel, ICommDataStream 등)

- **Libraries**:
    - Eigen3: 행렬·쿼터니언 연산 (ESKF, 유도 계산)
    - pthread (std::thread): 통신 RX/TX 스레드 및 제어 루프 병렬 처리

- **Build**: CMake (3.16+), 크로스 컴파일 툴체인(`zynq_toolchain.cmake`)

## 프로젝트 개요
- 제작 이유

    수중 어뢰는 GPS가 닿지 않는 환경에서 스스로 위치를 추정하고
    표적까지 유도되어야 합니다.

    이 환경을 5m × 5m 평면의 RC카 시연으로 재현하여,
    IMU만으로 발생하는 위치 누적 오차(drift)를 외부 측정(LiDAR)으로 보정하고,
    추정된 위치를 바탕으로 표적을 추적하는
    **실시간 임베디드 유도 시스템**을 직접 설계·구현했습니다.

- 시스템 구성

    ```
    [통제소(GCS)] ──RS-485── [어뢰 (ZYBO)] ──UART── [STM32 (구동부)]
                                  │
                             [IMU (SPI/UART)]
    ```

    - 어뢰 노드는 IMU로 자세·가속도를 읽어 위치를 추정하고,
      통제소로부터 표적·LiDAR 좌표를 받아 유도 명령을 산출하며,
      STM32로 속도·조향 명령을 전달합니다.

## 주요 기능
- 9-state ESKF 기반 위치 / 속도 / 자세 통합 추정
- 발사 전 5초 정지 캘리브레이션 (Bias·초기 자세 추정)
- LiDAR 위치 측정 융합 및 NHC(비홀로노믹 제약) 보조 보정
- PN / Hybrid(PN + Pure Pursuit) 유도 알고리즘
- 중기 유도(MIDCOURSE) / 종말 유도(TERMINAL) 페이즈 전환
- RS-485·UART 양방향 통신 (CRC 검증 포함)
- 워치독 기반 FAILSAFE / LOCKDOWN 안전 로직
- 100Hz 실시간 제어 루프

## 핵심 동작 흐름
- INS / 추정

    IMU(ISM330DHCX, EBIMU) 측정값을 8샘플 다운샘플링(Trimmed Mean)으로
    spike·노이즈를 제거한 뒤, 9-state ESKF로 위치·속도·자세를 통합 추정합니다.

    - 발사 전 5초 정지 상태에서 가속도/자이로 bias와 초기 자세를 캘리브레이션
    - IMU 단독 적분으로 발생하는 drift를 LiDAR 위치 측정(100Hz, ±5cm)으로 보정
    - LiDAR 끊김 시 NHC(차량은 옆으로 미끄러지지 않는다는 제약)로 fallback

- 유도

    추정된 자기 위치와 통제소가 전달한 표적 좌표를 바탕으로
    조향각을 산출합니다.

    - 중기 유도: LiDAR 실측 표적 좌표 추적
    - 종말 유도: 추측항법(Dead Reckoning)으로 표적 위치 예측
    - 거리 기반으로 PN과 Pure Pursuit 성분을 블렌딩하여 조향 안정성 확보

- 제어 / 안전

    산출된 명령은 STM32로 전달되기 전 ModeMux에서 모드별로 검증됩니다.

    - 일정 시간 데이터 미수신 시 FAILSAFE 전환 (워치독)
    - 요격 완료·하드웨어 이상 시 LOCKDOWN
    - 모든 제어 출력은 범위 제한 및 NaN/Inf 검증 후 송신

## 프로젝트 구조
```
Aqua-Vector-Torpedo
├── include/                # 헤더
│   ├── torpedo/            # 어뢰 핵심 (sensor, domain/estimator, comm, hal)
│   ├── ebimu/              # EBIMU 기반 INS (sensor, domain, app)
│   ├── guidance/           # 유도 알고리즘 인터페이스 및 구현
│   ├── control/            # 모드 관리, 제어 소스
│   ├── protocol/           # 패킷 구조, 파서, 정책
│   ├── communication/      # UART / UDP 링크
│   └── utils/              # 큐, 필터, 수학·시간 유틸
│
├── src/                    # 구현부 (include 구조와 대응)
│   ├── sensor/             # IMU 드라이버 (SPI / UART)
│   ├── domain/estimator/   # ESKF, Bias 캘리브레이터, RPS 트래커
│   ├── guidance/           # PN / Hybrid 유도, 페이즈 관리
│   ├── control/            # ModeMux, 데이터 검증
│   ├── core/               # TorpedoControlSystem (전체 흐름 관장)
│   └── main.cpp            # 진입점
│
├── tests/                  # 단위 / 통합 / 하드웨어 테스트
├── analysis/INS/           # INS 설계 결정(ADR) 및 분석 코드
│   ├── ADR/                # 설계 의사결정 기록 (bias, ESKF, 샘플링 등)
│   └── codes/              # Allan Variance, spike 검출 분석 도구
├── docs/                   # 협업 규칙(RULE.md), 모듈 개발 보고서
│
├── CMakeLists.txt          # 통합 빌드 설정
├── zynq_toolchain.cmake    # ZYBO 크로스 컴파일 툴체인
└── README.md               # 프로젝트 설명 문서
```

## 빌드 & 실행
- 네이티브 (개발 / 테스트용)

    ```bash
    mkdir build && cd build
    cmake ..
    make -j
    ctest        # 단위·통합 테스트 실행
    ```

- ZYBO 크로스 컴파일

    ```bash
    mkdir build-zynq && cd build-zynq
    cmake -DCMAKE_TOOLCHAIN_FILE=../zynq_toolchain.cmake ..
    make -j
    ```

- 실행

    ```bash
    ./torpedo_main /dev/ttyPS1 /dev/spidev0.0
    ```

    실행 직후 5초 정지 캘리브레이션을 수행하므로 어뢰를 움직이지 마세요.

## 개발 목적 & 배운점
- 개발 목적

    LIG Nex1 The SSEN 임베디드 스쿨 과정에서 학습한
    Zynq/PetaLinux, 센서 드라이버, 실시간 제어, 칼만 필터 지식을
    하나의 통합 시스템으로 구현해보기 위해 진행한 프로젝트입니다.

    단순 모듈 구현이 아닌, IMU·통신·추정·유도·제어가
    100Hz 루프 안에서 유기적으로 동작하는
    실제 동작 가능한 자율 유도 시스템을 목표로 했습니다.

- 배운점

    - SPI / UART 센서 드라이버를 인터페이스(IImu)로 추상화하여
      실제 하드웨어 없이도 알고리즘을 단위 테스트하는 설계 경험
    - Allan Variance 측정으로 IMU 노이즈를 정량 분석하고,
      그 결과를 ESKF의 Q 행렬과 bias 시간 상수 설계 근거로 활용
    - 9-state Error-State Kalman Filter의 predict / update / inject 구현
    - RS-485 기반 양방향 패킷 통신과 CRC 검증, 비동기 RX/TX 스레드 설계
    - 워치독·FAILSAFE 등 실패 상황을 가정한 안전 로직 설계
    - 좌표계 변환(body ↔ nav, CW/CCW 부호 규약)에서 발생하는
      미묘한 버그를 추적·디버깅한 경험

## 설계 의사결정 (ADR)
주요 알고리즘 선택은 측정 데이터에 근거하여 결정하고,
그 과정을 `analysis/INS/ADR/`에 기록으로 남겼습니다.

- **자세 표현 — Quaternion 채택**: 짐벌락 회피, 수치 안정성, 3D 확장성
- **Bias 추정 시간 상수 분리**: Allan Variance 분석 결과 가속도(τ≈15s)와
  자이로(τ≈200s)의 최적 시간 상수가 한 자릿수 차이 → 분리 처리
- **9-state ESKF 채택 (vs 15-state)**: bias를 EMA로 별도 추정하는 결정과의
  일관성, 시연 환경 적합성, 일정 내 구현 안정성을 종합 판단
- **외부 측정 통합**: LiDAR(주) + NHC(보조) + 초기 캘리브의 다층 보정으로
  단일 측정 실패에 대한 robustness 확보
- **샘플링 / 필터링**: 833Hz IMU + 100Hz 메인 루프, 6축 Median + 8샘플 평균

## 개선 예정 사항
- 15-state full ESKF로 확장하여 운영 중 bias 변동 추정
- 다중 표적 환경 대응 및 회피 기동 로직
- 통신 패킷 손실에 대한 재전송·복구 정책 강화
- 실제 수중 환경(6DoF) 운동 모델로 확장

## 왜 이 구조인가?
"왜 모놀리식이 아닌 계층형·인터페이스 기반 설계인가?"라는 질문에 대해
본 프로젝트는 다음 가치에 집중했습니다.

1. 테스트 가능성 (Testability)

    센서·통신·구동기를 모두 인터페이스로 추상화했습니다.

    - `IImu`, `IPwmChannel`, `ICommDataStream` 등으로 하드웨어 의존성을 분리
    - FakeImu·Mock 링크를 주입하여 ZYBO 없이도 PC에서 알고리즘 검증
    - 단위·통합·하드웨어 테스트를 계층별로 분리하여 회귀를 빠르게 탐지

2. 데이터 기반 설계 (Data-Driven Design)

    알고리즘 파라미터를 임의로 정하지 않았습니다.

    - 1시간 정지 측정 + Allan Variance 분석으로 노이즈 특성을 정량화
    - 측정값(VRW, ARW, Bias Instability)을 그대로 ESKF 노이즈 모델에 반영
    - 모든 선택의 근거를 ADR 문서로 남겨 재현성과 설득력을 확보

3. 실패를 가정한 안전 설계 (Fail-Safe)

    실시간 제어 시스템은 정상 동작만큼 실패 처리가 중요합니다.

    - 워치독으로 통신 두절을 감지하여 FAILSAFE 전환
    - LiDAR 끊김 시 NHC fallback → 즉시 정지가 아닌 점진적 성능 저하
    - 요격·하드웨어 이상 시 LOCKDOWN으로 의도치 않은 재가동 차단

4. 확장 가능한 아키텍처 (Scalable Architecture)

    핵심 추정·유도 로직이 특정 하드웨어에 종속되지 않도록 설계하여,
    9-state → 15-state ESKF 확장이나 2D → 3D 운동 모델 전환 시
    인터페이스를 유지한 채 구현만 교체할 수 있는 토대를 마련했습니다.

## 요약
Aqua-Vector-Torpedo는 ZYBO/PetaLinux 위에서 동작하는
수중 어뢰 유도 시스템의 **어뢰 노드** 애플리케이션입니다.
IMU 기반 INS, LiDAR 융합 ESKF, PN 유도, 실시간 통신·제어를
100Hz 루프 안에서 통합하여 표적까지의 자율 유도를 구현했습니다.

아키텍처 설계 (계층형 + 인터페이스 기반)

본 프로젝트는 Sensor → HAL → Domain → Guidance/Control → Core의
계층형 구조와 인터페이스 기반 의존성 주입으로
하드웨어와 알고리즘 로직을 분리했습니다. 이를 통해 다음을 달성했습니다.

- 테스트 용이성

    하드웨어를 인터페이스로 추상화하여 FakeImu·Mock 통신 링크로
    실제 보드 없이도 ESKF·유도·제어 로직을 단위 테스트할 수 있는
    구조를 확보했습니다.

- 정량적 설계

    Allan Variance 등 실측 데이터를 기반으로 알고리즘 파라미터를 결정하고,
    모든 의사결정을 ADR로 문서화하여 설계의 근거와 재현성을 확보했습니다.

- 확장성 확보

    핵심 추정·유도 로직이 UI나 특정 센서에 종속되지 않도록 설계하여,
    더 높은 차원의 ESKF나 6DoF 운동 모델로의 확장을 고려한
    유연한 토대를 마련했습니다.

주요 특징 및 기술적 성과
- 실시간 통합 추정

    833Hz IMU 입력을 8샘플 다운샘플링·Median 필터로 정제한 뒤
    9-state ESKF로 위치·속도·자세를 통합 추정하고,
    LiDAR 측정과 NHC로 누적 오차를 실시간 보정합니다.

- 다층 안전 설계

    워치독, FAILSAFE, LOCKDOWN, fallback 등
    실패 상황을 가정한 안전 로직을 시스템 전반에 적용하여
    신뢰성 있는 실시간 제어를 구현했습니다.

- 통합 및 디버깅 역량

    센서·통신·추정·유도·제어 다섯 영역을 하나의 루프로 통합하는 과정에서
    좌표계 부호 규약, 데이터 동기화, 스레드 경합 문제를 디버깅하며
    실시간 임베디드 시스템의 동작 원리를 깊이 있게 학습했습니다.

## 협업 규칙

[`RULE.md`](https://github.com/Aqua-Vector/Aqua-Vector-Torpedo/blob/master/docs/RULE.md) 참고
