# Autonomous_Driving

**LQR-PID 기반 F1TENTH 경로 추종(Path Tracking) 제어기**

2025-1 자율주행 프로그래밍 A조 (김규린, 김현지, 이하경)

ROS 2 / F1TENTH Gym 환경에서 동작하는 자율주행 레이스카의 **경로 추종 제어** 프로젝트입니다. 조향 제어에는 **LQR(Linear Quadratic Regulator)**, 속도 제어에는 **PID**를 결합한 `lqr_pid` 제어기를 직접 구현하고, 베이스라인인 **Pure Pursuit** 제어기와 랩타임 · 평균 Cross Track Error(CTE)를 비교 실험했습니다.

## 프로젝트 배경 및 목표

기존 경로 추종 기법(Pure Pursuit, Stanley)은 구현이 간단하지만 고속·급곡률 구간에서 추종 정밀도가 떨어지는 한계가 있습니다. 관련 연구에서 LQR 제어가 두 기법보다 낮은 오차로 더 안정적인 경로 추종 성능을 보인다는 점에 착안해, 아래 두 목표로 프로젝트를 설계했습니다.

1. **경로 추종 정밀도 향상** — LQR의 상태 피드백 기반 조향 보정 (crosstrack error + heading error 동시 최소화)
2. **속도 안정성 및 랩타임 단축** — PID 기반 속도 제어 + 곡률 기반 감속 게인으로 코너에서는 감속, 직선에서는 가속

시간·구현 복잡도 제약으로 동역학(dynamic) 모델 대신 **Kinematic 모델** 기반으로 시스템을 구성했습니다.

## 저장소 구조

`main` 브랜치는 실험에 사용된 최종 구성(`lqr_pid_labtime_CTE`)을 병합한 상태입니다.

```
.
├── lqr_pid/                 # 핵심 제어기 패키지 (C++, ROS 2)
│   ├── src/lqr_pid_node.cpp #   LQRPID 노드: LQR 조향 + PID 속도 제어
│   ├── include/lqr_pid/lqr_pid.hpp
│   ├── config/sim_config.yaml
│   ├── racelines/           #   basic.csv, wall1.csv (raceline waypoints)
│   └── launch/sim_lqr_pid_launch.py
├── pp.cpp / pp.hpp          # 비교 실험용 Pure Pursuit 베이스라인 (CL2-UWaterloo 기반)
├── bringup/                 # 시뮬레이션/실차 구동 launch 모음 (조이스틱 teleop, mux 등)
└── 변경 시 유의사항          # 맵 교체 시 수정해야 할 파일 체크리스트 (개발 노트)
```

> `lqr_pid`는 F1TENTH 워크스페이스(`f1tenth_ws`)의 `src/` 하위 패키지로 동작하도록 만들어졌으며, `particle_filter`(측위)·`f1tenth_gym_ros`(시뮬레이터) 등 워크스페이스의 다른 패키지가 함께 있어야 합니다. 토픽 흐름과 노드 구조는 `pure_pursuit`, `mpc` 패키지를 참고해 설계했습니다.

## 알고리즘 개요

### LQR 조향 제어 (`lqr_steering`)
차량을 4차 선형 상태공간(`crosstrack error`, `crosstrack error rate`, `heading error`, `yaw rate`)으로 모델링하고, Riccati 방정식(`solve_dare`)을 풀어 얻은 최적 이득 행렬 `K`(`dlqr`)로 피드백 조향각을 계산합니다. 최종 조향각은 곡률 기반 피드포워드(`ff`)와 LQR 피드백(`fb`)의 합으로 결정되며, `max_steer_rad`로 클램핑됩니다.

### PID 속도 제어 (`pid_velocity`)
현재 속도와 raceline상의 목표 속도(`vx_mps`) 오차를 비례·적분·미분항으로 보정합니다. 곡률 `kappa`가 클수록 게인을 낮추는 `1/(kappa+ε)` 스케일링을 적용해, 코너에서는 자동으로 감속하고 직선에서는 가속하도록 설계했습니다.

### 랩타임 / CTE 측정 (`odom_callback`)
차량이 출발 지점(1m 이내)으로 복귀하면 랩을 종료 처리하고, 랩 동안의 총 주행 시간과 평균 CTE(`total_cte / cte_count`)를 로그로 출력합니다.

## 빌드 & 실행

F1TENTH ROS 2 워크스페이스(`f1tenth_ws`) 안에 이 저장소의 패키지들을 위치시킨 후:

```bash
cd ~/f1tenth_ws
colcon build --packages-select lqr_pid bringup
source install/setup.bash
ros2 launch lqr_pid sim_lqr_pid_launch.py
```

주요 파라미터는 `lqr_pid/config/sim_config.yaml`에서 조정합니다.

| 파라미터 | 설명 | 기본값 |
|---|---|---|
| `lqr_q_y` | crosstrack error 가중치 (Q) | 200.0 |
| `lqr_q_yaw` | heading error 가중치 (Q) | 100.0 |
| `lqr_r_steer` | 조향 입력 가중치 (R) | 1.3 |
| `pid_kp` / `pid_ki` / `pid_kd` | 속도 PID 게인 | 1.0 / 0.1 / 0.1 |
| `wheelbase` | 차량 휠베이스 (m) | 0.33 |
| `max_speed` / `max_steer_rad` | 속도·조향 한계값 (mpc 패키지 값과 동일하게 설정) | 6.0 / 0.4189 |

맵을 교체할 때 수정해야 할 파일 목록은 `변경 시 유의사항` 문서를 참고하세요.

## 실험 결과

`lqr_pid`와 `pure_pursuit`을 동일한 속도·조향 한계값 조건에서 비교했습니다 (평가지표: 랩타임, 평균 CTE).

| 맵 | 제어기 | 랩타임 | 평균 CTE |
|---|---|---|---|
| Basic map | Pure Pursuit | 16.13 s | 0.068 m |
| Basic map | **LQR-PID** | **14.08 s** (−12.7%) | 0.124 m |
| wall1 map (급커브 포함) | Pure Pursuit | 22.28 s | 0.079 m |
| wall1 map (급커브 포함) | **LQR-PID** | **21.81 s** (−2.1%) | 0.1117 m |

- **LQR-PID**는 두 맵 모두에서 더 빠른 랩타임을 기록했습니다. 곡률 기반 피드포워드 조향으로 보다 공격적인 경로 추종이 가능했기 때문으로 분석됩니다.
- 반면 평균 CTE는 **Pure Pursuit**가 더 낮았습니다. LQR의 높은 응답성 때문에 급커브 구간에서 미세한 흔들림(jerk)이 관찰되었고, 이는 빠른 랩타임과의 트레이드오프로 해석됩니다.
- LQR-PID는 두 맵에서 경로 이탈 없이 안정적으로 완주했습니다.

## 브랜치 히스토리

각 브랜치는 개발 과정에서의 실험 단계를 나타냅니다 (참고용으로 보존).

| 브랜치 | 역할 |
|---|---|
| `main` | **병합된 최종 결과물** — `lqr_pid_labtime_CTE`를 병합한 안정 버전 |
| `lqr_pid_labtime_CTE` | ★ 최종 실험 브랜치. `lqr_pid` 패키지 + Pure Pursuit 비교 코드(`pp.cpp/hpp`) + 랩타임/평균 CTE 로깅까지 모두 포함. 보고서의 성능 비교 결과가 이 브랜치 기준. |
| `lqr_pid` | `lqr_pid` 패키지의 반복 개발/디버깅 브랜치 (bringup 정리, csv 오류 수정 등 다수 커밋) |
| `lqr_pid2` | `lqr_pid` 패키지 초기 버전 (bringup 미포함) |
| `lqr_pid_labtime` | 랩타임 로깅만 추가한 실험 브랜치 (CTE 로깅 없음, 단일 파일 구성) |
| `lqr_c++` | 초기 LQR 프로토타입 (패키지화 이전, 단일 cpp 파일) |
| `pid_c++` | Pure Pursuit + PID 베이스라인 패키지 (성능 비교 기준) |
| `mpc`, `mpc_2` | 검토했던 Kinematic MPC 방식 실험 (파라미터 튜닝 부담으로 최종 채택하지 않음) |

## 한계 및 향후 개선 방향

- **재현성**: ROS 2/DDS 특성상 동일 코드·파라미터라도 실행 환경(CPU, 스레드 스케줄링)에 따라 주행 결과가 달라짐 → 시간 동기화, QoS 설정, `rclcpp::Rate` 등 실시간성 보완 필요
- **모델 한계**: 현재 Kinematic 모델은 고속·급곡선·급가감속 구간에서 차량의 동적 특성을 충분히 반영하지 못함 → 질량, 관성, 마찰 계수, 조향 반응 속도를 포함한 Dynamic 모델 기반 LQR로 확장 예정

## 참고 문헌

김형규, 이명규, 김종탁, 김원균, "자율주행 차량의 고속 주행 안정성 향상을 위한 곡률 기반 경로 추종 제어 알고리즘 개발 및 검증," 한국정밀공학회지, vol.41, no.6, pp. 435-449, 2024.
