# AG-CoNav 고정 규약 (Conventions)

> 이 문서는 **모듈이 서로 맞물리는 접점**과 **공통 규약**을 고정한다. 각 모듈 내부 구현(알고리즘·노드 구조·언어·하이퍼파라미터)은 이 계약만 지키면 자유다.
>
> 상태 표기: **[확정]** 합의됨 · **[기본값]** 담당자가 정한 작업 기본값(이의 없으면 확정) · **[합의필요]** 팀 논의 필요.
> 변경 시 이 문서를 PR로 수정하고 전원 공지한다.

---

## 1. 환경 · 버전 [확정]

| 항목 | 값 |
| --- | --- |
| OS | Ubuntu 24.04 LTS |
| 미들웨어 | ROS 2 Jazzy Jalisco |
| 메인 시뮬 | Gazebo Harmonic (gz-sim 8) |
| 내비 | Nav2 (Jazzy apt 바이너리) |
| RL 시뮬 | MuJoCo 3.10.x |
| 언어 | Python 3.12 (Jazzy 기본) / C++17 |
| 빌드 | colcon (`colcon build --symlink-install`) |
| RMW | `rmw_fastrtps_cpp` (Fast DDS, Jazzy 기본) |
| **ROS_DOMAIN_ID** | **42** — 전원 동일. 같은 네트워크의 다른 팀과 토픽 격리 |

셋업 시 `~/.bashrc` 에 고정:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 2. 좌표계 · 프레임 [기본값, REP-103/105]

- **전역 공유 프레임: `map` 하나.** 원점은 **Gazebo world 원점 (0,0,0)** 과 일치시킨다. 드론이 배포하는 모든 맵·목표 좌표는 이 `map` 기준.
- **오른손 좌표계 · ENU**: x=동(전방), y=북(좌), z=위. 회전은 rad.
- 로봇별 TF 트리 (네임스페이스 접두):

```
map
 ├─ drone/odom ─ drone/base_link ─ drone/lidar
 ├─ husky/odom ─ husky/base_link ─ husky/{lidar,camera}
 └─ go2/odom   ─ go2/base_link   ─ go2/{lidar,camera}
```

- MVP 단계에서는 각 로봇의 `map→odom`을 **Gazebo ground-truth pose**로 발행(정확 위치). SLAM은 고도화에서 대체.

## 3. 단위 [확정, SI 통일]

| 물리량 | 단위 |
| --- | --- |
| 길이 / 위치 | m |
| 속도 | m/s |
| 각도 | rad |
| 각속도 | rad/s |
| 시간 | s (ROS Time) |
| 질량 | kg |

- **격자맵 해상도: 0.10 m/cell** [기본값]
- 고도(elevation) 값: m
- **traversability 값: float `0.0 ~ 1.0`** (1.0 = 완전 통과가능, 0.0 = 불가), **미관측 셀 = `NaN`**. Nav2 costmap 변환 시 `cost = (1.0 - trav) * 254`, NaN → unknown(255).

## 4. 네이밍 · 네임스페이스 [기본값]

- 로봇 네임스페이스: **`drone` · `husky` · `go2`**
- 로봇 개별 토픽: `/<ns>/...` (예: `/husky/scan`, `/go2/cmd_vel`)
- 공유 토픽: `/map/*`, `/goals`, `/mission/*`, `/events/*` (아래 계약 참조)
- ROS 패키지명: 소문자+언더스코어, **접두어 `ag_`** (`ag_msgs`, `ag_drone`, `ag_orchestration` …). 폴더명은 현행 유지.
- 커스텀 메시지 패키지: **`ag_msgs`** (위치: `integration/`)

## 5. 시간 동기화 [확정]

- 모든 노드 **`use_sim_time: true`**
- **`/clock`의 유일 소스는 Gazebo.** 별도 시계 발행 금지.

## 6. 모듈 간 인터페이스 계약 ★ (가장 중요) [기본값]

이 표가 "누가 무엇을 발행/구독하는가"의 계약서다. `ag_msgs`를 먼저 만들어 타입을 고정하면 전원 병렬 작업이 가능하다.

| 토픽 | 타입 | 발행 → 구독 | QoS |
| --- | --- | --- | --- |
| `/map/elevation` | `grid_map_msgs/GridMap` (layer `elevation`) | drone → all | reliable, **transient_local**, depth 1 |
| `/map/trav_wheeled` | `grid_map_msgs/GridMap` (layer `traversability`) | drone → husky, orch | reliable, transient_local, depth 1 |
| `/map/trav_legged` | `grid_map_msgs/GridMap` (layer `traversability`) | drone → go2, orch | reliable, transient_local, depth 1 |
| `/goals` | `ag_msgs/GoalArray` | 각 로봇·drone → orch, orch → all | reliable, transient_local, depth 1 |
| `/mission/assignment` | `ag_msgs/AssignmentArray` | orch → robots | reliable, transient_local, depth 1 |
| `/events/failure` | `ag_msgs/FailureEvent` | robots → orch | reliable, volatile, depth 10 |

> 맵·목표·배정은 **transient_local(래치)** — 늦게 접속한 노드도 마지막 값을 받는다. 실패 이벤트는 스트림이라 volatile.

### `ag_msgs` 메시지 정의 [기본값]

```
# Goal.msg
uint32 id
geometry_msgs/PoseStamped pose      # frame_id = "map"
uint8 difficulty                    # 0 WHEELED_ONLY, 1 LEGGED_ONLY, 2 COMMON
uint8 type                          # 0 SUPPLY, 1 ALLY
uint8 status                        # 0 UNKNOWN,1 DETECTED,2 ASSIGNED,3 IN_PROGRESS,4 DONE,5 FAILED
uint8 WHEELED_ONLY=0
uint8 LEGGED_ONLY=1
uint8 COMMON=2
uint8 SUPPLY=0
uint8 ALLY=1

# GoalArray.msg
std_msgs/Header header
Goal[] goals

# Assignment.msg
uint32 goal_id
string robot_id                     # "husky" | "go2"
float32 est_cost                    # 이동시간+위험+통과가능성 종합 비용
builtin_interfaces/Time stamp

# AssignmentArray.msg
std_msgs/Header header
Assignment[] assignments

# FailureEvent.msg
std_msgs/Header header
string robot_id
uint32 goal_id
uint8 reason                        # 0 BLOCKED, 1 TIPOVER, 2 TIMEOUT
geometry_msgs/PoseStamped pose
string detail
```

### 배정 비용 함수 [기본값]

```
cost = w_t * 이동시간_추정
     + w_r * 위험(경사·잔해·전복 확률)
     + w_p * (1 - 통과가능성)
기본 가중치: w_t=1.0, w_r=1.0, w_p=2.0   # 통과 실패가 가장 치명적 → 가중 최대
```

## 7. 시나리오 · 환경 파라미터 [기본값]

- **맵 소스**: 기존 시가지 에셋 + 전쟁 잔해(rubble)만 배치. (직접 제작은 후순위)
- **월드 크기**: 약 50 m × 50 m 시가지 블록 (조정 가능)
- **로컬라이제이션**: 정적 맵 + ground-truth pose (MVP). SLAM은 고도화. **[합의필요 — 발표 전 확정]**
- **목표점**: **8개**, 난이도 비율 `WHEELED_ONLY:LEGGED_ONLY:COMMON = 3:3:2`
- 난이도 태깅 기준: 해당 셀을 어느 trav 맵이 통과가능(≥0.5)으로 판정하는지에 따라 자동 부여.

## 8. 로깅 · 실험 [기본값]

- 포맷: **rosbag2 `mcap`**. 저장 위치 `experiments/results/` (git 제외)
- 파일명: `<시나리오>_<YYYYMMDD>_<runNN>.mcap`
- 기본 기록 토픽: `/tf`, `/tf_static`, `/map/*`, `/goals`, `/mission/assignment`, `/events/failure`, 각 로봇 `/<ns>/odom`, `/<ns>/cmd_vel`
- 실험 설정은 YAML로 `experiments/configs/`:

```yaml
scenario: urban_supply_v1
seed: 0
goals: 8
ratio: {wheeled_only: 3, legged_only: 3, common: 2}
ablation:
  use_terrain_info: true      # 지형정보 사용/미사용
  assign_mode: optimal        # optimal | rule
  replan_mode: llm            # llm | rule
```

## 9. LLM [기본값]

- 인증: 키는 **환경변수 `OPENAI_API_KEY`**, `.env` 사용. **절대 커밋 금지**(`.gitignore` 포함).
- 모델명: **[합의필요]** 오케스트레이션 담당 확정.
- 규칙 재배분 vs LLM 재배분은 `ablation.replan_mode` 플래그로 스위치.

## 10. 코드 규약 [기본값]

- 브랜치: `<이름>` 개인 브랜치 → 작업 단위로 `main`에 PR (`CONTRIBUTING.md` 참조)
- 커밋: `<모듈>: <요약>` (예: `drone: 고도맵 생성 노드 추가`)
- 각 ROS 패키지는 `package.xml`에 라이선스 `MIT` 명시
- 파이썬 `ament_flake8`, C++ `ament_cpplint` 통과 권장

---

## 발표(7/13) 전 확정 필요 요약 [합의필요]

1. SLAM vs 정적맵 (기본값: 정적맵)
2. LLM 모델명
3. 월드/맵 최종 에셋

나머지는 위 기본값으로 즉시 착수 가능.