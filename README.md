# AG-CoNav

**Aerial-Ground Cooperative Navigation — 이기종 3로봇 통합 시뮬레이션**

> 새 알고리즘 연구가 아니라, 기존 라이브러리(ROS2 · Gazebo · Nav2 · grid_map/elevation_mapping · robot_localization)를 조합해 **드론·4륜·4족 3대가 하나의 시뮬레이션에서 함께 동작하고, 세 로봇의 지도를 하나로 통합**하는 **통합 엔지니어링 과제**. (한양대학교 UNICONLAB 인턴)

---

## 1. 프로젝트 개요

미지의 도시 환경에서 이기종 로봇 3대가 협력해 지도를 만들고 목표를 탐지한다.

- **드론(drone)** — 수동(사람이 pose 경로 지정), 상공에서 하향 LiDAR로 먼저 지형을 훑어 **2.5D 지도**를 만든다.
- **4륜(wheel, Husky A300)** — 개활지·연속 도로를 빠르게 이동한다.
- **4족(leg, Unitree Go2 + CHAMP)** — 낮은 장애물이 막은 길 등 지상을 이동한다.

지상 로봇은 SLAM을 새로 돌리지 않는다(드론 지도가 이미 있음). 드론 2.5D 지도에서 **로봇별 주행 가능 맵(wheel용·leg용)을 분리**하고, 각 로봇이 자기 주행맵으로 **Nav2** 이동하면서 자기 LiDAR로 주변 지형을 **자기 2.5D 지도로 누적**한다. 마지막에 **드론·4륜·4족 세 2.5D 지도를 하나로 병합**한다.

### 근본 목표 (이 둘이 메인)

1. **3대 이기종 로봇이 하나의 시뮬레이션에서 동시 구동**된다.
2. **세 로봇의 지도를 하나로 통합**한다.

> 정밀 탐지·임무 배분·LLM·RL은 현재 범위에서 제외(컷).

### 미션 플로우

```
드론 탐지(고도 84m, 하향)  →  드론 2.5D 지도 생성
        │
        ▼
wheel 주행가능 맵 · leg 주행가능 맵 분리   ← 드론 2.5D 에서 로봇별 주행 영역 산출
        │
        ▼
각 로봇이 자기 주행맵으로 Nav2 이동  +  이동 중 자기 2.5D 지도 누적
        │
        ▼
드론 + wheel + leg  세 2.5D 지도 → 하나로 병합(/merged_map)   ← 최종 결과물
        │
        ▼
     복귀  →  로봇 출동
        │
        ▼
   RViz2 로 통합 시각화
```

---

## 2. 확정 사항 (Fixed)

### 2.1 환경 · 버전

| 항목 | 값 |
| --- | --- |
| OS | Ubuntu 24.04 LTS (Noble) |
| 미들웨어 | ROS 2 **Jazzy Jalisco** (LTS ~2029) |
| 시뮬레이터 | **Gazebo Harmonic** (gz-sim 8, LTS ~2028) |
| 내비게이션 | Nav2 (Jazzy apt) |
| 2.5D 지도화 | `grid_map` + `elevation_mapping` |
| 위치추정 | `robot_localization` (EKF + navsat) |
| 브리지 / 시각화 / 로깅 | `ros_gz` / RViz2 / rosbag2(mcap) |
| 언어 | Python 3.12(시스템, **venv 미사용**) / C++17 |
| 빌드 | `colcon build --symlink-install` |
| RMW / DOMAIN | `rmw_fastrtps_cpp` / `ROS_DOMAIN_ID=42` (전원 동일) |

### 2.2 지형 · 맵

| 항목 | 값 |
| --- | --- |
| 장소 | **서울 성수동** (일반 도시, 숲·계단 없음) |
| 좌표 원점(datum) | **37.5412278, 127.0565741** |
| 크기 | **500 m × 500 m** |
| 4족(leg)용 조건 | **낮은 장애물로 길 막기** |
| 4륜(wheel)용 조건 | **끊기지 않은 연속 도로** |
| map 원점 | Gazebo world 원점 (0,0,0)와 일치 |

### 2.3 로봇 · 센서

| 항목 | 값 |
| --- | --- |
| 드론 | Gazebo 멀티콥터, **수동 pose 이동**(kinematic, 자율비행 없음) |
| 4륜(wheel) | Clearpath **Husky A300** |
| 4족(leg) | Unitree **Go2 + CHAMP** (`unitree_go2_ros2_jazzy`) |
| 센서 | **Ouster OS1-32 (3D LiDAR) — 3대 통일** |
| 드론 LiDAR | **하향 장착**, 탐지 고도 **84 m** |
| GPS / IMU | GPS 적극 활용(GT 아님) + **IMU 사용**(skid-steer·보행 yaw 드리프트 보정) |

**OS1-32 스펙**: 32채널 / 수직 FOV 42.4°(±21.2°) / 수평 360° / 사거리 0.5–170 m(80% 반사)·90 m(10%) / 최소 0.5 m / 10–20 Hz / 865 nm / 최대 2 returns.

### 2.4 지도화 · 위치추정 · 주행

| 항목 | 값 |
| --- | --- |
| SLAM 방식 | **2.5D SLAM** (elevation 격자) |
| 드론 지도 | 하향 스캔 → 2.5D 고도맵 |
| 주행 가능 맵 | 드론 2.5D → **wheel용·leg용 2D 주행맵 분리** (로봇별 지형 통과 기준) |
| 위치추정 | robot_localization(EKF + navsat), GPS 기반, **GT 사용 안 함** |
| 지상 주행 | **Nav2 공통 설정**으로 wheel·leg, **각자 주행맵** 사용. Voxel Layer로 점군 직접(LaserScan 없음) |
| 지상 지도 | SLAM 아님 → 자기 주행맵 위 Navigation + 자기 elevation 지도 누적 |
| 맵 병합 | `multirobot_map_merge` Jazzy 미지원 → **커스텀**(`agconav_map_fusion`) |

---

## 3. 공통 규약 (Conventions) — 모듈이 맞물리는 접점

### 3.1 좌표 · 프레임 · TF

- 전역 프레임 **`map` 하나**, 원점 = **Gazebo world (0,0,0)**. map=ENU, base_link=FLU, 오른손 좌표계.
- TF 사슬: `map → X/odom → X/base_link → X/{os1_lidar, gps_link}` (X = drone/wheel/leg).
- **TF 소유권 — 한 관계에 발행자 하나.**
  - `map→X/odom` = 위치추정(robot_localization)만 (wheel·leg)
  - `X/odom→X/base_link` = 시뮬 오도메트리만 (wheel·leg)
  - **드론**: kinematic이라 `drone_path_player`가 명령 pose로 **`map→drone/base_link`를 직접 발행**(odom·EKF 없음). 드론엔 사실상 명령 pose를 그대로 쓴다(수동 비행 경로 = 알고 있는 값).
  - `X/base_link→센서` = robot_state_publisher만
- `earth`/`utm` 프레임은 필요 확인 전까지 트리에 넣지 않는다.
- 드론 TF: `drone_path_player`가 명령 pose로 `map→drone/base_link` 직접 발행 — 3.1

### 3.2 단위 (SI)

| 물리량 | 단위 |
| --- | --- |
| 길이·위치 | m |
| 각도 | rad |
| 속도 / 각속도 | m/s / rad/s |
| 방향 | quaternion |
| 시간 | ROS Time (s), Gazebo `/clock` 기준 |

### 3.3 지도

- 해상도 **0.10 m/cell**(전 지도 동일 → 병합 시 리샘플 불필요) · 2.5D 핵심 레이어 **`elevation`**(m).
- **미관측 셀 = `NaN`** (grid_map 표준). Nav2용 2D(OccupancyGrid) 투영 시 자유 0 / 점유 100 / 미관측 −1(NaN→−1).
- **주행성 통과 기준(F)**: wheel = 최대 경사 20°·최대 단차 **0.08 m**, leg = 최대 경사 30°·최대 단차 **0.15 m**. → 월드의 낮은 장애물은 **정확히 0.10 m**(wheel 막힘·leg 통과)로 배치해야 두 nav_map이 갈린다. (0.11 m↑는 leg 판정 불안정 구간, 값은 yaml 튜닝)
- **병합 규칙(E)**: 같은 해상도 전제, 출력 = 세 입력의 합집합 범위. 중복 셀은 **지상(wheel/leg) 관측 우선 → 드론**(가림영역 세부 보완 목적), 유효값을 NaN으로 덮지 않음.
- **저장 형식**: 2.5D elevation = **rosbag2 `mcap`으로 GridMap 직렬화**, 2D nav_map/occupancy = **map_server `.yaml`+`.pgm`**.
- **미관측 셀** = `NaN` (2D 투영 시 −1) — 3.3
- **통과 기준**: wheel 20°/0.08 m, leg 30°/0.15 m — 3.3
- **병합**: 같은 해상도·합집합 범위·지상 우선 — 3.3
- **저장**: 2.5D=mcap(GridMap), 2D=map_server(yaml+pgm) — 3.3

### 3.4 시간 · 네임스페이스 · QoS

- 전 노드 `use_sim_time: true`, `/clock`의 유일 소스는 Gazebo.
- 네임스페이스 `/drone`, `/wheel`, `/leg`.
- QoS: 센서(points/gps/imu) = best_effort · 명령·odom = reliable · 지도(elevation/map/merged) = reliable + transient_local(래치).

### 3.5 환경 변수 (전원 `~/.bashrc`)

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## 4. 모듈 A~F (담당 · 책임 · 입출력)

각 모듈의 상세 스펙(범위 밖·전제조건·완료기준)은 별도 설계 문서로 관리한다. 아래는 요약.

| 모듈 | 담당 | 책임(한 줄) | 주요 입력 | 주요 출력 |
| --- | --- | --- | --- | --- |
| **A 드론 지도 생성** | 홍연주 | 드론을 pose 경로로 이동시키며 하향 LiDAR로 2.5D 지도 누적 | 경로 YAML, `/drone/points`, 드론 pose/TF | `/drone/elevation_map`, 저장 |
| **F 지형 주행성 분석** | 이종헌 | 드론 2.5D → **wheel/leg 주행가능 맵 분리** | `/drone/elevation_map`, 로봇별 통과기준 | `/wheel/nav_map`, `/leg/nav_map` |
| **B 지상 위치추정** | 이수빈 | wheel·leg의 GPS+odom을 융합해 공통 map 좌표 정렬 | `/X/gps`, `/X/odom` | `map→X/odom` TF, 필터 odom |
| **C 지상 Nav2 이동** | 이수빈 | 공통 Nav2로 wheel·leg를 **각자 주행맵**으로 목표까지 이동 | `/X/nav_map`, B의 TF, `/X/points` | `/X/cmd_vel`, 경로/상태 |
| **D 지상 지도 누적** | 채현우 | wheel·leg가 이동하며 주변 지형을 2.5D 지도로 누적 | `/X/points`, B의 pose/TF | `/wheel/elevation_map`, `/leg/elevation_map`, 저장 |
| **E 모든 지도 병합** | 채현우 | 세 2.5D 지도를 하나로 병합 | 3개 `elevation_map` | `/merged_map`, 저장 |

- **F(주행성 분석)**: 드론 2.5D에서 로봇별(경사·단차 기준) 통과 영역을 갈라 `/wheel/nav_map`·`/leg/nav_map`을 만든다. 낮은 장애물 = wheel 막힘 / leg 통과. Nav2 설정은 **공통 하나**, 로봇별 차이는 **입력 주행맵·footprint**뿐.
- D는 A의 지도 생성 구조를 재사용(협업: 홍연주 ↔ 채현우). E는 이미 map 프레임으로 정렬된 지도를 겹치기만 한다(정렬은 B).

---

## 5. 모듈 간 인터페이스 계약 (핵심 토픽)

`X` = drone / wheel / leg.

| 토픽 | 타입 | 발행 → 구독 | QoS |
| --- | --- | --- | --- |
| `/X/points` | `sensor_msgs/PointCloud2` | 브리지 → 지도화·Nav2 | best_effort |
| `/X/odom` | `nav_msgs/Odometry` | 브리지 → 위치추정 | reliable |
| `/X/gps` | `sensor_msgs/NavSatFix` | 브리지 → 위치추정 | best_effort |
| `/X/imu` | `sensor_msgs/Imu` | 브리지 → 위치추정(EKF) | best_effort |
| `/drone/cmd_pose` | `geometry_msgs/PoseStamped` | 드론 경로 재생 → 드론 | reliable |
| `/wheel/cmd_vel`·`/leg/cmd_vel` | `geometry_msgs/Twist` | Nav2 → 로봇 | reliable |
| `/X/elevation_map` | `grid_map_msgs/GridMap` (layer `elevation`) | 지도화 → 병합 | reliable, transient_local |
| `/wheel/nav_map`·`/leg/nav_map` | `nav_msgs/OccupancyGrid` | F(주행성 분석) → C(Nav2) | reliable, transient_local |
| `/merged_map` | `grid_map_msgs/GridMap` | 병합 → RViz·저장 | reliable, transient_local |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → all | best_effort |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | — | 기본 / latched |

**액션**: `/wheel/navigate_to_pose`, `/leg/navigate_to_pose` (`nav2_msgs/NavigateToPose`).
**커스텀 메시지·서비스: 없음**(표준 타입 + 라이브러리 제공분으로 충분).

---

## 6. 파일 · 폴더 구조

현재 저장소 구조(패키지 접두어 `agconav_`).

```
AG-CoNav/
├── README.md              # 이 문서
├── CONTRIBUTING.md        # 기여 규칙(브랜치·PR·커밋)
├── config/
├── src/
│   ├── agconav_worlds/           # 공통(이종헌)  서울 성수동 500×500 월드·지형
│   ├── agconav_description/      # 공통(이종헌)  로봇 3종 모델 + OS1-32/GPS/IMU, 정적 TF
│   ├── agconav_gz_bridge/        # 공통(이종헌)  Gazebo↔ROS2 브리지 설정
│   ├── agconav_bringup/          # 공통(이종헌)  전체 통합 launch(원클릭)
│   ├── agconav_drone/            # A(홍연주)     드론 2.5D 지도 생성
│   ├── agconav_traversability/   # F(이종헌)     드론 2.5D → wheel/leg 주행맵 분리
│   ├── agconav_localization/     # B(이수빈)     GPS/EKF 위치추정
│   ├── agconav_navigation/       # C(이수빈)     지상 공통 Nav2 이동
│   ├── agconav_ground_mapping/   # D(채현우)     지상 로봇 2.5D 지도 누적
│   ├── agconav_map_fusion/       # E(채현우)     세 지도 병합 (메인 결과물)
│   └── unitree_go2_ros2_jazzy/   # 외부          Go2 + CHAMP 통합 (vcstool, git에 직접 커밋 안 함)
├── deps.repos             # vcstool 외부 저장소 목록(URL+커밋 고정)
└── (build/ install/ log/ 는 colcon 산출물 — gitignore)
```

> 6개 모듈(A·F·B·C·D·E)이 각각 패키지로 매핑됨. `package.xml`/`CMakeLists.txt`는 각 담당이 구현 착수 시 추가.
>
> `src/unitree_go2_ros2_jazzy`는 메쉬 포함 ~170MB짜리 외부 저장소([RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy))라 이 저장소 git 히스토리에 직접 넣지 않는다. 루트 `deps.repos`에 URL과 커밋 해시를 고정해두고 `vcstool`로 받는다(10장 참조). `.gitignore`에도 등록되어 있어 로컬에 받아도 커밋되지 않는다.

---

## 7. 역할 · 소유권

모듈별 담당은 **4장 모듈표**(담당 컬럼)에 있다. 패키지·모듈 소유권 표와 기여 절차는 **[CONTRIBUTING.md](CONTRIBUTING.md)** 참조.

---

## 8. 확정된 세부 결정 (검증 완료) · 남은 튜닝

이전 7개 미결정은 아래 기본값으로 **확정**(상세는 3장). 남은 건 실측 튜닝·조율뿐.

**확정**

- 미관측 셀 = `NaN` (2D 투영 시 −1) — 3.3
- 통과 기준: wheel 20°/0.08 m, leg 30°/0.15 m — 3.3
- 병합: 같은 해상도·합집합 범위·지상 우선 — 3.3
- 저장: 2.5D=mcap(GridMap), 2D=map_server(yaml+pgm) — 3.3
- 드론 TF: `drone_path_player`가 명령 pose로 `map→drone/base_link` 직접 발행 — 3.1
- **지상 IMU 사용** (skid-steer·보행 yaw 드리프트 보정)
- **드론 스캔 경로: 간격 ≈ 32 m, 약 16줄**. 지면 스와스 65 m지만 가장자리 슬랜트 거리 ≈ 90 m가 OS1-32의 10% 반사율 사거리 한계라, **오버랩 ~50%**로 신뢰 스와스만 사용.

**남은 튜닝·조율**

1. 통과 기준 파라미터 실측 튜닝(위 값은 시작점).
2. **월드의 낮은 장애물 높이 0.10 m 배치** (worlds·F 모두 이종헌). wheel(0.08)와 leg(0.15) 통과 기준 사이이며, leg 판정이 안정적인 0.10 m로 맞춘다.
3. 지도 저장 경로·파일명 규칙(형식은 확정).

---

## 9. 설계 원칙

1. **연구가 아니라 통합.** 새 알고리즘을 만들지 않고 기존 라이브러리를 쓴다. 발표 때 "실제로 돌려봤는지"까지 보여준다.
2. **알고리즘은 하나로 통일.** wheel·leg에 같은 Nav2 설정. 로봇별 최적화 금지. **작동(목표 도착)만 되면 통과.**
3. **센서는 OS1-32로 통일.** SLAM/정렬 방식도 여기에 맞춘다.

> 코드 규약("설명 가능한 것만" · 토픽-only 결합 · 공통 규약 준수)과 Git 워크플로는 **[CONTRIBUTING.md](CONTRIBUTING.md)**.

---

## 10. 설치 · 실행 (요약)

```bash
# ROS2 Jazzy + 도구 (venv 안 씀, 시스템에 설치)
sudo apt install ros-jazzy-desktop gz-harmonic ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-robot-localization \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  python3-numpy python3-scipy python3-matplotlib python3-opencv \
  python3-vcstool
# grid_map / elevation_mapping / Husky A300 모델은 소스 빌드

git clone <이 저장소 URL> AG-CoNav && cd AG-CoNav

# 외부 패키지(Go2 + CHAMP, ~170MB)는 커밋되어 있지 않음 → vcstool로 받기
vcs import src < deps.repos

colcon build --symlink-install && source install/setup.bash
# ros2 launch agconav_bringup <통합 launch>   # 원클릭 실행 (구현 후)
```

> `deps.repos`에 등록된 외부 저장소를 최신 커밋으로 갱신하려면 `vcs pull src` 후 `deps.repos`의 `version`을 새 커밋 해시로 고쳐 커밋한다(임의 갱신 금지 — 팀 전원이 같은 커밋을 쓰기 위함).

---

## 11. 범위 밖 (컷 — 하지 않음)

MuJoCo/4족 RL · LLM 재배분 · 드론 자율비행 · 로봇별 알고리즘 최적화 · 정밀 목표 탐지(`/X/detections`) · 임무 배분 · 지상 독립 SLAM · LaserScan 변환.
(필요가 실제로 확인되기 전까지 추가하지 않는다.)
