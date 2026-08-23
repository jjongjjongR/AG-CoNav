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
| 장소 | **서울 성동구** (일반 도시, 숲·계단 없음) |
| 좌표 원점(datum) | **37.54233814881853, 127.06050643805561** (`navsat_transform_node.yaml` · 월드 `<spherical_coordinates>` 동일) |
| 크기 | **577.883 m × 481.838 m** (`Seongdong_gu_aligned`, 축 정렬 월드 — `aligned_params.txt` 기준) |
| 4족(leg)용 조건 | **낮은 장애물로 길 막기** |
| 4륜(wheel)용 조건 | **끊기지 않은 연속 도로** |
| map 원점 | Gazebo world 원점 (0,0,0)와 일치 |

### 2.3 로봇 · 센서

| 항목 | 값 |
| --- | --- |
| 드론 | Gazebo 멀티콥터, **실제 로터 추력 비행**(`MulticopterVelocityControl`). `drone_velocity_follower` 가 스캔 경로를 `/drone/cmd_vel` 로 따라간다 — 자율탐색이 아니라 사전 경로 재생 |
| 4륜(wheel) | Clearpath **Husky A300** |
| 4족(leg) | Unitree **Go2** + `rl_quadruped_controller` (robot_lab 정책). CHAMP·`unitree_guide` 는 비교용으로만 유지 — **8.1 참고** |
| 센서 | **Ouster OS1-32 (3D LiDAR) — 3대 통일** |
| 드론 LiDAR | **하향 장착**, 탐지 고도 **84 m**, **방위각 창 ±45°(`<samples>256`)** — docs/8 |
| GPS / IMU | GPS 적극 활용(GT 아님) + **IMU 사용**(skid-steer·보행 yaw 드리프트 보정) |

**OS1-32 스펙**: 32채널 / 수직 FOV 42.4°(±21.2°) / 수평 360° / 사거리 0.5–170 m(80% 반사)·90 m(10%) / 최소 0.5 m / 10–20 Hz / 865 nm / 최대 2 returns.

**적용 설정(드론)**: 하드웨어 스펙은 수평 360° 지만 **방위각 창을 ±45° 로 좁혀 쓴다**(`<samples>` 1024→256, 각도 분해능은 2.84/deg 로 동일). 실제 OS1 도 `azimuth_window` 로 지원한다. 커버리지 손실 사실상 0 인데 wheel FN 이 소폭 개선된다 — **docs/8**.

> ⚠ **드론 모델 파일이 두 개다.** 실험 월드는 `agconav_test_worlds/models/agconav_drone_dynamic/model.sdf`,
> 운용은 `agconav_description/models/agconav_drone/model.sdf` 를 스폰한다. **한쪽만 고치면 반영되지 않는다.**
> 바꾼 뒤에는 점군의 `width` 와 θ 범위를 직접 확인할 것.
> **±30° 는 절대 쓰지 말 것** — wheel FN +1.5 %p (잡음의 25배).

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
  - **드론**: 월드의 `OdometryPublisher`(`odom_frame: map`, `robot_base_frame: drone/base_link`, `dimensions: 3`)가 내는 pose 를 `drone_tf_bridge` 가 `/model/X3/pose → /tf` 로 브리지한다(odom·EKF 없음). 드론은 gz 모델뿐이라 TF 를 발행하는 로봇 스택이 없어서 이 경로가 필요하다.
  - `X/base_link→센서` = robot_state_publisher만
- `earth`/`utm` 프레임은 필요 확인 전까지 트리에 넣지 않는다.
- 드론 TF: 월드 `OdometryPublisher` → `drone_tf_bridge` 가 `/tf` 로 브리지 — 3.1

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
- **주행성 통과 기준(F) — 실측 확정**: wheel = 최대 경사 **10°**·최대 단차 **0.08 m**, leg = 최대 경사 **15°**·최대 단차 **0.15 m**. 문헌값이 아니라 각 로봇을 계단식 램프·단차 하네스에서 **각 조건 3회 반복** 주행시켜 얻은 값이다(8.1). 낮은 장애물 **0.10 m** 는 여전히 wheel 막힘·leg 통과로 갈린다.
- **경사 기저 `slope_window: 3`**. 0.1 m 격자에서 창을 1로 두면 인접 셀 높이 잡음이 그대로 증폭된다 — 진짜 0.2~1.5° 인 지형이 5.96° 로 나왔고, 창 3 에서 1.76° 가 됐다.
- **병합 규칙(E)**: 같은 해상도 전제, 출력 = 세 입력의 합집합 범위. 중복 셀은 **지상(wheel/leg) 관측 우선 → 드론**(가림영역 세부 보완 목적), 유효값을 NaN으로 덮지 않음.
- **저장 형식**: 2.5D elevation = **rosbag2 `mcap`으로 GridMap 직렬화**, 2D nav_map/occupancy = **map_server `.yaml`+`.pgm`**.
- **미관측 셀** = `NaN` (2D 투영 시 −1) — 3.3
- **통과 기준**: wheel 10°/0.08 m, leg 15°/0.15 m (실측) — 3.3
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
| `/drone/cmd_vel` | `geometry_msgs/Twist` | `drone_velocity_follower` → 드론(gz 브리지) | reliable |
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
│   ├── agconav_worlds/           # 공통(홍연주)  서울 성수동 500×500 월드·지형
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
- 통과 기준: wheel **10°/0.08 m**, leg **15°/0.15 m** (실측 확정) — 3.3
- 병합: 같은 해상도·합집합 범위·지상 우선 — 3.3
- 저장: 2.5D=mcap(GridMap), 2D=map_server(yaml+pgm) — 3.3
- 드론 TF: 월드 `OdometryPublisher` → `drone_tf_bridge` 가 `/tf` 로 브리지 — 3.1
- **지상 IMU 사용** (skid-steer·보행 yaw 드리프트 보정)
- **드론 스캔 경로 — 실측 확정: 고도 84 m · 스트립 간격 3 m · 순항 6 m/s** (320 웨이포인트, 총 92.9 km). 간격 3 m 는 자유 공간이 단일 덩어리로 이어지는 가장 싼 값이다(단일성 99.1%; 2.5 m·4 m 는 36~38%). **속도가 지도 품질을 지배한다** — 같은 파이프라인에서 v10×6 은 wheel 자유 37.5%·경로계획 실패 19건·18.5 m 후 ABORTED, v6×3 은 자유 88.5%·실패 0건·191.5 m SUCCEEDED. 높이 오차 σ 가 0.536 m 대 **0.017 m** 로 31배 갈린다.

**남은 튜닝·조율**

1. ~~통과 기준 파라미터 실측 튜닝~~ → **완료**(8.1). 값은 각 yaml 에 잠금 주석과 함께 고정.
2. **월드의 낮은 장애물 높이 0.10 m 배치** (worlds·F 모두 이종헌). wheel(0.08)와 leg(0.15) 통과 기준 사이이며, leg 판정이 안정적인 0.10 m로 맞춘다.
3. 지도 저장 경로·파일명 규칙(형식은 확정).

---

### 8.1 실측 확정값 — 실험 근거

파라미터를 문헌에서 가져오지 않고 **직접 주행시켜 측정**했다. 상세는
`docs/` 아래 번호 문서에 있고, 각 설정 파일에는 잠금 주석이 달려 있다.

**드론 스캔 조건** (docs/2·4·9·10)

| 파라미터 | 확정값 | 근거 |
|---|---|---|
| 고도 | **84 m** | OS1-32 의 10% 반사율 사거리 90 m 에서 가장자리 슬랜트 거리 상한 |
| 순항 속도 | **6 m/s** | 6→8 m/s 에서 제어가 무너진다 — 선 이탈 0.053→**0.312 m**(6배), 고도 오차 0.610→**2.999 m**(5배) |
| 스트립 간격 | **3 m** | 자유 공간 **단일성** 99.1% (2.5 m 36.1%, 4 m 37.8%). 양이 아니라 *이어지는지*가 갈린다 |
| LiDAR 모드 | 1024×10 유지 | 상위 모드에서 커버리지가 포화 — 비행 간 편차(2.5%)보다 이득이 작다 |
| **방위각 창** | **±45° (`<samples>256`)** | 광선 1/4 · 지면 반사 94% 유지 · 커버리지 손실 사실상 0(994,634 vs 994,716 셀) · wheel FN 소폭 개선. **±30° 금지**(FN +1.5 %p) — docs/8 |

**4족 컨트롤러 비교** (docs/13·14) — 각 조건 3회 반복. docs/11 의 이전 측정은 관절 초기 자세 시딩이 깨진 상태여서 근거를 잃었다

| 컨트롤러 | 등판 한계 | 실패 양상 | 단차 | 종단 주행 |
|---|---|---|---|---|
| `unitree_guide` | **20°** | 25°에서 **전복** (이탈 1.51~3.54 m) | 150 mm (`gait_height 0.20`) | **실패 — 전복** |
| **`rl_quadruped` (robot_lab)** ← 운용 | 15° | 20°에서 **정지** (이탈 0.13~0.62 m) | 150 mm (기본값) | **성공** |
| legged_gym / himloco 정책 | 평지 보행 실패 | — | — | — |

> 등판만 보면 `unitree_guide` 가 5° 우세하지만 Nav2 자율주행에서 반복
> 전복해 운용에서 제외했다. 지형은 원인이 아니다 — 경로 191 m 의 최대
> 경사는 5.2°, 최대 단차는 12 mm 로 guide 한계의 1/4 이하다.
> **전말은 [docs/14](docs/14.%20한계점%20—%20unitree_guide%20를%20운용에%20못%20쓴%20이유.md).**

**4륜 (Husky A300)** (docs/12·13)

| 항목 | 결과 |
|---|---|
| 등판 | 10° 통과 3/3, 12° 정지 3/3 |
| 단차 × 속도 | 80 mm: 0.5·0.8 실패 / 1.2·1.6 통과 2/2 · 100 mm: 전 속도 실패 |
| 최적 속도 | **1.2** (실측 0.553~0.555 m/s, 이탈 0.22~0.39 m) |

**지상 관측 이상치 제거** (모듈 D)

지상 로봇의 고도 오차는 **사거리에 비례**한다. 실측으로 경로에서
0~10 m 구간의 `|오차|>3 m` 비율이 59.8% 인 반면 80~160 m 구간은 88.7%
였고, 보행하는 leg 는 −5 m 아래 셀이 5.1%(최저 −46.4 m)까지 나왔다
(해당 지형의 정답은 2.0~6.3 m). 두 겹으로 거른다.

- `max_point_range_m: 60.0` — 그 바깥은 드론 지도가 이미 98.9% 덮는다.
- `max_z_below_sensor_m: 12.0` / `max_z_above_sensor_m: 60.0`

**평가 지표**

자유 셀을 로봇 반지름(wheel 0.55 m, leg 0.40 m) 원판으로 침식한 뒤의
**최대 연결덩어리 비율**(`eval_connectivity.py`). 자유 공간이 많아도
잘게 쪼개져 있으면 목표에 갈 수 없으므로 셀 수만으로는 부족하다.
FN 은 순위 매기기에만 쓰고 크기 재기에는 쓰지 않는다.

---

### 8.2 종단 검증 — A→E 단일 launch 통과 (2026-08-21)

**드론 실비행부터 병합까지 전 구간을, 인자 없는 단일 launch 로, 사람 개입
없이 통과시켰다.**

```bash
ros2 launch agconav_bringup agconav_all.launch.py \
    use_nav2:=true headless:=true enable_drone:=true spawn_ground_early:=true
```

| 모듈 | 결과 |
|---|---|
| **A** 드론 스캔 | 320 웨이포인트 / **92.9 km** 완주, x 도달 −289.0 ~ 289.0 m, 선 이탈 **0.00~0.06 m**, 소요 320분 |
| **F** 주행성 분리 | wheel 통과 24,106,640 / leg 통과 24,615,099 셀 |
| **B** 위치추정 | EKF + navsat 정상 |
| **C** Nav2 | **wheel `Goal succeeded`** (494 s) · **leg `Goal succeeded`** (308 s) |
| **D** 지상 누적 | wheel 1,280,385 셀 (2470×2583) · leg 1,153,068 셀 (2555×2452) |
| **E** 병합 | 5786×4855 셀 (578.6 × 485.5 m), 108 MB mcap |

전체 소요 **331분**. 실패 신호 0건 — `Goal ABORTED` 0, `0 poses` 0,
**경로계획 실패 0**, 예외 0, 프로세스 사망 0.
자원은 전 구간 안정적이었다(RAM 여분 24 GB 유지, 스왑 0,
`drone_elevation_mapper` 0.43 GB 고정).

검증 도구: `test/scripts/verify_e2e_full.py`

#### 이 검증에서 발견해 고친 것 — 목표 표식이 지도를 막았다

1차 실행은 **경로계획 76회 연속 실패**로 두 로봇이 출발조차 못 했다.
원인은 목표 좌표에 놓아둔 **시각용 빨간 원기둥**이었다. 충돌체가 없어도
**GPU 라이다는 visual 을 때리므로**, 드론이 84 m 상공에서 그 3 m 원기둥을
지형으로 찍어 목표를 막았다.

| 조건 | wheel 목표 반경 1 m 자유 | leg |
|---|---|---|
| 표식 있음 | 27.9% | 31.3% |
| `<visibility_flags>0x01</visibility_flags>` | 28.6% | 27.2% — **효과 없음** |
| **표식 제거 (확정)** | **100.0%** | **100.0%** |

gz-sim 의 GPU 라이다는 `visibility_flags` 를 무시한다. 표식을 월드에서
제거했다. **목표 시각화는 RViz 마커로 할 것** — 월드에 합성물을 두면
센서가 그것을 지형으로 읽어 지도가 오염된다.

#### 이상치 필터 효과 (실측 대조)

| | 최저 z | −5 m 아래 셀 | 비율 |
|---|---|---|---|
| wheel 이전 | −9.9 m | 1,392 | 0.085% |
| **wheel 이후** | **−3.7 m** | **0** | **0.000%** |
| leg 이전 | −46.4 m | 65,062 | 5.099% |
| **leg 이후** | **−12.0 m** | 27,245 | **2.363%** |

wheel 은 이상치가 완전히 사라졌고, leg 는 최저값이 −46.4 → −12.0 m,
비율이 5.10 → 2.36% 로 절반 이하가 됐다. **완전히 없어지지는 않았다** —
남은 값은 z 밴드 하한(−12 m)에 붙어 있어, 사거리 60 m 안에서도 보행 중
자세 오차가 여전히 오차를 만든다는 뜻이다. 더 줄이려면 사거리나 밴드를
더 조여야 하고, 그만큼 지상 관측 범위를 잃는다.

주행 중 실제 기각률도 컸다 — wheel 은 매 스캔 27,307개 중 **2,927개
(10.7%)** 가 60 m 초과로 걸러졌고 최대 169.9 m 짜리 반사까지 잡혔다.
leg 격자는 4008×4613 에서 **2555×2452 로 3.1배 줄었다** — 원거리 오측이
격자를 부풀리던 것이 사라졌다.

---

## 9. 설계 원칙

1. **연구가 아니라 통합.** 새 알고리즘을 만들지 않고 기존 라이브러리를 쓴다. 발표 때 "실제로 돌려봤는지"까지 보여준다.
2. **알고리즘은 하나로 통일.** wheel·leg에 같은 Nav2 설정. 로봇별 최적화 금지. **작동(목표 도착)만 되면 통과.**
3. **센서는 OS1-32로 통일.** SLAM/정렬 방식도 여기에 맞춘다.

> 코드 규약("설명 가능한 것만" · 토픽-only 결합 · 공통 규약 준수)과 Git 워크플로는 **[CONTRIBUTING.md](CONTRIBUTING.md)**.

---

## 10. 설치 · 실행 (요약)

```bash
git clone https://github.com/jjongjjongR/AG-CoNav.git && cd AG-CoNav
./scripts/setup_simulation.sh
./scripts/run_simulation.sh
```

자동 설정 스크립트가 apt 의존성, Go2/CHAMP 고정 커밋 다운로드, AG-CoNav용
Go2 패치, rosdep, 전체 빌드와 설치 검증까지 수행한다. 자세한 수동 절차와
트러블슈팅은 [simulation_guide_jongheon.md](simulation_guide_jongheon.md)를 따른다.

> `deps.repos`에 등록된 외부 저장소를 갱신하려면 `deps.repos`의 `version`뿐 아니라
> `patches/unitree_go2_ros2_jazzy.patch`도 새 upstream 기준으로 재검증해야 한다.
> 임의 갱신 금지 — 팀 전원이 같은 커밋과 같은 패치를 사용해야 한다.

---

## 11. 범위 밖 (컷 — 하지 않음)

MuJoCo · LLM 재배분 · 드론 자율비행 · 로봇별 알고리즘 최적화 · 정밀 목표 탐지(`/X/detections`) · 임무 배분 · 지상 독립 SLAM · LaserScan 변환.
(필요가 실제로 확인되기 전까지 추가하지 않는다.)
d


단일 공중 스캔 기반 플랫폼별 주행성 지도 생성과 이기종 지상 로봇 협업 항법