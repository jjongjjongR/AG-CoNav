# AG-CoNav

**Aerial-Ground Cooperative Navigation — 이기종 3로봇 통합 시뮬레이션**

> 새 알고리즘 연구가 아니라, 이미 있는 라이브러리(ROS2 · Nav2 · Gazebo · SLAM)를 조합해 **드론·4륜·4족 3대가 하나의 시뮬레이션에서 함께 동작하고, 세 로봇의 맵을 하나로 통합**하는 **통합 엔지니어링 과제**. (학부 인턴 / 약 6주)
>
> 갱신: 2026-07-17. 확정/미확정을 명확히 구분함. 이전 연구형 문서 `README_leader_parts.md`(M1~M4·LLM·RL)는 **폐기**.

---

## 1. 개요

미지 환경에 목표점이 흩어져 있고, 이기종 로봇 3대가 협력해 지도를 만들고 목표를 탐지한다.

- **드론(drone)** — 수동(경로를 사람이 pose로 지정), 위에서 LiDAR로 먼저 훑어 **맵을 생성**.
- **4륜(wheel, Husky)** — 개활지를 빠르게 이동하며 탐지.
- **4족(leg, Go2/CHAMP)** — 낮은 장애물 등 지상을 이동하며 탐지.

지상 로봇은 SLAM을 새로 돌리지 않는다(맵이 이미 있음). 드론이 만든 맵 위에서 **Nav2 Navigation**으로 목표까지 이동한다.

### 근본 목표 (이 두 개가 메인)

1. **3대 이기종 로봇이 하나의 시뮬레이션에서 동시 구동**된다.
2. **세 로봇의 맵을 하나로 통합**한다.

> 나머지(정밀 탐지, 최적 배분 등)는 전부 부가. 여기에 시간 쓰지 않는다.

---

## 2. 시스템 흐름

```
[수동 드론] LiDAR로 상공에서 스캔 (경로는 pose로 지정)
        │  (드론 맵 생성)  ← 2.5D SLAM (slam_toolbox + grid_map/elevation_mapping)
        ▼
   드론 맵 완성
        │  (맵 로드)
        ▼
[4륜 wheel] ─┐
[4족 leg]   ─┴─ 드론 맵 위에서 Nav2 Navigation 으로 이동 + 탐지 (SLAM 아님)
        │
        ▼
[3맵 통합]  드론 맵 + 4륜 맵 + 4족 맵 → 하나의 통합 맵  ← 최종 결과물
        │        (GPS / 공통 프레임 기준 정렬)
        ▼
   RViz2 로 통합 시각화
```

- **임무 순서는 드론 먼저 → 지상 로봇.** 단, 3대 모두 같은 시뮬·같은 맵에 함께 떠 있다.
- **센서는 3대 모두 3D LiDAR로 통일.**
- **내비게이션 알고리즘은 하나로 통일** — 4륜/4족에 같은 Nav2 설정을 그대로 적용. 4족에서 조금 어색해도 목표 도착만 하면 OK(로봇별 튜닝 안 함).

---

## 3. 확정 사항 (Fixed)

| 항목 | 확정 내용 |
| --- | --- |
| 지형 크기 | **500m × 500m**, 일반 도시형 (후보지: **코펜하겐**) |
| 지형 임포트 | 위성/DEM 기반으로 Gazebo에 임포트 (**gazebo_terrain_generator** / **BlenderGIS**) |
| 지형 제약 | **계단 없음, 숲 없음** |
| 장애물 담당 분리 | 낮은 장애물(길 막힘) → **4족(leg)** / 끊기지 않은 연속 도로 → **4륜(wheel)** |
| 드론 이동 | **수동, pose로 이동** (자율비행 없음), 비행 고도 = **환경 최대 높이 + 5m**, 센서 하향 장착 |
| 센서 | **Ouster OS1-32 (3D LiDAR)로 3대 통일** — 채널 32 / 수평 FOV 360° / 수직 FOV 42.4°(±21.2°) / 사거리 0.5~170m / 회전율 10·20Hz (§3-1 스펙 참조) |
| SLAM / 맵 | **2.5D** (OS1-32 3D 점군 → 높이 격자). pose: `slam_toolbox`/LiDAR odom + `grid_map`/`elevation_mapping` |
| GPS / 위치 정렬 | **적극 활용** (ground-truth 아님) → `robot_localization`(**EKF + navsat_transform**)로 좌표·맵 정렬 |
| OS/미들웨어 | Ubuntu 24.04 + **ROS2 Jazzy** + **Gazebo Harmonic** |
| 내비게이션 | **Nav2** (지상로봇 공통, `cmd_vel` 인터페이스) |
| 지상로봇 지도 | SLAM 아님 → **기존 맵 위 Navigation** |
| 로봇 네임스페이스 | **`/drone`, `/wheel`, `/leg`** |
| Python 환경 | **가상환경 미사용** (apt로 시스템 설치, Python 3.12) |
| 제외(컷) | MuJoCo/4족 RL, LLM 재배분, 드론 자율비행, 로봇별 알고리즘 최적화 (개인 확장으로만) |

### 3-1. Ouster OS1-32 사양 (Rev7 데이터시트)

| 항목 | 값 |
| --- | --- |
| 채널(수직 라인) | 32 |
| 수평 FOV | 360° |
| 수직 FOV | 42.4° (±21.2°) |
| 사거리 | 최소 0.5m / 90m @10% 반사율 / 170m @80% 반사율 (1024@10Hz) |
| 수평 해상도 | 512 / 1024 / 2048 (설정) |
| 회전율 | 10 또는 20 Hz |
| 각 샘플링 정확도 | ±0.01° (수직·수평) |
| 거리 정밀도 | ±0.5 ~ 3 cm |
| 거리 분해능 | 0.8 cm |
| 파장 | 865 nm |
| 리턴 수 | 최대 2점 |
| 출력(계산) | 약 33만 pts/s (1024@10Hz) ~ 최대 130만+ pts/s (2048@20Hz) |

> 시뮬레이션에서는 Gazebo `gpu_lidar`에 위 값(32채널·수직 42.4°·360°·회전율·사거리)을 그대로 넣어 흉내 낸다.

---

## 4. 미확정 사항 (Undecided) — 담당 · 옵션

> 아직 **확정 아님**. 결정되면 3장으로 이동.
>
> 2026-07-17: 지형(4-1) · LiDAR 모델(4-2) · SLAM 방식(4-3)은 확정되어 3장으로 이동. 4-4는 투영 방식(3D→높이격자)만 해소됨.

### 4-4. 드론 지도화 흐름 — 담당: 홍연주
- 투영 방식은 **높이 격자(2.5D)로 확정**(3장 SLAM/맵 참조).
- 드론 → 지상(알아서) → 출동

---

## 5. 기술 스택

**확정**

| 구분 | 사용 | 버전/비고 |
| --- | --- | --- |
| OS | Ubuntu | 24.04 LTS (Noble) |
| 미들웨어 | ROS2 | **Jazzy** (LTS ~2029) |
| 시뮬레이터 | Gazebo | **Harmonic** (LTS ~2028) |
| 브리지 | `ros_gz` | Gazebo ↔ ROS2 토픽 연결 |
| 내비게이션 | `Nav2` | 지상로봇 이동(공통 설정) |
| 위치추정 | `Nav2 AMCL` | 기존 맵 위 로컬라이즈 |
| 좌표/GPS 융합 | `robot_localization` | **EKF + navsat_transform**, 맵 정렬용 |
| SLAM(드론, 2.5D) | `slam_toolbox`(pose) + `grid_map`(apt)/`elevation_mapping`(소스 빌드) | OS1-32 3D 점군 → 높이 격자 |
| LiDAR | Ouster OS1-32 | 3대 공통, 32채널·수직 42.4°·360° |
| 시각화 | `RViz2` | 통합 뷰 |
| 로깅 | `rosbag2` | 재현/디버깅 |

**검토 중**

- `pointcloud_to_laserscan` — 2.5D(높이 격자) 채택으로 필요성 낮음, costmap 변환 방식 확정 시 재검토
- 맵 병합 — `multirobot_map_merge`는 **Jazzy 공식 지원 없음** → GPS/공통 프레임 정렬 후 occupancy grid를 겹치는 **커스텀 노드**로 처리

**로봇 모델**(소스 빌드): 드론 = Gazebo 기본 멀티콥터 + 하향 Ouster OS1-32 (비행고도 = 환경 최대 높이 + 5m) / 4륜 = Clearpath Husky A300 + Ouster OS1-32 / 4족 = Unitree Go2 + CHAMP + Ouster OS1-32.

**언어**: Python(rclpy) 중심. venv 미사용, **시스템 Python 3.12 + apt(`ros-jazzy-*`)**.

---

## 6. 폴더 구조

ROS2 워크스페이스 관례(`src/` 아래 패키지들).

```
AG-CoNav/
├── README.md
└── src/
    ├── agconav_bringup/        # 전체 시스템 통합 launch (원클릭 구동)
    ├── agconav_worlds/         # Gazebo Harmonic 월드, 지형, 시나리오
    ├── agconav_description/    # 로봇 3종 모델(URDF/xacro/SDF) + LiDAR 센서
    ├── agconav_gz_bridge/      # ros_gz_bridge 설정 (+ 필요 시 pointcloud_to_laserscan)
    ├── agconav_drone/          # 드론 수동 경로 재생 + 맵 생성
    ├── agconav_navigation/     # Nav2 공통 params + 지상로봇 bringup + map_server/AMCL
    └── agconav_map_merge/      # 3맵 통합 커스텀 노드 (메인 결과물)
```

> `maps/`, `config/`, `docs/` 는 필요해질 때 추가. 지금은 뼈대만.

---

## 7. 개발 지침 (팀 규칙)

1. **연구가 아니라 통합.** 새 알고리즘을 만들지 않고 **기존 라이브러리**를 찾아 쓴다. 발표 때 "실제로 돌려봤는지"까지 보여준다.
2. **알고리즘은 하나로 통일.** 4륜/4족에 같은 Nav2 설정. 로봇별 최적화 금지. **작동(목표 도착)만 되면 통과.**
3. **센서는 LiDAR로 통일.** SLAM/맵 정렬 방식도 여기에 맞춘다.
4. **설명 가능한 것만 넣는다.** 좌표 변환·용어·라이브러리·툴 전부 스스로 설명 가능해야. (AI 추천만 보고 넣지 않기. 연동·통합 방법은 도움받아도 됨.)
5. **네임스페이스로 로봇 분리** — `/drone`, `/wheel`, `/leg`. tf 트리는 `map → odom → base_link`.
6. **시뮬 시간 사용** — 모든 노드 `use_sim_time:=true`, `/clock` 동기화.
7. **Git** — 모듈별 브랜치 + PR. 커밋 단위 작게.

### 추천 토픽/노드 (초안 — 미확정)

| 로봇별(`/wheel` 예시) | 전역(공유) |
| --- | --- |
| `/wheel/points` (PointCloud2) | `/map` (드론 생성 맵) |
| `/wheel/scan` (LaserScan, 2D 경로 시) | `/merged_map` (통합 맵) |
| `/wheel/odom` | `/tf`, `/tf_static` |
| `/wheel/cmd_vel` | `/clock` |
| `/wheel/detections` | `/drone/cmd_pose` (수동 경로) |

주요 노드: `gz_sim` / `ros_gz_bridge` / `drone_path_player` / `slam_toolbox`+`elevation_mapping` / `wheel_nav2`·`leg_nav2` / `map_merge_node` / `mission_coordinator`

---

## 8. R&R

| 담당 | 역할 |
| --- | --- |
| **이종헌**(팀장) | **시뮬레이션 총괄**, 노드/토픽 설계 |
| **채현우** | 4족 보행 로봇 |
| **이수빈** | 4륜 모바일 로봇 |
| **홍연주** | 드론 |

---

## 9. 설치 (요약)

```bash
# ROS2 Jazzy + 도구 (venv 안 씀, 시스템에 설치)
sudo apt install ros-jazzy-desktop gz-harmonic ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  python3-numpy python3-scipy python3-matplotlib python3-opencv \
  ros-jazzy-slam-toolbox ros-jazzy-grid-map
# elevation_mapping은 Jazzy apt 패키지 없음 → 소스 빌드

cd AG-CoNav && colcon build --symlink-install && source install/setup.bash
```

로봇 모델(Husky A300 / Go2·CHAMP), `elevation_mapping`, 맵 병합 노드는 소스 빌드 필요.

---

## 10. 다음 결정 대기 (Open)

1. **드론 지도화 흐름**: (a) 드론→드론(디테일)→지상(보완)→출동 vs (b) 드론→지상(알아서)→출동 — 4-4
