# AG-CoNav 토픽 계약

> 로봇 3종(drone·wheel·leg)의 센서 토픽과 모듈 간 인터페이스 정리.
> 모든 좌표계는 전역 프레임 `map` 하나. 해상도는 지도류 전부 **0.10 m/cell**.
> 시뮬 시간(`use_sim_time: true`) 사용, `/clock`은 Gazebo가 유일한 소스.

---

## 1. 센서 토픽

### drone (X3)

| 센서 | SDF 센서이름 | gz 토픽 | ROS 토픽(계약) | 프레임 |
| --- | --- | --- | --- | --- |
| 3D LiDAR | `os1_lidar` | `/drone/points/points` | `/drone/points` | `drone/os1_lidar` |
| IMU | `drone_imu_sensor` | `/drone/imu` | `/drone/imu` | `drone/base_link` |
| GPS | `navsat_sensor` | `/drone/gps` | `/drone/gps` | `drone/gps_link` |

- 사설 TF: 없음 (gz 모델, 프레임이 이미 `drone/*`)

### leg (Go2)

| 센서 | SDF 센서이름 | gz 토픽 | ROS 토픽(계약) | 사설 TF 프레임 | 전역 TF 프레임 |
| --- | --- | --- | --- | --- | --- |
| 3D LiDAR | `os1_lidar` | `/leg/points/points` | `/leg/points` | `os1_lidar` | `leg/os1_lidar` |
| IMU | `leg_imu_sensor` | `/leg/imu` | `/leg/imu` | `base_link` | `leg/base_link` |
| GPS | `navsat_sensor` | `/leg/gps` | `/leg/gps` | `gps_link` | `leg/gps_link` |

- 사설 TF 토픽: `/leg/tf`, `/leg/tf_static`
- base/odom: 사설 `base_link`·`odom`·`base_footprint`
  → 전역 `leg/base_link`·`leg/odom`·`leg/base_footprint`

### wheel (A300)

| 센서 | SDF 센서이름 | gz 토픽 | ROS 토픽(계약) | 사설 TF 프레임 | 전역 TF 프레임 |
| --- | --- | --- | --- | --- | --- |
| 3D LiDAR | `lidar3d_0` | `/wheel/sensors/lidar3d_0/scan/points` | `/wheel/points` | `lidar3d_0_sensor_link` | `wheel/lidar3d_0_sensor_link` |
| IMU | `imu_0` | `/wheel/sensors/imu_0/data` | `/wheel/imu` | `imu_0_link` | `wheel/imu_0_link` |
| GPS | `gps_0` | `/wheel/sensors/gps_0/navsat` | `/wheel/gps` | `gps_0_link` | `wheel/gps_0_link` |

- 사설 TF 토픽: `/wheel/tf`, `/wheel/tf_static` (clearpath 기본)
- base/odom: 사설 `base_link`·`base_footprint`
  → 전역 `wheel/base_link`·`wheel/base_footprint`

### Go2에 남아있는 비활성 센서

원본 모델에 있지만 AG-CoNav에서 쓰지 않음 (비활성화).

| 센서 | 센서 이름 (SDF `name`) | gz 토픽 |
| --- | --- | --- |
| IMU (원본) | `imu_sensor` | `/imu/data` |
| Velodyne LiDAR | `velodyne-VLP16` | `/velodyne_points/points` |
| Unitree LiDAR | `lidar_l1` | `/unitree_lidar/points` |
| RGB 카메라 | `rgb_camera` | `/rgb_image` |

---

## 2. 모듈 간 인터페이스

모듈은 **토픽으로만** 결합한다 (다른 패키지 내부를 직접 import하지 않음).

### 모듈 A — 드론 지도 생성 (`agconav_drone`)

| 방향 | 토픽 | 타입 | 비고 |
| --- | --- | --- | --- |
| 입력 | `/drone/points` | `sensor_msgs/PointCloud2` | 드론 LiDAR |
| 입력 | `/drone/path_status` | `std_msgs/Bool` | 경로 비행 완료 신호 |
| 출력 | `/drone/elevation_map` | `grid_map_msgs/GridMap` | layer `elevation`, 래치 |
| 출력 | `/drone/elevation_map_status` | `std_msgs/Bool` | 생성 완료 (1회) |

노드: `drone_elevation_mapper`, `elevation_map_saver`, `drone_path_player`, `drone_pose_controller`

### 모듈 B — 위치추정 (`agconav_localization`)

로봇별(`wheel`/`leg`) 네임스페이스로 각각 실행.

| 방향 | 토픽 | 타입 | 비고 |
| --- | --- | --- | --- |
| 입력 | `/{X}/imu` | `sensor_msgs/Imu` | |
| 입력 | `/{X}/gps` | `sensor_msgs/NavSatFix` | §1 센서 표의 계약 이름 |
| 입력 | `/{X}/odom` | `nav_msgs/Odometry` | EKF `odom0` |
| 출력 | `/{X}/gps/odom` | `nav_msgs/Odometry` | EKF `odom1` (navsat 변환 결과) |
| 출력 | `/{X}/odometry/filtered` | `nav_msgs/Odometry` | EKF 융합 결과 |
| 출력 | `/{X}/gps/filtered` | `sensor_msgs/NavSatFix` | |
| 출력 | `/{X}/yaw_check_status` | — | yaw 정합성 검사 |
| TF | `map` → `odom` | | EKF가 발행 (`world_frame: map`) |

**datum (map 원점의 위경도)** — `config/navsat_transform_node.yaml`

```yaml
datum: [37.54233814881853, 127.06050643805561, 0.0]
```

`Seongdong_gu.world`의 `<spherical_coordinates>`와 **반드시 같아야 한다.**
어긋나면 GPS 기반 위치가 그 차이만큼 통째로 평행이동한다(과거 사례: 동 347 m·북 124 m 오차).
월드를 바꾸면 이 값도 같이 갱신할 것.

### 모듈 C — 주행 (`agconav_navigation`)

| 방향 | 토픽 | 비고 |
| --- | --- | --- |
| 입력 | `/{X}/nav_map` | 모듈 F가 만든 주행 가능 맵 |
| 입력 | `/{X}/points` | 지면 분할용 |
| 출력 | `/{X}/points_filtered` | 지면 제거 결과 |
| 출력 | `/{X}/navigation_status` | 이동 완료 (모듈 D의 발행 트리거) |

노드: `ground_segmentation_node`, `navigation_complete_node`, `mock_publisher`

### 모듈 D — 지상 지도 누적 (`agconav_ground_mapping`)

로봇별 네임스페이스로 각각 실행. 토픽은 **상대 이름**(네임스페이스로 해결).

| 방향 | 토픽 | 타입 | 비고 |
| --- | --- | --- | --- |
| 입력 | `/{X}/points` | `sensor_msgs/PointCloud2` | |
| 입력 | `/{X}/navigation_status` | `std_msgs/Bool` | 모듈 C의 이동 완료 |
| 출력 | `/{X}/elevation_map` | `grid_map_msgs/GridMap` | 완료 신호 시 **1회만** 발행 |
| 출력 | `/{X}/elevation_map_status` | `std_msgs/Bool` | |

### 모듈 E — 지도 병합 (`agconav_map_fusion`)

시스템 전체에 1개만 실행 (절대 토픽).

| 방향 | 토픽 | 타입 | 비고 |
| --- | --- | --- | --- |
| 입력 | `/drone/elevation_map` | `grid_map_msgs/GridMap` | 모듈 A |
| 입력 | `/wheel/elevation_map` · `/leg/elevation_map` | `grid_map_msgs/GridMap` | 모듈 D |
| 입력 | `/wheel/elevation_map_status` · `/leg/elevation_map_status` | `std_msgs/Bool` | 병합 트리거 조건 |
| 내부 | `/merged/merge_trigger` | `std_msgs/Bool` | collector → merger |
| 출력 | `/merged/elevation_map` | `grid_map_msgs/GridMap` | 병합 결과 (1회) |
| 출력 | `/merged/merge_status` | `std_msgs/Bool` | |
| 출력 | `/merged/merge_error` | `std_msgs/String` | 실패 사유 |

- 병합 우선순위: **wheel > leg > drone** (유효값을 NaN으로 덮지 않음)
- `/drone/elevation_map_status`는 구독하지 않음 (드론 지도는 구조상 먼저 끝남)

### 모듈 F — 지형 주행성 분석 (`agconav_traversability`)

| 방향 | 토픽 | 타입 | 비고 |
| --- | --- | --- | --- |
| 입력 | `/drone/elevation_map` | `grid_map_msgs/GridMap` | |
| 입력 | `/drone/elevation_map_status` | `std_msgs/Bool` | 계산 트리거 |
| 내부 | `/terrain/features` | `grid_map_msgs/GridMap` | layer `slope`, `step` |
| 출력 | `/wheel/nav_map` · `/leg/nav_map` | `nav_msgs/OccupancyGrid` | frame `map`, 0.10 m/cell |
| 출력 | `/wheel/nav_map_status` · `/leg/nav_map_status` | `std_msgs/Bool` | |
| 서비스 | `/map_saver/save_map` | `nav2_msgs/SaveMap` | wheel → leg 순서 |

통과 기준: wheel `20°`/`0.08 m`, leg `30°`/`0.15 m` (낮은 장애물은 wheel 막힘 / leg 통과)

---

## 3. 제어 · 시스템 토픽

| 토픽 | 타입 | 비고 |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS, 시간의 유일한 소스 |
| `/wheel/clock` | `rosgraph_msgs/Clock` | clearpath 스택 전용 사본 |
| `/drone/cmd_vel` | `geometry_msgs/Twist` | gz `/X3/gazebo/command/twist`로 브리지 |
| `/wheel/cmd_vel` | `geometry_msgs/Twist` | twist_mux 경유 → `/wheel/platform/cmd_vel` |
| `/leg/cmd_vel` | `geometry_msgs/Twist` | CHAMP |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 전역. 로봇 프레임은 `wheel/*`·`leg/*` 접두어 |
| `/wheel/tf`, `/leg/tf` | `tf2_msgs/TFMessage` | 사설 TF (충돌 방지용 격리) |

---

## 4. QoS 규약

지도류·완료신호는 **한 번만** 발행되므로 늦게 붙은 노드도 받을 수 있어야 한다.

| 토픽 종류 | Reliability | Durability | Depth |
| --- | --- | --- | --- |
| `*/elevation_map`, `*/nav_map`, `/terrain/features` | RELIABLE | **TRANSIENT_LOCAL** | 1 |
| `*_status`, `*_trigger` | RELIABLE | **TRANSIENT_LOCAL** | 1 |
| 원본 센서 (`*/points`, `*/imu`, `*/gps`) | BEST_EFFORT | VOLATILE | 5 |

---

## 5. 해소된 불일치 기록

### GPS 토픽 이름 — `/X/gps` 로 확정

한때 센서 브리지는 `/X/gps`를, 모듈 B의 `navsat_transform_node`는 `/X/gps/fix`를
쓰고 있어 **발행자 0개** 상태였다. navsat이 GPS fix를 한 번도 못 받아 datum이
설정되지 않고 `/X/gps/odom`이 비었다.

**§1 센서 표를 기준으로 `/X/gps`로 통일했다.** 모듈 B 문서에는 `/X/gps/fix`로
적혀 있으나 센서 표가 계약의 기준이다.

- `sensor_bridge.launch.py` : drone/leg는 gz 토픽이 이미 `/X/gps`,
  wheel만 `/wheel/sensors/gps_0/navsat` → `/wheel/gps` remap
- `localization.launch.py` : `('gps/fix', 'gps')`

### datum — 월드 값으로 확정

```yaml
datum: [37.54233814881853, 127.06050643805561, 0.0]
```

`Seongdong_gu.world`의 `<spherical_coordinates>`와 동일해야 한다(실측 오차 0.043 m).
모듈 B 문서 표의 `[37.5412278, 127.0565741, 0.0]`은 성동구 월드 이전 값이라
그대로 쓰면 동 347 m·북 124 m 어긋난다.

### wheel 센서 frame_id — 별칭으로 해결

clearpath가 시스템 xacro에서 `gz_frame_id`를 `${name}_sensor_link`로 고정해
접두어를 못 붙인다(`/wheel/imu`의 frame_id = `imu_0_link`).
전역 `/tf`에 같은 위치의 별칭을 달아 조회가 되게 했다.

```
wheel/imu_0_link            -> imu_0_link
wheel/gps_0_link            -> gps_0_link
wheel/lidar3d_0_sensor_link -> lidar3d_0_sensor_link
```

이름이 로봇 간 고유해 충돌하지 않는다(leg는 `os1_lidar`·`base_link`·`gps_link`).

### 남은 것

- `bump_010`만 회색(`0.4 0.4 0.4`), 다른 방지턱은 노란색 — 의도 확인 필요.
- wheel GPS 발행 주기가 leg의 약 1/10 — 라이다처럼 통일할지 결정 필요.
