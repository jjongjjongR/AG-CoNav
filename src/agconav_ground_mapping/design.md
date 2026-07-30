# D. 지상 로봇 주변 지형 지도 누적 — 설계 문서

이 문서는 wheel(바퀴형)과 leg(4족 보행) 지상 로봇의 3D LiDAR 관측을 공통 `map` 좌표에 누적하여, 로봇별 2.5D 높이 지도를 생성·저장하는 ROS2 패키지의 설계 문서다. 이 문서에 정의된 인터페이스, 파라미터, QoS 조건을 정확히 따라서 구현해야 한다.

---

## 1. 현실 장면 작성

1. wheel 아래 장착된 LiDAR가 지면과 주변 지형을 측정하고 출력값을 제공받는다.
2. leg 아래 장착된 LiDAR가 지면과 주변 지형을 측정하고 출력값을 제공받는다.
3. 외부 위치 추정 모듈이 제공하는 각 관측 시각의 wheel 위치·자세·TF를 확인한다.
4. 외부 위치 추정 모듈이 제공하는 각 관측 시각의 leg 위치·자세·TF를 확인한다.
5. wheel의 LiDAR 관측점을 wheel의 위치·자세를 이용해 공통 `map` 좌표로 변환한다.
6. leg의 LiDAR 관측점을 leg의 위치·자세를 이용해 공통 `map` 좌표로 변환한다.
7. wheel 관측점을 x-y 격자에 배치하고 셀별 높이를 계산해 wheel 지도에 누적한다.
8. leg 관측점을 x-y 격자에 배치하고 셀별 높이를 계산해 leg 지도에 누적한다.
9. wheel 지도와 leg 지도는 서로 덮어쓰지 않고 독립적으로 유지된다.
10. 지도 누적이 끝나면 wheel 지도와 leg 지도를 각각 저장하고, 후속 모듈(지도 병합)에 제공한다.

---

## 2. 필요한 데이터

1. wheel, leg의 3D LiDAR 관측값 — LiDAR 기준 좌표의 측정점
2. 각 관측 시각의 wheel, leg의 위치, 자세, TF
3. wheel, leg 각각의 LiDAR 센서 간의 고정 좌표 관계
4. 시뮬레이션 시간
5. 지도에 대한 기본 정보(해상도, 지도 범위)
6. 지도 누적 시작 조건
7. 지도 누적 종료 조건
8. 완성된 지도와 저장 위치 — 파일의 각각 경로와 이름

### 연결

- wheel 위치·자세·TF + wheel LiDAR 관측점 → wheel 전용 map 좌표 변환 → wheel 2.5D 지도 누적 → wheel 지도 저장
- leg 위치·자세·TF + leg LiDAR 관측점 → leg 전용 map 좌표 변환 → leg 2.5D 지도 누적 → leg 지도 저장
- wheel 지도, leg 지도 → 지도 병합 모듈, RViz2, 검증 담당자

---

## 3. 변환 찾기

1. wheel/leg 위치·자세·TF → 해당 로봇의 `map → odom → base_link → os1_lidar` TF 체인 조회
2. LiDAR 원본 점군(로봇별) + TF 체인 → map 좌표계 점군 : 좌표 변환 적용
3. map 좌표계 점군 → x-y 격자 셀 배치 및 셀별 높이 계산
4. 새 측정값 → 기존 지도에 누적 (wheel과 leg는 서로 다른 지도 상태를 독립적으로 유지)
5. wheel/leg 2.5D 지도 → 지도 파일(mcap) : 저장 및 재현 가능한 형식으로 변환

---

## 4. 책임 상자 (노드 단위)

### 4-1. PointCloud 수집 책임 — `ground_pointcloud_collector`

- **입력:** Wheel LiDAR 토픽, Leg LiDAR 토픽
- **출력:** Wheel PointCloud, Leg PointCloud
- **하는 일**
  - Wheel LiDAR 토픽 구독
  - Leg LiDAR 토픽 구독
  - 최신 PointCloud 데이터 저장
  - 데이터 수신 여부 확인

### 4-2. 좌표 변환 책임 — `ground_lidar_tf_transformer`

- **입력:** PointCloud, TF, 센서 장착 관계
- **출력:** Map 좌표계 PointCloud
- **하는 일**
  - 측정 시각의 TF 조회 (10Hz)
  - LiDAR → Base Link 변환
  - Base Link → Map 변환
  - 모든 측정점을 Map 좌표계로 변환

### 4-3. 2.5D 지도 생성 책임 — `ground_elevation_mapper`

- **입력:** Map 좌표계 PointCloud(로봇별)
- **출력:** Elevation Map(로봇별)
- **하는 일**
  - PointCloud를 XY 격자로 변환 (2D 지도화)
  - 각 셀의 높이 계산
  - 높이 정보를 Elevation Map에 반영
  - 새로운 측정값 누적 → 2.5D 지도 생성

### 4-4. 지도 저장 책임 — `ground_elevation_map_saver`

- **입력:** Wheel 2.5D 지도, Leg 2.5D 지도, 외부 이동 완료 상태(Topic, `navigation_complete`)
- **출력:** 지도 파일, wheel 지도 생성 완료 상태, leg 지도 생성 완료 상태 (`elevation_map_status`)
- **하는 일**
  - 외부 이동 완료 상태(`navigation_complete`)를 구독하여 저장 트리거로 사용
  - 파일 형식으로 변환
  - 지정 경로에 저장
  - 저장 완료 여부 확인
  - 저장이 실제로 성공한 직후 `elevation_map_status`를 True로 발행해 후속 모듈과 검증 담당자가 확인 가능하게 함 (저장 실패 시에는 발행하지 않음)

> 이전에는 완료 상태 발행이 별도 노드(`ground_completion_status_publisher`)로 분리되어 있었으나, 두 노드가 같은 `navigation_complete` 입력에 서로 통신 없이 독립적으로 반응하면서 저장(디스크 I/O, 상대적으로 느림)과 상태 발행(즉시 끝남) 사이의 순서가 보장되지 않는 레이스 컨디션이 있었다. `elevation_map_status=True`가 실제 파일 저장 완료보다 먼저 나갈 수 있어, 이를 구독하는 지도 병합 모듈이 아직 쓰이지 않았거나 존재하지 않는 파일을 읽으러 갈 위험이 있었다. 이를 없애기 위해 완료 상태 발행 책임을 지도 저장 책임에 흡수했다 — 저장이 성공한 바로 그 콜백 안에서만 상태를 발행하므로, 상태 발행 시점에는 파일이 항상 디스크에 존재한다.

---

## 5. 상자 사이 연결

- PointCloud 수집 책임 → 좌표 변환 책임
- 좌표 변환 책임 → 2.5D 지도 생성 책임
- 2.5D 지도 생성 책임 → 지도 저장 책임
- 외부 내비게이션 모듈(이동 완료 상태) → 지도 저장 책임 (저장 트리거 + 완료 상태 발행)
- 2.5D 지도 생성 책임 → RViz2, 지도 병합 모듈 (실시간 소비)

### 5-1. 구현 단위 (wheel/leg 네임스페이스로 분리)

| 책임 | 구현 방식 | 노드 이름 |
| --- | --- | --- |
| PointCloud 수집 | ROS 노드, 커스텀 | `ground_pointcloud_collector` (`/wheel`, `/leg` 각각 실행) |
| 좌표 변환 | ROS 노드(tf2), 커스텀 | `ground_lidar_tf_transformer` (`/wheel`, `/leg` 각각 실행) |
| 2.5D 지도 생성 | ROS 노드, elevation_mapping 기반 | `ground_elevation_mapper` (`/wheel`, `/leg` 각각 실행) |
| 지도 저장 + 완료 상태 제공 | ROS 노드, 커스텀 | `ground_elevation_map_saver` (`/wheel`, `/leg` 각각 실행) |

추가 구성 요소:
- `ros_gz_bridge` (wheel, leg 각각)
- 네임스페이스로 로봇 구분: `/wheel`, `/leg`

### 5-2. 데이터 흐름

- **wheel 경로:** Gazebo wheel LiDAR → `ros_gz_bridge`(`/wheel`) → `ground_pointcloud_collector`(`/wheel`) → `ground_lidar_tf_transformer`(`/wheel`) → `ground_elevation_mapper`(`/wheel`) → `ground_elevation_map_saver`(`/wheel`) → 파일시스템
- **leg 경로:** Gazebo leg LiDAR → `ros_gz_bridge`(`/leg`) → `ground_pointcloud_collector`(`/leg`) → `ground_lidar_tf_transformer`(`/leg`) → `ground_elevation_mapper`(`/leg`) → `ground_elevation_map_saver`(`/leg`) → 파일시스템
- **위치·자세 정보:** 외부 위치 추정 모듈 → `ground_lidar_tf_transformer`(`/wheel`, `/leg`) : TF 제공
- **지도 소비:** `ground_elevation_mapper`(`/wheel`, `/leg`) → RViz2, 지도 병합 모듈
- **이동 완료 상태:** 외부 내비게이션 모듈 → `ground_elevation_map_saver`(`/wheel`, `/leg`) : `navigation_complete` 구독 → 저장 실행 → 저장 성공 시 같은 콜백에서 `elevation_map_status` 발행

---

## 6. ROS 인터페이스 결정

### 6-1. Gazebo LiDAR(wheel/leg) → 브릿지

| 항목 | 내용 |
| --- | --- |
| 출력 책임 | wheel/leg LiDAR 측정 |
| 출력 구현 | Gazebo `gpu_lidar` (wheel용, leg용 각 1개) |
| 수신 구성요소 | `ros_gz_bridge`(`/wheel`, `/leg`) |
| ROS 인터페이스 여부 | 아직 ROS 인터페이스 아님, 브릿지 설정 후 확정 |

### 6-2. 브릿지 → PointCloud 수집

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 브릿지(`ros_gz_bridge`) |
| 수신 책임 | PointCloud 수집(`ground_pointcloud_collector`) |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/wheel/points`, `/leg/points` |
| 데이터 의미 | 각 로봇의 OS1-32가 측정한 3차원 점군 |
| 타입 | `sensor_msgs/msg/PointCloud2` |

### 6-3. 공통 좌표 관계(TF) → 좌표 변환 책임

| 항목 | 내용 |
| --- | --- |
| 정보 제공 책임 | 외부 위치 추정 모듈(공통 좌표 관계) |
| 정보 사용 책임 | 좌표 변환(`ground_lidar_tf_transformer`) |
| 인터페이스 방식 | tf2 조회 |
| Target frame | `map` |
| Source frame | `wheel/os1_lidar` 또는 `leg/os1_lidar` |
| 조회 시각 | `/wheel(leg)/points.header.stamp` |
| 반환 타입 | `geometry_msgs/msg/TransformStamped` |

### 6-4. 좌표 변환 책임 → 2.5D 지도 생성 책임

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 좌표 변환(`ground_lidar_tf_transformer`) |
| 수신 책임 | 2.5D 지도 생성(`ground_elevation_mapper`) |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/wheel/points_map`, `/leg/points_map` |
| 데이터 의미 | map 좌표계로 변환된 로봇별 점군 |
| 타입 | `sensor_msgs/msg/PointCloud2` (frame_id만 `map`으로 변경) |

### 6-5. 2.5D 지도 생성 책임 → 지도 저장·RViz·지도 병합 모듈

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 2.5D 지도 생성(`ground_elevation_mapper`) |
| 수신 책임 | 지도 저장, RViz2, 지도 병합 모듈 |
| 수신 구현 | `ground_elevation_map_saver`, RViz2 |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/wheel/elevation_map`, `/leg/elevation_map` |
| 데이터 의미 | 로봇별 점군을 map 좌표에 누적한 2.5D 높이 지도 |
| 타입 | `grid_map_msgs/msg/GridMap` |
| 필수 레이어 | `elevation` |

### 6-6. 외부 내비게이션 모듈(이동 완료 상태) → 지도 저장 책임

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 외부 내비게이션 모듈 (경로 계획 및 이동) |
| 수신 책임 | 지도 저장 |
| 수신 구현 | `ground_elevation_map_saver`가 `navigation_complete`를 직접 구독, 완료 신호 수신 시 저장 실행 |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/wheel/navigation_complete`, `/leg/navigation_complete` (외부 모듈과 확정 필요) |
| 데이터 의미 | wheel/leg의 이동(임무)이 끝났는지 여부 |
| 타입 | 확정 필요 (예: `std_msgs/msg/Bool`) — 구현 시 우선 `std_msgs/msg/Bool`로 가정 |
| 저장 실행 방식 | 서비스 호출이 아니라, 완료 토픽 콜백에서 내부적으로 저장 함수 호출 |
| 보조 인터페이스 | 디버깅/재현용 수동 저장 Service(`std_srvs/srv/Trigger`)를 함께 구현 (기본 비활성화) |

### 6-7. 지도 저장 책임 → 지도 병합 모듈, 검증 담당자

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 지도 저장(`ground_elevation_map_saver`) |
| 수신 책임 | 지도 병합 모듈, 검증 담당자 |
| 발행 시점 | 6-6의 저장이 실제로 성공한 직후, 같은 완료 콜백 안에서 발행 (저장 실패 시 발행하지 않음) |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/wheel/elevation_map_status`, `/leg/elevation_map_status` |
| 데이터 의미 | 로봇별 지도 누적 및 저장 완료 여부 |
| 타입 | `std_msgs/msg/Bool` |

과거에는 이 발행을 `navigation_complete`를 독립적으로 구독하는 별도 노드(`ground_completion_status_publisher`)가 담당했다. 저장(디스크 I/O)과 상태 발행(즉시 완료) 사이에 순서 보장이 없어 상태가 저장보다 먼저 나갈 수 있는 레이스 컨디션이 있었기 때문에, 이 책임을 지도 저장 책임(`ground_elevation_map_saver`)에 흡수했다.

---

## 7. 좌표, 시간, 통신 조건

### 7-1. 브릿지(`ros_gz_bridge`) → PointCloud 수집(`ground_pointcloud_collector`)

`/wheel/points`, `/leg/points`

| 조건 | 결정 |
| --- | --- |
| 타입 | `sensor_msgs/msg/PointCloud2` |
| frame_id | `wheel/os1_lidar` / `leg/os1_lidar` |
| stamp | LiDAR가 점군을 측정한 시뮬레이션 시각 |
| 점 좌표 단위 | m |
| 센서 주기 | 기본 10Hz |
| Reliability | best effort |
| Durability | volatile |
| History | keep last |
| Depth | 5 |

### 7-2. 공통 좌표 관계(TF) → 좌표 변환(`ground_lidar_tf_transformer`)

| 조건 | 결정 |
| --- | --- |
| Target frame | `map` |
| Source frame | `wheel/os1_lidar` 또는 `leg/os1_lidar` |
| 조회 시각 | `/wheel(leg)/points.header.stamp` |
| 결과 타입 | `TransformStamped` |
| 실패 조건 | 해당 시각의 TF가 존재하지 않음 |
| 실패 처리 | 해당 점군 건너뛰기 및 경고 로그 |

필요한 최종 TF 사슬 (로봇별로 동일 구조, 이름만 구분):

```
map
├─ wheel/odom
│   └─ wheel/base_link
│       └─ wheel/os1_lidar
└─ leg/odom
    └─ leg/base_link
        └─ leg/os1_lidar
```

### 7-3. 좌표 변환(`ground_lidar_tf_transformer`) → 2.5D 지도 생성(`ground_elevation_mapper`)

`/wheel/points_map`, `/leg/points_map`

| 조건 | 결정 |
| --- | --- |
| 타입 | `sensor_msgs/msg/PointCloud2` |
| frame_id | `map` |
| stamp | 변환 전 점군의 원본 시각 유지 |
| 점 좌표 단위 | m |
| Reliability | best effort |
| Durability | volatile |
| History | keep last |
| Depth | 5 |

### 7-4. 2.5D 지도 생성(`ground_elevation_mapper`) → 지도 저장/RViz2/지도 병합 모듈

`/wheel/elevation_map`, `/leg/elevation_map`

| 조건 | 결정 |
| --- | --- |
| 타입 | `grid_map_msgs/msg/GridMap` |
| frame_id | `map` |
| stamp | 지도가 마지막으로 갱신된 시뮬레이션 시각 |
| 필수 레이어 | `elevation` |
| 높이 단위 | m |
| 해상도 | 기본 0.10m/cell |
| 지도 범위 | 각 로봇의 실제 관측 영역 기준 (관측 영역만 동적으로 확장하는 방식) |
| 발행 주기 | 기본 1Hz |
| Reliability | reliable |
| Durability | transient local |
| History | keep last |
| Depth | 1 |

### 7-5. 외부 내비게이션 모듈(이동 완료 상태) → 지도 저장

`/wheel/navigation_complete`, `/leg/navigation_complete`

| 조건 | 결정 |
| --- | --- |
| 타입 | `std_msgs/msg/Bool` (확정 전제) |
| 트리거 소스 | 외부 내비게이션 모듈의 wheel/leg 이동 완료 상태 |
| 구독 시점 | wheel/leg 각각의 이동 완료 상태 수신 시 |
| 저장 대상 | 마지막으로 받은 `/wheel(leg)/elevation_map` |
| 저장 형식 | rosbag2 `mcap` |
| 로봇 구분 | wheel/leg는 별도 토픽으로 구분 (`/wheel/navigation_complete`, `/leg/navigation_complete`) |
| 누적 지속 여부 | 완료 신호 수신 시 해당 로봇의 지도 저장을 트리거 (누적 자체는 계속 가능하도록 구현, 저장은 완료 시점 기준) |
| Reliability | reliable (외부 모듈 발행 QoS 확정 시 재조정) |
| Durability | transient local (외부 모듈 발행 QoS 확정 시 재조정) |

지도 저장 Parameter (로봇별 각각 설정):

| Parameter | 의미 | 기본값 |
| --- | --- | --- |
| `input_topic` | 저장할 지도 토픽 | `/wheel/elevation_map` 또는 `/leg/elevation_map` |
| `completion_topic` | 이동 완료 상태를 구독할 토픽 | `/wheel/navigation_complete` 또는 `/leg/navigation_complete` |
| `completion_type` | 완료 상태 메시지 타입 | `std_msgs/msg/Bool` |
| `output_directory` | 저장 폴더 | 프로젝트 `maps/` |
| `map_name` | 파일 이름 | `wheel_elevation_map` / `leg_elevation_map` |
| `output_format` | 저장 형식 | `mcap` |
| `use_sim_time` | Gazebo 시간 사용 | `true` |
| `enable_manual_save_service` | 디버깅용 수동 저장 서비스 활성화 여부 | `false` |
| `status_topic` | 저장 성공 시 발행할 완료 상태 토픽 | `elevation_map_status` |

### 7-6. 지도 저장(`ground_elevation_map_saver`) → 지도 병합 모듈, 검증 담당자

`/wheel/elevation_map_status`, `/leg/elevation_map_status`

| 조건 | 결정 |
| --- | --- |
| 타입 | `std_msgs/msg/Bool` |
| 발행 시점 | 6-6의 저장이 실제로 성공한 직후, 저장을 트리거한 것과 같은 완료 콜백 안에서 (저장 실패 시 발행하지 않음) |
| Reliability | reliable |
| Durability | transient local |
| History | keep last |
| Depth | 1 |
| Parameter | `status_topic`(기본 `elevation_map_status`) — 로봇별 각각 설정. `completion_topic`(기본 `navigation_complete`)은 7-5와 공유 |

---

## 8. 실행 묶음 (launch 구조)

```
ground_elevation_mapping.launch.py
 │
 ├─ wheel 브릿지(ros_gz_bridge)
 │   └─ wheel_bridge.yaml
 │
 ├─ wheel PointCloud 수집(ground_pointcloud_collector)
 │   └─ wheel_pointcloud_collector.yaml
 │
 ├─ wheel 좌표 변환(ground_lidar_tf_transformer)
 │   └─ wheel_tf_transformer.yaml
 │
 ├─ wheel 지도 생성(ground_elevation_mapper)
 │   └─ wheel_elevation_mapper.yaml
 │
 ├─ wheel 지도 저장 + 완료 상태 제공(ground_elevation_map_saver)
 │   └─ wheel_elevation_map_saver.yaml
 │
 ├─ leg 브릿지(ros_gz_bridge)
 │   └─ leg_bridge.yaml
 │
 ├─ leg PointCloud 수집(ground_pointcloud_collector)
 │   └─ leg_pointcloud_collector.yaml
 │
 ├─ leg 좌표 변환(ground_lidar_tf_transformer)
 │   └─ leg_tf_transformer.yaml
 │
 ├─ leg 지도 생성(ground_elevation_mapper)
 │   └─ leg_elevation_mapper.yaml
 │
 └─ leg 지도 저장 + 완료 상태 제공(ground_elevation_map_saver)
     └─ leg_elevation_map_saver.yaml
```

---

## 구현 시 참고사항

- 패키지 이름: `agconav_ground_mapping`
- ROS2 배포판: Jazzy
- 언어: Python
- wheel과 leg는 반드시 같은 노드 구조(코드)를 네임스페이스(`/wheel`, `/leg`)로만 구분해서 재사용할 것. 로봇별로 다른 알고리즘을 만들지 말 것.
- 지도 범위는 고정 크기가 아니라 로봇이 실제로 관측한 영역만큼 동적으로 확장하는 방식으로 구현할 것.
- TF 조회 실패 시 반드시 해당 점군을 건너뛰고 경고 로그만 남길 것 (노드가 죽으면 안 됨).
