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

1. wheel/leg 위치·자세·TF → 해당 로봇의 `map → {robot}/odom → {robot}/base_link → {robot}의 LiDAR 프레임` TF 체인 조회 (변경사항 5: 로봇마다 실제 장착 LiDAR 기종이 달라 프레임 이름이 다르다 — wheel: `wheel/lidar3d_0_sensor_link`, leg: `leg/os1_lidar`)
2. LiDAR 원본 점군(로봇별) + TF 체인 → map 좌표계 점군 : 좌표 변환 적용
3. map 좌표계 점군 → x-y 격자 셀 배치 및 셀별 높이 계산
4. 새 측정값 → 기존 지도에 누적 (wheel과 leg는 서로 다른 지도 상태를 독립적으로 유지)
5. wheel/leg 2.5D 지도 → 지도 파일(mcap) : 저장 및 재현 가능한 형식으로 변환

---

## 4. 책임 상자 (노드 단위)

### 4-1. 지도 생성 책임 — `ground_elevation_mapper`

- **입력:** Wheel/Leg LiDAR 원본 토픽, 공통 좌표 관계(TF), 외부 이동 완료 상태(Topic, `navigation_status`)
- **출력:** Wheel/Leg Elevation Map (이동 완료 시 1회)
- **하는 일**
  - Wheel/Leg LiDAR 토픽 직접 구독
  - 데이터 수신 여부 확인 (헬스체크)
  - 측정 시각의 TF 조회 (LiDAR → Base Link → Map, 10Hz)
  - 모든 측정점을 Map 좌표계로 변환
  - 변환된 점을 XY 격자로 배치하고, 셀별 독립 스칼라 칼만필터로 높이(x)와 분산(P)을 갱신 (배치=이번 콜백의 스캔 단위, 러닝 애버리지에서 변경 — 2.5D 지도 생성)
  - 측정 노이즈(R)는 거리·입사각·점밀도를 결합해 콜백마다 셀별로 계산 (입사각은 이전까지 누적된 높이 격자의 기울기로 근사)
  - 이미 값이 있는 셀에 한해 이노베이션 게이팅으로 이상치(튄 점) 스캔을 걸러냄
  - `navigation_status`(True) 수신 시 그 시점까지 누적된 지도(`elevation`, `elevation_variance`)를 Elevation Map으로 1회만 발행

> 이전에는 이 책임이 3개 노드(`ground_pointcloud_collector`: PointCloud 수집 + 수신 감시, `ground_lidar_tf_transformer`: 좌표 변환, `ground_elevation_mapper`: 격자 누적 + 1Hz 주기 발행)로 나뉘어 있었다. `ground_pointcloud_collector`는 점군을 저장·감시만 하고 재발행하지 않아서 `ground_lidar_tf_transformer`가 같은 원본 토픽을 별도로 다시 구독해야 했고, `ground_lidar_tf_transformer`를 독립 노드로 유지하는 것은 10Hz × 32ch × 1024점 규모의 PointCloud2를 변환 후 재발행하는 오버헤드만 추가할 뿐 그 출력을 소비하는 곳이 `ground_elevation_mapper` 하나뿐이라 이점이 없었다. 그래서 세 노드를 `ground_elevation_mapper` 하나로 합쳤다.
>
> **결정 근거 1 — 수신 감시 기능 유지**: `ground_pointcloud_collector`의 수신 감시(1초마다 마지막 수신 이후 경과 시간 체크, 타임아웃 시 경고 로그) 책임은 노드 통합과 무관하게 여전히 유용하다고 판단해 `ground_elevation_mapper` 내부에 `_last_received`/`_check_timer`로 그대로 이식했다.
>
> **결정 근거 2 — source_frame 처리 방식**: TF 조회 시 사용할 source_frame을 메시지의 `header.frame_id`를 그대로 신뢰할지, 로봇별 전역 프레임 이름으로 덮어쓸지는 `/wheel/points`, `/leg/points`의 실제 발행값을 실행 환경에서 확인해야 답할 수 있는 문제였다. 이 저장소에는 `ros2`/`gz` 실행 환경 자체가 없어 실측이 불가능했고, 정적 분석 결과도 wheel(clearpath 표준 launch가 네임스페이스로 감싸 전역 프레임일 개연성이 있음)과 leg(현재 `go2_spawn.launch.py`가 네임스페이스/프리픽스 없이 사설 프레임을 그대로 발행)가 서로 다른 결론을 가리켰다. 이에 `target_source_frame` 파라미터를 추가해, 값이 있으면 그 전역 프레임 이름으로 source_frame을 덮어쓰고 비어 있으면(기본값) 기존처럼 `msg.header.frame_id`를 신뢰하도록 했다. wheel/leg 설정에는 각각 `wheel/lidar3d_0_sensor_link`, `leg/os1_lidar`를 채워 두었으나, 이는 정적 분석에 근거한 잠정값이며 브릿지(`agconav_gz_bridge`)가 실제로 구현되고 `ros2 topic echo`로 실측된 뒤 재확인이 필요하다.
>
> **결정 근거 3 — TF 구독 범위**: `map ↔ odom`(모듈 B 담당, 전역)과 로봇 자체 TF(사설 네임스페이스 토픽일 가능성)가 서로 다른 토픽에 나뉘어 있을 수 있다는 우려가 있었으나, 이 역시 실행 환경 부재로 실측하지 못했다. 정적 분석상 wheel(clearpath 표준 launch가 `/tf`를 `/wheel/tf`로 리맵)과 leg(`go2_spawn.launch.py`가 리맵 없이 전역 `/tf` 그대로 사용)가 서로 다르게 구성되어 있었지만, 기본 `tf2_ros.TransformListener`를 그대로 사용하기로 결정했다 — 네임스페이스 안에서 자동으로 `/wheel/tf`, `/leg/tf`로 리맵된다. leg 쪽에서 `map ↔ leg/odom` 연결이 끊길 위험은 남아 있으며, 실제 Gazebo/브릿지 환경에서 재검증이 필요하다 (7-2 참고).

### 4-2. 지도 저장 책임 — `ground_elevation_map_saver`

- **입력:** Wheel/Leg Elevation Map (완료 시 1회 수신 자체가 저장 트리거)
- **출력:** 지도 파일, wheel 지도 생성 완료 상태, leg 지도 생성 완료 상태 (`elevation_map_status`)
- **하는 일**
  - `elevation_map` 구독
  - 수신 즉시 저장 실행 (별도의 완료 토픽 구독 없음)
  - 파일 형식으로 변환
  - 지정 경로에 저장
  - 저장 완료 여부 확인
  - 저장이 실제로 성공한 직후 `elevation_map_status`를 True로 발행해 후속 모듈과 검증 담당자가 확인 가능하게 함 (저장 실패 시에는 발행하지 않음)

> `elevation_map`이 이동 완료 시 1회만 발행되는 이벤트 기반으로 바뀌면서, `ground_elevation_mapper`와 `ground_elevation_map_saver`가 `navigation_status`를 각자 독립적으로 구독하면 두 콜백의 실행 순서가 보장되지 않아 `ground_elevation_map_saver`가 아직 발행되지 않은 지도를 저장하려는 레이스 컨디션이 생길 수 있었다. 이는 예전에 저장(디스크 I/O)과 완료 상태 발행 사이에서 겪었던 레이스 컨디션과 근본적으로 같은 유형의 문제다 — 그때는 저장 노드가 저장 성공 후 직접 상태를 발행하도록 트리거를 단일화해 해결했는데(아래 참고), 이번에는 반대 방향으로 트리거를 단일화했다: 발행 쪽(`ground_elevation_mapper`)이 완료 신호를 받아야만 지도를 발행하고, 저장 쪽(`ground_elevation_map_saver`)은 그 발행 자체(`elevation_map` 수신)를 저장 트리거로 삼는다. 이렇게 하면 `ground_elevation_map_saver`가 `elevation_map`을 받는 시점에는 이동이 이미 완료된 상태임이 항상 보장된다.
>
> 과거에는 완료 상태 발행이 별도 노드(`ground_completion_status_publisher`)로 분리되어 있었으나, 두 노드가 같은 완료 토픽 입력에 서로 통신 없이 독립적으로 반응하면서 저장(디스크 I/O, 상대적으로 느림)과 상태 발행(즉시 끝남) 사이의 순서가 보장되지 않는 레이스 컨디션이 있었다. `elevation_map_status=True`가 실제 파일 저장 완료보다 먼저 나갈 수 있어, 이를 구독하는 지도 병합 모듈이 아직 쓰이지 않았거나 존재하지 않는 파일을 읽으러 갈 위험이 있었다. 이를 없애기 위해 완료 상태 발행 책임을 지도 저장 책임에 흡수했다 — 저장이 성공한 바로 그 콜백 안에서만 상태를 발행하므로, 상태 발행 시점에는 파일이 항상 디스크에 존재한다.

---

## 5. 상자 사이 연결

- Wheel/Leg LiDAR 원본 토픽 + 공통 좌표 관계(TF) → 지도 생성 책임 (직접 구독 + 변환 + 누적)
- 외부 내비게이션 모듈(이동 완료 상태, `navigation_status`) → 지도 생성 책임 (완료 신호 수신 시 지도 1회 발행 트리거)
- 지도 생성 책임 → 지도 저장 책임 (`elevation_map` 발행 자체가 저장 트리거)
- 지도 생성 책임 → RViz2, 지도 병합 모듈 (실시간 소비)

### 5-1. 구현 단위 (wheel/leg 네임스페이스로 분리)

| 책임 | 구현 방식 | 노드 이름 |
| --- | --- | --- |
| 지도 생성 (수집 + 좌표 변환 + 격자 누적) | ROS 노드(tf2), 커스텀 | `ground_elevation_mapper` (`/wheel`, `/leg` 각각 실행) |
| 지도 저장 + 완료 상태 제공 | ROS 노드, 커스텀 | `ground_elevation_map_saver` (`/wheel`, `/leg` 각각 실행) |

추가 구성 요소:
- `ros_gz_bridge` (wheel, leg 각각)
- 네임스페이스로 로봇 구분: `/wheel`, `/leg`

### 5-2. 데이터 흐름

- **wheel 경로:** Gazebo wheel LiDAR → `ros_gz_bridge`(`/wheel`) → `ground_elevation_mapper`(`/wheel`) → `ground_elevation_map_saver`(`/wheel`) → 파일시스템
- **leg 경로:** Gazebo leg LiDAR → `ros_gz_bridge`(`/leg`) → `ground_elevation_mapper`(`/leg`) → `ground_elevation_map_saver`(`/leg`) → 파일시스템
- **위치·자세 정보:** 외부 위치 추정 모듈 → `ground_elevation_mapper`(`/wheel`, `/leg`) : TF 제공
- **지도 소비:** `ground_elevation_mapper`(`/wheel`, `/leg`) → RViz2, 지도 병합 모듈
- **이동 완료 상태:** 외부 내비게이션 모듈 → `ground_elevation_mapper`(`/wheel`, `/leg`) : `navigation_status` 구독 → 완료 시 그 시점까지 누적된 지도를 `elevation_map`으로 1회 발행 → `ground_elevation_map_saver`(`/wheel`, `/leg`) : `elevation_map` 수신 자체가 저장 트리거 → 저장 성공 시 같은 콜백에서 `elevation_map_status` 발행

---

## 6. ROS 인터페이스 결정

### 6-1. Gazebo LiDAR(wheel/leg) → 브릿지

| 항목 | 내용 |
| --- | --- |
| 출력 책임 | wheel/leg LiDAR 측정 |
| 출력 구현 | Gazebo `gpu_lidar` (wheel용, leg용 각 1개) |
| 수신 구성요소 | `ros_gz_bridge`(`/wheel`, `/leg`) |
| ROS 인터페이스 여부 | 아직 ROS 인터페이스 아님, 브릿지 설정 후 확정 |

### 6-2. 브릿지 → 지도 생성 책임 (직접 입력)

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 브릿지(`ros_gz_bridge`) |
| 수신 책임 | 지도 생성(`ground_elevation_mapper`) |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/wheel/points`, `/leg/points` |
| 데이터 의미 | 각 로봇의 3D LiDAR가 측정한 점군 (wheel: A300 `lidar3d_0`, leg: Go2 OS1-32) |
| 타입 | `sensor_msgs/msg/PointCloud2` |

> 이전에는 `ground_pointcloud_collector`가 이 토픽을 구독해 저장·감시만 하고, `ground_lidar_tf_transformer`가 같은 토픽을 다시 독립 구독했다. 지금은 `ground_elevation_mapper`가 직접 구독한다 (4-1 참고).

### 6-3. 공통 좌표 관계(TF) → 지도 생성 책임

| 항목 | 내용 |
| --- | --- |
| 정보 제공 책임 | 외부 위치 추정 모듈(공통 좌표 관계) |
| 정보 사용 책임 | 지도 생성(`ground_elevation_mapper`) |
| 인터페이스 방식 | tf2 조회 |
| Target frame | `map` |
| Source frame | `target_source_frame` 파라미터가 지정되어 있으면 그 값(전역 프레임), 비어 있으면 `/wheel(leg)/points.header.frame_id` (4-1 결정 근거 2 참고) |
| 조회 시각 | `/wheel(leg)/points.header.stamp` |
| 반환 타입 | `geometry_msgs/msg/TransformStamped` |

### 6-4. 지도 생성 책임 → 지도 저장/RViz2/지도 병합 모듈

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 지도 생성(`ground_elevation_mapper`) |
| 수신 책임 | 지도 저장, RViz2, 지도 병합 모듈 |
| 수신 구현 | `ground_elevation_map_saver`, RViz2 |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/wheel/elevation_map`, `/leg/elevation_map` |
| 데이터 의미 | 로봇별 점군을 map 좌표에 누적한 2.5D 높이 지도 |
| 타입 | `grid_map_msgs/msg/GridMap` |
| 필수 레이어 | `elevation` |
| 부가 레이어 | `elevation_variance` — 셀별 칼만필터 불확실성(P). `basic_layers`에는 포함하지 않음(관측 여부 판단 기준이 아니라 부가 정보) |
| 발행 시점 | `navigation_status`(True) 수신 시 그 시점까지 누적된 지도를 1회만 (구 1Hz 주기 발행에서 변경, 7-3 참고) |

### 6-5. 외부 내비게이션 모듈(이동 완료 상태) → 지도 생성 책임

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 외부 내비게이션 모듈 (모듈 C, 경로 계획 및 이동) |
| 수신 책임 | 지도 생성(`ground_elevation_mapper`) |
| 수신 구현 | `ground_elevation_mapper`가 `navigation_status`를 직접 구독, True 수신 시 누적된 지도를 `elevation_map`으로 1회 발행 |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/wheel/navigation_status`, `/leg/navigation_status` (모듈 C가 발행하는 이름. 이전 `navigation_complete`에서 변경됨) |
| 데이터 의미 | wheel/leg의 이동(임무)이 끝났는지 여부 |
| 타입 | `std_msgs/msg/Bool` |

### 6-6. 지도 생성 책임(`elevation_map`) → 지도 저장 책임 (저장 트리거)

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 지도 생성(`ground_elevation_mapper`) |
| 수신 책임 | 지도 저장(`ground_elevation_map_saver`) |
| 인터페이스 종류 | 6-4와 동일한 Topic (`/wheel/elevation_map`, `/leg/elevation_map`) — 별도 신호 없이 이 발행 자체가 저장 트리거 |
| 저장 실행 방식 | 서비스 호출이 아니라, `elevation_map` 구독 콜백에서 내부적으로 저장 함수 호출 |
| 보조 인터페이스 | 디버깅/재현용 수동 저장 Service(`std_srvs/srv/Trigger`)를 함께 구현 (기본 비활성화) |

> `ground_elevation_map_saver`는 더 이상 `navigation_status`를 직접 구독하지 않는다 — `ground_elevation_mapper`가 `navigation_status`를 받아야만 `elevation_map`을 발행하므로, `elevation_map` 수신 자체가 "이동 완료 + 최종 지도 확정"을 의미한다 (4-2 참고).

### 6-7. 지도 저장 책임 → 지도 병합 모듈, 검증 담당자

| 항목 | 내용 |
| --- | --- |
| 발행 책임 | 지도 저장(`ground_elevation_map_saver`) |
| 수신 책임 | 지도 병합 모듈, 검증 담당자 |
| 발행 시점 | 6-6의 저장이 실제로 성공한 직후, `elevation_map`을 받은 것과 같은 콜백 안에서 (저장 실패 시 발행하지 않음) |
| 인터페이스 종류 | Topic |
| 인터페이스 이름 | `/wheel/elevation_map_status`, `/leg/elevation_map_status` |
| 데이터 의미 | 로봇별 지도 누적 및 저장 완료 여부 |
| 타입 | `std_msgs/msg/Bool` |

과거에는 이 발행을 완료 토픽을 독립적으로 구독하는 별도 노드(`ground_completion_status_publisher`)가 담당했다. 저장(디스크 I/O)과 상태 발행(즉시 완료) 사이에 순서 보장이 없어 상태가 저장보다 먼저 나갈 수 있는 레이스 컨디션이 있었기 때문에, 이 책임을 지도 저장 책임(`ground_elevation_map_saver`)에 흡수했다.

---

## 7. 좌표, 시간, 통신 조건

### 7-1. 브릿지(`ros_gz_bridge`) → 지도 생성(`ground_elevation_mapper`)

`/wheel/points`, `/leg/points`

| 조건 | 결정 |
| --- | --- |
| 타입 | `sensor_msgs/msg/PointCloud2` |
| frame_id | 실측 필요 (4-1 결정 근거 2 참고) — wheel: `lidar3d_0_sensor_link`(사설) 또는 `wheel/lidar3d_0_sensor_link`(전역), leg: `os1_lidar`(사설) 또는 `leg/os1_lidar`(전역). 이 저장소에는 실행 환경이 없어 확정하지 못했고, `target_source_frame` 파라미터로 우회함 |
| stamp | LiDAR가 점군을 측정한 시뮬레이션 시각 |
| 점 좌표 단위 | m |
| 센서 주기 | 기본 10Hz |
| Reliability | best effort |
| Durability | volatile |
| History | keep last |
| Depth | 5 |

### 7-2. 공통 좌표 관계(TF) → 지도 생성(`ground_elevation_mapper`)

| 조건 | 결정 |
| --- | --- |
| Target frame | `map` |
| Source frame | `target_source_frame` 파라미터(전역 프레임) 우선, 비어 있으면 점군의 `header.frame_id` |
| 조회 시각 | `/wheel(leg)/points.header.stamp` |
| 결과 타입 | `TransformStamped` |
| 실패 조건 | 해당 시각의 TF가 존재하지 않음 |
| 실패 처리 | 해당 점군 건너뛰기 및 경고 로그 |
| TF 구독 범위 | 기본 `tf2_ros.TransformListener` 그대로 사용 (네임스페이스 안에서 `/tf`, `/tf_static`이 자동으로 `/wheel/tf`, `/leg/tf`로 리맵됨). wheel의 clearpath 표준 launch는 TF를 `/wheel/tf`로 리맵해 이 방식과 맞지만, leg(`go2_spawn.launch.py`)는 현재 리맵 없이 전역 `/tf`로 발행하고 있어 `map ↔ leg/odom` 구간(모듈 B 담당)이 안 이어질 위험이 있다. 실행 환경 부재로 실측하지 못한 채 사용자 결정에 따라 기본 리스너로 진행했으며, 실제 Gazebo/브릿지 환경에서 `ros2 topic list`, `tf2_echo`로 재확인이 필요하다 (4-1 결정 근거 3 참고). |

필요한 최종 TF 사슬 (로봇별로 동일 구조, 프레임 이름만 구분):

```
map
├─ wheel/odom
│   └─ wheel/base_link
│       └─ wheel/lidar3d_0_sensor_link
└─ leg/odom
    └─ leg/base_link
        └─ leg/os1_lidar
```

### 7-3. 지도 생성(`ground_elevation_mapper`) → 지도 저장/RViz2/지도 병합 모듈

`/wheel/elevation_map`, `/leg/elevation_map`

| 조건 | 결정 |
| --- | --- |
| 타입 | `grid_map_msgs/msg/GridMap` |
| frame_id | `map` |
| stamp | 지도가 마지막으로 갱신된 시뮬레이션 시각 |
| 필수 레이어 | `elevation` |
| 부가 레이어 | `elevation_variance` (셀별 칼만필터 분산 P, `basic_layers`에는 미포함) |
| 높이 단위 | m |
| 해상도 | 기본 0.10m/cell |
| 지도 범위 | 각 로봇의 실제 관측 영역 기준 (관측 영역만 동적으로 확장하는 방식) |
| 발행 시점 | `navigation_status`(True) 수신 시 1회만 (구 1Hz 주기 발행에서 변경) |
| Reliability | reliable |
| Durability | transient local |
| History | keep last |
| Depth | 1 |

> QoS를 reliable + transient_local + keep_last + depth 1로 확정한 이유: 1회성 발행으로 바뀐 뒤에는 이 한 번의 발행이 유실되면 복구 수단이 없기 때문 (best_effort였다면 구독자가 그 순간 연결되어 있지 않으면 영영 못 받는다). 모듈 E의 `merge_trigger`와 동일한 이유.

### 7-4. 외부 내비게이션 모듈(이동 완료 상태) → 지도 생성(`ground_elevation_mapper`)

`/wheel/navigation_status`, `/leg/navigation_status`

| 조건 | 결정 |
| --- | --- |
| 타입 | `std_msgs/msg/Bool` |
| 트리거 소스 | 외부 내비게이션 모듈(모듈 C)의 wheel/leg 이동 완료 상태 |
| 구독 시점 | wheel/leg 각각의 이동 완료 상태 수신 시 |
| 발행 대상 | 그 시점까지 누적된 `/wheel(leg)/elevation_map` — True 수신 후 1회만, 중복 발행 방지(`_published` 플래그) |
| 로봇 구분 | wheel/leg는 별도 토픽으로 구분 (`/wheel/navigation_status`, `/leg/navigation_status`) |
| Reliability | reliable |
| Durability | transient local |
| History | keep last |
| Depth | 1 |

> 토픽 이름이 `navigation_complete`에서 `navigation_status`로 변경됨 (모듈 C의 실제 발행 토픽 이름 반영). QoS를 reliable + transient_local로 확정한 이유는 7-3과 동일 — 1회성 신호라 유실 시 복구 수단이 없음.

지도 생성 Parameter (로봇별 각각 설정):

| Parameter | 의미 | 기본값 |
| --- | --- | --- |
| `points_topic` | 원본 점군 구독 토픽 | `points` |
| `elevation_map_topic` | 지도 발행 토픽 | `elevation_map` |
| `navigation_status_topic` | 이동 완료 상태 구독 토픽 | `navigation_status` |
| `target_frame` | TF 조회 대상 프레임 | `map` |
| `target_source_frame` | TF 조회 source frame 강제 지정(전역 프레임). 비우면 점군의 `header.frame_id` 사용 | wheel: `wheel/lidar3d_0_sensor_link`, leg: `leg/os1_lidar` |
| `resolution` | 격자 해상도 | `0.10` (m/cell) |
| `frame_id` | 발행할 GridMap의 frame_id | `map` |
| `data_timeout_sec` | 수신 감시 타임아웃 | `2.0` |
| `check_period_sec` | 수신 감시 체크 주기 | `1.0` |
| `measurement_noise_base` | 칼만필터 측정 노이즈(R) 기본값 (R0) | wheel/leg 각각 `0.0004` (2cm 표준편차 가정 placeholder — 실측 필요, 아래 참고) |
| `distance_noise_coefficient` | 거리에 따른 R 증가 계수 (k) | wheel/leg 각각 `0.01` (placeholder — 실측 필요) |
| `incidence_cos_floor` | 입사각 cos 하한 (grazing angle에서 R 발산 방지) | `0.17` (cos 80°) |
| `innovation_gate_threshold` | 이노베이션 게이팅 임계값 (카이제곱, 자유도 1, ~3-시그마) | `9.0` |
| `use_sim_time` | Gazebo 시간 사용 | `true` |

> `measurement_noise_base`/`distance_noise_coefficient`는 wheel(A300 `lidar3d_0`)과 leg(Go2 OS1-32)가 서로 다른 LiDAR 기종이라 실측 정확도 스펙도 다를 수 있어, `wheel_elevation_mapper.yaml`/`leg_elevation_mapper.yaml`에 로봇별로 독립적으로 채운다. 이 문서 작성 시점에는 팀이 두 LiDAR의 데이터시트/실측 정확도 스펙을 아직 확정하지 않아 두 값 모두 placeholder다 — 재확인 필요.

### 7-5. 지도 저장(`ground_elevation_map_saver`) → 지도 병합 모듈, 검증 담당자

`/wheel/elevation_map_status`, `/leg/elevation_map_status`

| 조건 | 결정 |
| --- | --- |
| 타입 | `std_msgs/msg/Bool` |
| 발행 시점 | `elevation_map` 수신 후 저장이 실제로 성공한 직후, 같은 콜백 안에서 (저장 실패 시 발행하지 않음) |
| Reliability | reliable |
| Durability | transient local |
| History | keep last |
| Depth | 1 |

지도 저장 Parameter (로봇별 각각 설정):

| Parameter | 의미 | 기본값 |
| --- | --- | --- |
| `input_topic` | 저장할 지도 토픽(저장 트리거 겸용) | `elevation_map` |
| `output_directory` | 저장 폴더 | 프로젝트 `maps/` |
| `map_name` | 파일 이름 | `wheel_elevation_map` / `leg_elevation_map` |
| `output_format` | 저장 형식 | `mcap` |
| `use_sim_time` | Gazebo 시간 사용 | `true` |
| `enable_manual_save_service` | 디버깅용 수동 저장 서비스 활성화 여부 | `false` |
| `status_topic` | 저장 성공 시 발행할 완료 상태 토픽 | `elevation_map_status` |

> `completion_topic`/`completion_type` 파라미터는 제거됨 — `ground_elevation_map_saver`가 `navigation_status`를 더 이상 직접 구독하지 않기 때문 (6-6 참고).

---

## 8. 실행 묶음 (launch 구조)

```
ground_elevation_mapping.launch.py
 │
 ├─ wheel 지도 생성(ground_elevation_mapper)
 │   └─ wheel_elevation_mapper.yaml
 │
 ├─ wheel 지도 저장 + 완료 상태 제공(ground_elevation_map_saver)
 │   └─ wheel_elevation_map_saver.yaml
 │
 ├─ leg 지도 생성(ground_elevation_mapper)
 │   └─ leg_elevation_mapper.yaml
 │
 └─ leg 지도 저장 + 완료 상태 제공(ground_elevation_map_saver)
     └─ leg_elevation_map_saver.yaml
```

`ros_gz_bridge`(wheel/leg, `wheel_bridge.yaml`/`leg_bridge.yaml`)는 `agconav_gz_bridge` 패키지가 소유하며 이 launch 파일에서는 의도적으로 띄우지 않는다 — `ground_elevation_mapping.launch.py`는 이 패키지가 소유한 노드만 다룬다.

로봇당 노드 수가 4개(수집/변환/생성/저장)에서 2개(생성/저장)로 줄었다 (4-1 참고).

---

## 구현 시 참고사항

- 패키지 이름: `agconav_ground_mapping`
- ROS2 배포판: Jazzy
- 언어: Python
- wheel과 leg는 반드시 같은 노드 구조(코드)를 네임스페이스(`/wheel`, `/leg`)로만 구분해서 재사용할 것. 로봇별로 다른 알고리즘을 만들지 말 것.
- 지도 범위는 고정 크기가 아니라 로봇이 실제로 관측한 영역만큼 동적으로 확장하는 방식으로 구현할 것.
- TF 조회 실패 시 반드시 해당 점군을 건너뛰고 경고 로그만 남길 것 (노드가 죽으면 안 됨).
- `wheel_bridge.yaml`/`leg_bridge.yaml`(`agconav_gz_bridge`)이 아직 저장소에 구현되어 있지 않아, `/wheel/points`·`/leg/points`의 실제 `header.frame_id`와 TF 발행 방식(전역 `/tf` vs 사설 `/wheel(leg)/tf`)을 이 문서 작성 시점에는 실측하지 못했다. `target_source_frame` 파라미터(4-1, 7-1, 7-2)와 기본 `TransformListener` 선택은 그 불확실성 위에서 내린 잠정 결정이며, 브릿지 구현 후 `ros2 topic echo`/`tf2_echo`로 재검증이 필요하다.
- drone(X3)은 이 패키지(`agconav_ground_mapping`)의 범위 밖이다 — 지상 로봇(wheel/leg)만 다룬다.

---

## 변경 이력

> **셀별 칼만필터 도입, `elevation_variance` 레이어 추가 (인터페이스 변경)**: `ground_elevation_mapper`의 셀별 높이 누적 방식을 러닝 애버리지(`self._sum`/`self._count`)에서 셀당 독립 스칼라 칼만필터(`self._elevation`/`self._variance`)로 바꿨다. 상태는 셀당 높이 하나, 셀 간 상관관계는 없다고 가정(완전 독립), 갱신은 점 단위가 아니라 콜백(스캔) 배치 단위로 1회 수행한다. 측정 노이즈 R은 거리 제곱 + 입사각(`1/cos_theta**2`, `incidence_cos_floor`로 하한) + 점밀도(`n_points`로 나눔)를 결합해 계산하며, 입사각은 별도 이웃탐색/포인트클라우드 라이브러리 없이 이전까지 누적된 `self._elevation`의 `np.gradient`로 근사한다 — 이웃 정보가 없는 콜드스타트 구간(처음 보는 셀, gradient가 NaN인 셀)은 수직 입사(`cos_theta=1.0`)로 보수적으로 폴백한다. 프로세스 노이즈 Q는 정적 지형 가정 하에 0으로 고정(파라미터로 노출하지 않음). 이미 값이 있는 셀에는 이노베이션 게이팅(`y**2/S > innovation_gate_threshold`, 기본 9.0 — 카이제곱분포 자유도 1, 약 3-시그마에 해당하는 고전적 추적이론의 표준값)을 적용해 이상치 스캔을 걸러낸다. EKF/UKF가 아니라 표준(선형) 칼만필터인 이유는 상태 전이가 없고(Q=0) 관측식이 완전히 선형(z = x + noise)이라 근사가 필요 없기 때문이다.
>
> 새 파라미터 `measurement_noise_base`, `distance_noise_coefficient`, `incidence_cos_floor`, `innovation_gate_threshold`가 추가됐다 (7절 참고). 앞의 두 값은 wheel(A300 `lidar3d_0`)/leg(Go2 OS1-32)가 서로 다른 LiDAR 기종이라 로봇별 yaml에 독립적으로 채웠으며, 팀이 아직 실측 정확도 스펙을 확정하지 않은 placeholder다.
>
> `_grow_to_fit`이 새로 확장되는 격자 영역을 패딩할 때 기존 `np.pad` 기본값(0)이 아니라 `np.nan`으로 채우도록 바꿨다 — `self._elevation`/`self._variance`로 바뀐 뒤에는 분산 0이 "완벽하게 확신한다"는 의미가 되어, 패딩된 새 셀의 칼만 게인이 `K = P/(P+R) = 0`으로 영구히 고정되는 치명적 버그가 되기 때문이다.
>
> `elevation_map`(`/wheel(leg)/elevation_map`)에 `elevation_variance` 레이어가 새로 추가됐다 — `elevation`과 동일한 축 뒤집기 + column-major 패킹 방식을 재사용했고, `basic_layers`에는 `elevation`만 남겼다(분산은 관측 여부 판단 기준이 아니라 부가 정보). 이는 `grid_map_msgs/GridMap`의 실제 데이터가 바뀌는 인터페이스 변경이라 CONTRIBUTING 2조에 따라 이 문서와 README를 함께 갱신했다 — 이 변경은 모듈 E(`agconav_map_fusion`)가 wheel/leg 중 더 확신 있는(분산이 작은) 쪽을 선택하는 데 쓰인다.
