# jongheon_dev.md — 시뮬레이션 개발일지 (이종헌 개인, git 제외)

> AG-CoNav 3로봇(드론·4륜·4족) 통합 시뮬레이션의 공통 인프라(월드·모델·브리지·bringup)
> 작업 기록. 파일·함수별 설명 + 트러블슈팅. `.gitignore`에 등록되어 커밋되지 않음.

---

## 0. 환경 / 버전

| 항목 | 값 |
| --- | --- |
| OS / ROS | Ubuntu 24.04 / ROS 2 **Jazzy** |
| 시뮬 | **Gazebo Harmonic (gz-sim 8.11)** — `gz sim`, 벤더 `ros-jazzy-ros-gz` |
| 4륜 | Clearpath A300 (apt `ros-jazzy-clearpath-simulator`), `config/clearpath_a300/robot.yaml` |
| 4족 | Unitree Go2 + CHAMP (`unitree_go2_ros2_jazzy`, vendored 원본 기반 + AG-CoNav ros2_control 연동) |
| 드론 | Gazebo 예제 X3 → `agconav_description/models/agconav_drone` 로 로컬화 |
| 실행 | `ros2 launch agconav_bringup agconav_sim.launch.py` (`ROS_DOMAIN_ID=42`) |

**중요 환경 변수**: gz sim은 `GAZEBO_MODEL_PATH`(classic)가 아니라 **`GZ_SIM_RESOURCE_PATH`** 를 봄.

---

## 1. 패키지 · 파일 · 함수 설명

### 1-1. `agconav_worlds/worlds/agconav_integrated.sdf` — 통합 월드
- `<world name="agconav_world">` 하나. 구성 순서:
  - **`<gui>`**: GUI 카메라 초기 시점(`MinimalScene`의 `<camera_pose>-13 -13 10 0 0.5 0.785</camera_pose>`) — 원점의 로봇 3대를 3/4 각도로 내려다봄. + WorldControl/WorldStats 등 표준 패널.
  - **`<spherical_coordinates>`**: GPS/map 원점(서울 성수동 위경도). ※ 오타 `spherial`→`spherical` 수정함.
  - **시스템 플러그인**: physics, sensors(ogre2), **`gz-sim-imu-system`**(imu), `gz-sim-navsat-system`(gps), Multicopter 모터/컨트롤(드론).
  - **`ground_plane`**: 평지 500×500, z=0 (로봇 스폰 지면).
  - **ENVIRONMENT include**: 기본값 Gazebo 예제 **Depot**(창고). collision이 평면(z=0, solid)이라 로봇이 확실히 스폰됨. `<uri>` 한 줄만 바꾸면 교체(직접 만든 서울 월드 = `model://<모델>`).
  - **X3(드론) include**: `<pose>0 -4 0.2 ...>`, 모터/컨트롤 플러그인 포함(`motorConstant`/`forceConstant` = `8.436e-05`, 15kg 스펙에 맞춤).

### 1-2. `agconav_description` — 로봇 모델/설명
- `models/agconav_drone/model.sdf` — 드론(X3 3배 스케일). base_link 질량 14.5kg + 로터 0.125kg×4 = 총 15kg, 관성 재계산, 올리브 군용색. 센서 3종:
  - `os1_lidar_mount` 외형과 `os1_lidar` 스캔 프레임을 분리. 하우징은 중앙 하단에 밀착하고 스캔 프레임만 pitch+90°로 회전(topic `drone/points`, frame `drone/os1_lidar`)
  - `drone_imu_sensor`(imu, topic `drone/imu`, frame `drone/base_link`)
  - `navsat_sensor`(navsat, 중앙 상단 밀착, topic `drone/gps`, frame `drone/gps_link`)
- `urdf/leg/leg_sensors.xacro` — Go2에 얹는 센서 오버레이 매크로(`leg_sensors`, params parent/ns). `os1_lidar`/`leg_imu_sensor`/`navsat_sensor`. ※ imu 이름을 `leg_imu_sensor`로 한 이유: go2 원본이 `imu_sensor`를 이미 써서 충돌.
- `urdf/leg/leg_with_sensors.urdf.xacro` — **래퍼**. 외부 go2 원본 xacro + `leg_sensors.xacro`를 include하고 매크로 호출(parent=base_link, ns=leg). ※ 주석에 "콜론+공백" 쓰면 robot_description yaml 파싱 깨짐(주의).
- `CMakeLists.txt`: `models`, `urdf` 디렉터리 통째 설치(하위 폴더 포함).

### 1-3. `agconav_gz_bridge` — 센서/TF 브리지 (신규, 공통 인프라)
- `launch/sensor_bridge.launch.py`:
  - **gz→ROS 센서 브리지**(`ros_gz_bridge parameter_bridge`): 3로봇 센서를 모듈 계약 이름으로 remap. `/X/points`(PointCloud2), `/X/imu`(Imu), `/X/gps`(NavSatFix). wheel은 clearpath가 `launch_enabled:false`라 ROS로 안 올려서 **gz 토픽(`/wheel/sensors/...`)을 직접 브리지**.
  - **`tf_prefix_relay` 노드 2개**(leg/wheel): 사설 `/X/tf`를 읽어 전역 `/tf`로 `leg/*`·`wheel/*` 접두어 재발행.
- `agconav_gz_bridge/tf_prefix_relay.py` — `TfPrefixRelay` 노드:
  - `_fix(frame)`: 공유 프레임(map)·이미 접두어 붙은 것 제외하고 `<prefix>/` 부착.
  - `_on_tf`: 동적 TF 각 메시지 접두어 붙여 `/tf`로 재발행.
  - `_on_tf_static`: **정적 TF 누적** — child_frame_id 키로 dict에 쌓아 매번 전체 집합 재발행(안 그러면 depth=1 구독자가 마지막 것만 받음).

### 1-4. `agconav_bringup` — 통합 실행
- `launch/agconav_sim.launch.py` — 최상위. Gazebo를 일시정지 상태로 1회 시작하고, `GZ_SIM_RESOURCE_PATH` 설정, `/clock`·드론 cmd_vel 브리지, A300 spawn, Go2 spawn, 센서 브리지 include, (옵션)Nav2. 로봇 스폰 좌표: wheel(-5,0,0.3)·leg(5,0,0.25)·drone(0,-4,0.2).
  - nav2 인자(`use_nav2`/`nav2_params_file`) 선언 + `use_namespace:"True"`(wheel/leg 분리).
- `launch/go2_spawn.launch.py` — Go2 spawn 전용(원본 gz 실행 제외). 핵심 수정:
  - `default_model_path` → `urdf/leg/leg_with_sensors.urdf.xacro`(센서 오버레이 래퍼).
  - `ros_control_file` 기본값 → `agconav_bringup/config/leg_controllers.yaml`. 같은 경로를 xacro와 Gazebo `gz_ros2_control` 플러그인까지 전달.
  - 물리가 멈춘 상태에서 컨트롤러를 load/configure하고, 두 컨트롤러를 동시에 활성화한 뒤 월드를 재개.
  - **leg 스택 전체를 `GroupAction`+`SetRemap`으로 `/tf`→`/leg/tf` 격리**(CHAMP가 프레임을 하드코딩해 frame_prefix를 못 쓰므로).
  - go2 원본 imu(`/imu/data`) 대신 오버레이 imu(`/leg/imu`)를 EKF 2개 + `state_estimation`에 재배선(imu0 인라인/remap). 원본 velodyne/lidar_l1/rgb 브리지는 비활성화.
- `config/leg_controllers.yaml` — Go2 실제 12관절용 controller manager 설정. `effort` 명령 인터페이스, position/velocity 상태 인터페이스, 12관절 PID gain.

### 1-5. `config/clearpath_a300/robot.yaml` — 4륜 설정
- clearpath가 이걸로 URDF/센서를 **자동 생성**(launch의 `generate:true`). imu/lidar3d/gps 센서 정의. ※ `launch_enabled:false`라 clearpath는 센서를 ROS로 안 올림 → 우리 브리지가 gz에서 직접 가져옴.

---

## 2. 핵심 설계 결정

- **센서 토픽 계약**: README §5 기준 `/X/points`·`/X/imu`·`/X/gps`. (모듈 B/C 문서는 `/X/gps/fix`를 썼으나 README로 통일.)
- **TF 아키텍처(하이브리드)**: 각 로봇은 사설 `/X/tf`(루트 프레임)에서 내부 동작(CHAMP/clearpath 무손), 리레이가 전역 `/tf`에 `X/*` 접두어 통합 뷰 제공(모듈 E·RViz·크로스로봇).
- **환경**: terrain mesh 예제(Sonoma)는 collision 구멍 때문에 불가 → **Depot처럼 collision이 평면/solid인 것** 사용.

---

## 3. 트러블슈팅 로그 (증상 → 원인 → 해결)

| # | 증상 | 원인 | 해결 |
| --- | --- | --- | --- |
| 1 | `ROS parameters None must be a dictionary` | robot.yaml lidar3d `ros_parameters:` 값 없이 하위 들여쓰기 오류 | `ouster_driver`를 `ros_parameters` 하위로 들여쓰기 |
| 2 | robot.yaml 오타 | `flase`, `urdf_enavled` | `false`, `urdf_enabled` |
| 3 | launch 실행 즉시 종료 | `declare_use_nav2`/`declare_nav2_params_file` 반환목록 누락 | 두 선언 추가 |
| 4 | `XML_ERROR_PARSING_ELEMENT L127` | world `<pose>...</pose` 닫는 `>` 누락 | `</pose>` |
| 5 | `Unable to find uri[model://agconav_drone]` | gz sim은 `GZ_SIM_RESOURCE_PATH` 사용(classic `GAZEBO_MODEL_PATH` 아님) | launch에서 `GZ_SIM_RESOURCE_PATH`에 models 경로 추가 |
| 6 | `Failed to load system plugin gz-sim-imu-sensor-system` | 존재하지 않는 플러그인명 | `gz-sim-imu-system` |
| 7 | Nav2 `local_costmap already added to executor` / `No critics` | nav2_bringup에 `use_namespace:=true` 미전달 → wheel/leg 루트 충돌 | `use_namespace:"True"` 추가 |
| 8 | leg 센서 안 뜸 (`sensor imu_sensor already exists`) | go2 원본 imu와 오버레이 imu 이름 충돌 | 오버레이 imu → `leg_imu_sensor` |
| 9 | wheel 계약 토픽 데이터 없음 | clearpath `launch_enabled:false`라 ROS 미발행 | gz 토픽(`/wheel/sensors/...`)을 직접 브리지 |
| 10 | robot_description yaml 파싱 에러 | 래퍼 xacro 주석의 "콜론+공백"이 출력에 남음 | 주석에서 콜론 제거 |
| 11 | leg frame 네임스페이스 불가 | CHAMP가 base_link 등 프레임 하드코딩(frame 파라미터 없음) | `/tf`→`/leg/tf` 격리 + `tf_prefix_relay` |
| 12 | 전역 /tf에 leg 정적 프레임 일부만 | 리레이가 static TF 누적 안 함(depth=1) | 리레이 static 누적 발행 |
| 13 | 원본 imu 남으면 중복 | go2 imu가 CHAMP EKF 입력 | EKF 2개 + state_estimation을 `/leg/imu`로 재배선, 원본 비활성화 |
| 14 | 가제보 GUI OpenGL 크래시 | **NVIDIA 드라이버/커널 버전 불일치**(세션 중 apt 업데이트) | **재부팅** |
| 15 | 로봇이 지면에 박힘 | Sonoma Raceway collision mesh가 희소(구멍) → 관통 낙하 | terrain 예제 폐기, **Depot**(평면 collision) 사용 |
| 16 | `tf2_echo` 조회 실패(프레임 없음) | `use_sim_time` 미설정으로 sim-time 스탬프를 wall-clock으로 조회 | `--ros-args -p use_sim_time:=true` |
| 17 | 반복 런치로 gz `create` 서비스 대기 무한 | 기동 중 gz를 kill(자업자득)·orphan 노드 충돌 | 프로세스 완전 정리 후 클린 런치 |
| 18 | Go2가 생성 직후 옆으로 넘어짐 | 제어기가 활성화되기 전에 물리가 진행되고, ros2_control 초기 관절값이 0이라 서 있는 자세가 아니었음 | 초기 관절값(hip 0, upper 1.0143535, lower -2.0287070) 지정 + paused spawn → controller load/configure → 동시 활성화 → unpause |
| 19 | 작성한 `leg_controllers.yaml`이 실제로 적용되지 않음 | description 아래 설정 파일은 설치/런치되지 않았고 Unitree Gazebo xacro가 외부 YAML을 하드코딩 | YAML을 `agconav_bringup/config`로 이동하고 `ros_control_file` launch 인자를 xacro/Gazebo 플러그인까지 전달 |
| 20 | `ros2 control switch_controllers`가 거대한 정수 변환 오류로 종료 | Jazzy `ros2controlcli`가 명시한 `--switch-timeout`을 문자열로 받아 나노초 변환 시 문자열 반복 | 해당 옵션을 생략해 CLI 기본 실수값 5.0 사용, `--activate`는 가변 인자이므로 마지막에 배치 |
| 21 | 드론과 상·하부 센서 사이에 빈 공간이 보임 | collision box가 아니라 실제 X3 mesh의 해당 위치 표면을 기준으로 장착하지 않음 | mesh 중앙 정점 범위를 계산해 GPS 하단과 LiDAR 상단이 동체 외형에 맞닿는 좌표로 이동 |
| 22 | `RTPS_TRANSPORT_SHM Failed init_port fastrtps_port7001` 반복 + wheel spawner service 대기 | 이전 통합 launch/Gazebo와 wheel teleop orphan 여러 개가 남았고 ROS 2 daemon이 `/dev/shm/fastrtps_port7001_el`을 점유. Go2 spawner가 전역 controller-spawner lock을 잡은 채 대기해 wheel spawner도 지연 | 중복 AG-CoNav 프로세스와 고착 daemon을 종료한 뒤 기본 Fast DDS로 클린 재실행. Cyclone/UDP-only 강제는 controller spawner lock 실패가 재현되어 채택하지 않음 |

**gz 프로세스 확인 팁**: `pgrep -f "gz sim"`은 자기 명령줄을 매칭하는 오탐이 잦음. `ps -eo comm | grep -E '^(gz|ruby)'`(실행파일명)으로 확인.

---

## 4. 알아둘 잔여/주의

- **드론 TF 미발행**: 드론은 gz 모델이라 robot_state_publisher가 없어 `map→drone/base_link→drone/os1_lidar` TF는 아직 없음(센서 메시지 frame_id만 `drone/*`). → 모듈 A(`drone_path_player`)가 pose로 발행할 몫.
- **wheel 센서 프레임 이름**: clearpath 자동명명(`wheel/lidar3d_0_sensor_link` 등)이라 계약의 `wheel/os1_lidar`와 다름(clearpath 제약).
- **환경 500m**: Depot은 창고(수십 m). 진짜 500m 드론 매핑은 직접 만든 서울 평지 월드 필요.
- **폴더**: leg xacro는 `urdf/leg/`로 정리함. wheel 설정은 clearpath 규약상 `config/clearpath_a300/` 유지.

---

## 5. 2026-07-30 통합 상태와 확정 인터페이스

아래 표는 README의 팀 인터페이스 계약과 현재 시뮬레이션 구현을 함께 정리한 것이다.
`확정/구현`은 현재 코드에서 연결된 항목, `계약`은 후속 모듈이 맞춰야 할 이름,
`미구현`은 이름만 확정되고 아직 발행 노드가 없는 항목을 뜻한다.

### 5-1. 이번 작업 완료 상태

| 대상 | 발생 상황 | 적용한 해결 | 현재 상태 |
| --- | --- | --- | --- |
| drone 센서 | 동체와 상·하부 센서 사이에 빈 공간이 보임 | X3 mesh 실제 중앙 표면을 계산해 LiDAR 상단과 GPS 하단을 동체에 접촉시킴. LiDAR 외형 링크와 하향 스캔 프레임 분리 | 완료, SDF 검사 통과 |
| wheel 센서 | 센서가 차체와 멀리 떨어져 있었음 | LiDAR z=0.23 m, GPS z=0.24 m로 하향 조정 | 완료, 부착 정상 확인 |
| leg 센서 | 센서 부착 상태 | 기존 LiDAR z=0.09 m, GPS z=0.07 m 유지 | 부착 정상 확인 |
| leg 자세 | 생성과 동시에 넘어짐 | 서 있는 초기 관절각 설정, 컨트롤러 준비 전 월드 pause, 활성화 후 unpause | 완료 |
| leg controller YAML | description 경로의 파일이 미적용되고 관절 정의도 실제 Go2와 불일치 | `agconav_bringup/config/leg_controllers.yaml`로 이동, 실제 12관절 effort/PID 설정 적용 | 완료 |
| controller 배선 | Unitree xacro가 외부 기본 YAML을 하드코딩 | `ros_control_file`을 최상위 launch → Go2 xacro → `gz_ros2_control` 플러그인까지 전달 | 완료 |

### 5-2. 좌표계·네임스페이스 공통 규약

| 항목 | 확정 값 |
| --- | --- |
| 전역 좌표 | `map`, Gazebo world 원점 `(0,0,0)`과 동일 |
| 월드 좌표축 | ENU |
| 로봇 좌표축 | FLU |
| 좌표계 방향 | 오른손 좌표계 |
| 로봇 네임스페이스 | `/drone`, `/wheel`, `/leg` |
| 길이·위치 | m |
| 각도 | rad |
| 선속도·각속도 | m/s, rad/s |
| 자세 | quaternion |
| 시간 | ROS simulation time, Gazebo `/clock` |
| 전역 datum | 서울 성수동 `37.5412278, 127.0565741` |
| 공통 지도 해상도 | 0.10 m/cell |
| 2.5D 핵심 layer | `elevation`, 미관측 셀 `NaN` |

### 5-3. 로봇별 spawn·모델·제어

| 로봇 | 모델 | spawn pose `(x,y,z,yaw)` | 이동 제어 | controller manager |
| --- | --- | --- | --- | --- |
| drone | X3, 15 kg, 3배 scale | `(0,-4,0.2,0)` | 현재 `/drone/cmd_vel` → Gazebo MulticopterVelocityControl. 팀 계약의 `/drone/cmd_pose` 경로 재생기는 별도 모듈 | 해당 없음 |
| wheel | Clearpath A300 | `(-5,0,0.3,0)` | Clearpath twist mux → `platform_velocity_controller` | `/wheel/controller_manager` |
| leg | Unitree Go2 + CHAMP | `(5,0,0.25,3.14159)` | `/leg/cmd_vel` → CHAMP → joint trajectory | `/controller_manager` |

### 5-4. 로봇별 센서 장착값

장착 좌표는 각 로봇 `base_link` 기준이다. drone LiDAR의 하우징은 수직이며,
별도 센서 프레임만 pitch `+1.5708 rad`로 회전해 하향 스캔한다.

| 로봇 | 센서 | 링크/frame_id | 장착 xyz (m) | 장착 rpy (rad) | Gazebo rate | 주요 설정 |
| --- | --- | --- | --- | --- | --- | --- |
| drone | OS1-32 LiDAR 외형 | `os1_lidar_mount` | `(0,0,-0.175406)` | `(0,0,0)` | — | mesh 접촉 기준 위치에서 0.5cm 위로 조정 |
| drone | OS1-32 스캔 | `drone/os1_lidar` | `(0,0,-0.175406)` | `(0,1.5708,0)` | 10 Hz | 1024×32, 수평 360°, 수직 ±21.2°, 0.5–170 m, 하향 |
| drone | IMU | `drone/base_link` | `(0,0,-0.04)` | `(0,0,0)` | 100 Hz | base 기준 4cm 아래, 시각 형상·collision 없음 |
| drone | GPS/NavSat | `drone/gps_link` | `(0,0,0.172223)` | `(0,0,0)` | 10 Hz | mesh 접촉 기준보다 0.5cm 아래. collision 없음 |
| wheel | Ouster OS1 | `wheel/lidar3d_0_sensor_link` | `(0,0,0.23)` | `(0,0,0)` | 20 Hz | 현재 Clearpath 생성값 1024×64, 수직 ±15°, 0.9–130 m. base collision이 상판과 약 12.26mm 겹침 |
| wheel | Phidgets IMU | `wheel/imu_0_link` | `(-0.10,0,0.202259)` | `(0,0,0)` | 시뮬 100 Hz | 이전 위치에서 4cm 아래. visual은 차체 내부이고 IMU 자체 collision 없음. robot.yaml 하드웨어 data interval은 20ms |
| wheel | NovAtel GPS | `wheel/gps_0_link` | `(-0.15,0,0.242259)` | `(0,0,0)` | 1 Hz | 상판과 겹치지 않도록 2.259mm 위로 조정 |
| leg | OS1-32 LiDAR | `leg/os1_lidar` | `(0,0,0.09385)` | `(0,0,0)` | 10 Hz | trunk collision과 겹치지 않도록 3.85mm 위로 조정. 1024×32, 수평 360°, 수직 ±21.2°, 0.5–170 m |
| leg | IMU | `leg/base_link` | `(0,0,-0.025)` | `(0,0,0)` | 100 Hz | 원본 Go2 IMU 대신 `leg_imu_sensor` 사용, 기존 위치에서 2cm 더 아래 |
| leg | GPS/NavSat | `leg/gps_link` | `(-0.10,0,0.068)` | `(0,0,0)` | 10 Hz | 이전 위치에서 1.8cm 위. collision 없음 |

> 센서 통일 계약은 OS1-32이지만 wheel의 현재 Clearpath 생성 URDF는 수직
> sample이 64다. 알고리즘에서 채널 수를 강제한다면 wheel 설정을 32로 맞춰야 한다.

### 5-5. TF 트리·발행 소유권

| 로봇 | 계약 TF 사슬 | 내부 TF 토픽 | 전역 변환 방식 | 발행 소유권/현재 상태 |
| --- | --- | --- | --- | --- |
| drone | `map → drone/base_link → drone/{os1_lidar,gps_link}` | 없음 | 향후 직접 전역 `/tf` 발행 | `map→drone/base_link`는 `drone_path_player`, 센서 정적 TF는 별도 publisher/RSP가 필요. 현재는 센서 메시지 frame_id만 존재 |
| wheel | `map → wheel/odom → wheel/base_link → wheel/{lidar3d_0_sensor_link,imu_0_link,gps_0_link}` | `/wheel/tf`, `/wheel/tf_static` | `wheel_tf_prefix_relay`가 `map`을 제외한 frame에 `wheel/`을 붙여 `/tf`, `/tf_static`으로 재발행 | `map→wheel/odom`은 위치추정 모듈 B, `wheel/odom→wheel/base_link`는 Clearpath EKF/odometry, 센서 TF는 RSP |
| leg | `map → leg/odom → leg/base_footprint → leg/base_link → leg/{os1_lidar,gps_link}` | `/leg/tf`, `/leg/tf_static` | `leg_tf_prefix_relay`가 `map`을 제외한 frame에 `leg/`을 붙여 `/tf`, `/tf_static`으로 재발행 | 현재 `map→odom` static, `odom→base_footprint` EKF, `base_footprint→base_link` static, 관절·센서 TF는 RSP |

| TF 관계 | 단일 소유자 규칙 |
| --- | --- |
| `map → X/odom` | 위치추정(`robot_localization`)만 발행. 단, leg는 현재 기동 확인용 static publisher 사용 |
| `X/odom → X/base_link` | 각 로봇 odometry/EKF만 발행 |
| `X/base_link → X/센서` | `robot_state_publisher`만 발행 |
| `map → drone/base_link` | `drone_path_player`만 발행 |
| `/tf_static` relay | child frame 기준으로 누적 후 transient_local 재발행 |
| 제외 프레임 | `earth`, `utm`은 필요 확정 전까지 TF 트리에 넣지 않음 |

### 5-6. Gazebo 센서 토픽 → ROS 2 계약 토픽

| 로봇 | Gazebo 원본 토픽 | ROS 2 출력 | ROS 타입 | 방향 | 구현 |
| --- | --- | --- | --- | --- | --- |
| drone | `/drone/points/points` | `/drone/points` | `sensor_msgs/msg/PointCloud2` | GZ→ROS | 공통 sensor bridge |
| drone | `/drone/imu` | `/drone/imu` | `sensor_msgs/msg/Imu` | GZ→ROS | 공통 sensor bridge |
| drone | `/drone/gps` | `/drone/gps` | `sensor_msgs/msg/NavSatFix` | GZ→ROS | 공통 sensor bridge |
| wheel | `/wheel/sensors/lidar3d_0/scan/points` | `/wheel/points` | `sensor_msgs/msg/PointCloud2` | GZ→ROS | 공통 sensor bridge |
| wheel | `/wheel/sensors/imu_0/data` | `/wheel/imu` | `sensor_msgs/msg/Imu` | GZ→ROS | 공통 sensor bridge |
| wheel | `/wheel/sensors/gps_0/navsat` | `/wheel/gps` | `sensor_msgs/msg/NavSatFix` | GZ→ROS | 공통 sensor bridge |
| leg | `/leg/points/points` | `/leg/points` | `sensor_msgs/msg/PointCloud2` | GZ→ROS | 공통 sensor bridge |
| leg | `/leg/imu` | `/leg/imu` | `sensor_msgs/msg/Imu` | GZ→ROS | Go2 전용 bridge |
| leg | `/leg/gps` | `/leg/gps` | `sensor_msgs/msg/NavSatFix` | GZ→ROS | 공통 sensor bridge |

### 5-7. 전체 ROS 2 토픽·액션 계약

`X`는 `drone`, `wheel`, `leg`이며, QoS는 팀 모듈 간 계약값이다.

| 토픽/액션 | 타입 | 발행 → 구독 | QoS | 상태/비고 |
| --- | --- | --- | --- | --- |
| `/X/points` | `sensor_msgs/msg/PointCloud2` | Gazebo bridge → 지도화·Nav2 | best_effort | 3대 구현 |
| `/X/gps` | `sensor_msgs/msg/NavSatFix` | Gazebo bridge → 위치추정 | best_effort | 3대 구현 |
| `/X/imu` | `sensor_msgs/msg/Imu` | Gazebo bridge → 위치추정/EKF | best_effort | 3대 구현 |
| `/X/odom` | `nav_msgs/msg/Odometry` | odometry bridge/EKF → 위치추정·Nav2 | reliable | 계약명. leg 내부는 현재 `/odom`, wheel 원본은 `/wheel/platform/odom`; 모듈 B에서 계약명 정합 필요 |
| `/drone/cmd_pose` | `geometry_msgs/msg/PoseStamped` | drone path player → drone | reliable | 팀 계약, path player 미구현 |
| `/drone/cmd_vel` | `geometry_msgs/msg/Twist` | 조종 노드 → X3 velocity controller | reliable | 현재 Gazebo 비행 입력 구현 |
| `/wheel/cmd_vel` | 계약 `geometry_msgs/msg/Twist` | Nav2 → wheel | reliable | Clearpath twist mux는 현재 `use_stamped=True`이므로 실제 입력 타입 정합 필요 |
| `/leg/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2/teleop → CHAMP | reliable | 구현 |
| `/drone/elevation_map` | `grid_map_msgs/msg/GridMap` | 드론 지도화 → 주행성·병합 | reliable + transient_local | 모듈 A 구현 대상 |
| `/wheel/elevation_map` | `grid_map_msgs/msg/GridMap` | wheel 지도화 → 병합 | reliable + transient_local | 모듈 D 구현 대상 |
| `/leg/elevation_map` | `grid_map_msgs/msg/GridMap` | leg 지도화 → 병합 | reliable + transient_local | 모듈 D 구현 대상 |
| `/wheel/nav_map` | `nav_msgs/msg/OccupancyGrid` | 주행성 분석 → wheel Nav2 | reliable + transient_local | 모듈 F 구현 대상 |
| `/leg/nav_map` | `nav_msgs/msg/OccupancyGrid` | 주행성 분석 → leg Nav2 | reliable + transient_local | 모듈 F 구현 대상 |
| `/merged_map` | `grid_map_msgs/msg/GridMap` | map fusion → RViz·저장 | reliable + transient_local | 모듈 E 구현 대상 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → 전체 노드 | best_effort | 중앙 bridge 1개 |
| `/tf` | `tf2_msgs/msg/TFMessage` | TF 소유 노드/relay → 전체 | 기본 TF QoS | 전역 prefix 통합 뷰 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | RSP/static publisher/relay → 전체 | transient_local | 정적 TF 누적 |
| `/wheel/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 사용자/상위계획 → wheel Nav2 | action 기본 | Nav2 사용 시 |
| `/leg/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 사용자/상위계획 → leg Nav2 | action 기본 | Nav2 사용 시 |

### 5-8. 내부 제어·상태 토픽

| 로봇 | 토픽 | 타입 | 용도 |
| --- | --- | --- | --- |
| drone | `/X3/gazebo/command/twist` | `gz.msgs.Twist` | ROS `/drone/cmd_vel`의 Gazebo 도착점 |
| drone | `/model/x3/odometry` 또는 모델명 기반 odometry | `gz.msgs.Odometry` | Gazebo OdometryPublisher 출력; ROS 계약 브리지는 아직 없음 |
| wheel | `/wheel/platform/odom` | `nav_msgs/msg/Odometry` | Clearpath 플랫폼 odometry |
| wheel | `/wheel/platform/cmd_vel` | controller 입력 타입 | twist mux에서 플랫폼 velocity controller로 전달 |
| leg | `/joint_group_effort_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | CHAMP가 12관절 effort controller에 주는 궤적 |
| leg | `/joint_states` | `sensor_msgs/msg/JointState` | Go2 12관절 상태 |
| leg | `/odom/raw` | `nav_msgs/msg/Odometry` | CHAMP/EKF 입력 |
| leg | `/odom/local` | `nav_msgs/msg/Odometry` | base-to-footprint EKF 출력 |
| leg | `/odom` | `nav_msgs/msg/Odometry` | footprint-to-odom EKF 출력, 향후 `/leg/odom` 계약명으로 정합 |

### 5-9. Go2 ros2_control 확정값

| 항목 | 값 |
| --- | --- |
| 설정 파일 | `src/agconav_bringup/config/leg_controllers.yaml` |
| update rate | 250 Hz |
| state broadcaster | `joint_states_controller` |
| trajectory controller | `joint_group_effort_controller` |
| 명령 interface | `effort` |
| 상태 interface | `position`, `velocity` |
| 관절 수 | 12개: LF/RF/LH/RH × hip/upper/lower |
| PID | 전 관절 `p=100.0, i=0.2, d=1.0, i_clamp=2.5` |
| 초기 hip | 0 rad |
| 초기 upper leg | 1.0143535 rad |
| 초기 lower leg | -2.0287070 rad |
| 시작 순서 | Gazebo pause → spawn → controller load/configure(inactive) → 두 controller 동시 activate → 0.25초 뒤 unpause |

### 5-10. 검증 결과

| 검사 | 결과 |
| --- | --- |
| drone SDF 구문 | `gz sdf -k` 통과(`gz_frame_id` 확장 경고만 존재) |
| drone 센서 런타임 pose | LiDAR mount `(0,0,-0.175406)`, GPS `(0,0,0.172223)` 설정 |
| 전체 센서 ROS 계약 | drone/wheel/leg 각각 `/points`, `/imu`, `/gps` 총 9개 토픽의 타입과 실제 메시지 수신 확인 |
| LiDAR 실측 | drone/wheel/leg 모두 유효한 `PointCloud2` 수신, 각 cloud width 1024 |
| IMU 실측 | drone 약 `(0,0,9.8)`, wheel 약 `(0,0,9.8)`, leg 약 `(0.142,0.009,10.171) m/s²`; 모두 finite이며 leg 값은 관절 제어 진동 범위 |
| GPS 실측 | drone/wheel/leg 모두 유효한 `NavSatFix` 위도·경도 수신 |
| drone 센서 간섭 | IMU·LiDAR·GPS에 collision geometry가 없어 동체 외형과 겹쳐도 접촉력 없음. 모든 센서값 정상 |
| wheel 센서 간섭 | IMU는 collision 없음. GPS collision은 상판과 0 mm 접촉/0 mm 침범. LiDAR base collision은 상판과 12.259 mm 겹치지만 고정관절 lump 후 같은 `base_link`이므로 self-collision 없음. 모든 센서값 정상 |
| leg 센서 간섭 | GPS·IMU는 collision 없음. GPS 외형은 trunk 위 1 mm 여유. OS1 collision 하단과 trunk 상단은 정확히 접촉(0 mm 침범)하며 같은 lumped link. 모든 센서값 정상 |
| Fast DDS 클린 재실행 | `fastrtps_port7001` 오류 0건. Go2 controller 전환 및 wheel `platform_velocity_controller`·`joint_state_broadcaster` 활성화 확인 |
| leg/wheel xacro·URDF | xacro 및 `check_urdf` 통과 |
| launch Python | `py_compile` 통과 |
| YAML | 파싱 통과 |
| 관련 패키지 build | `unitree_go2_description`, `agconav_description`, `agconav_bringup` 성공 |
| controller 파일 적용 | 설치된 `agconav_bringup/config/leg_controllers.yaml` 로딩 로그 확인 |
| controller 활성화 | `joint_states_controller`, `joint_group_effort_controller` 동시 전환 성공 |
| Go2 자세 유지 | 약 40초 후 xyz `(5.01014,0.000387,0.238958)`, roll/pitch `(0.000963,-0.005127)`로 직립 유지 |

---

## 6. 개발 8파트 요약 (일지 흐름)

이번 주 작업을 8개 파트로. 각 파트 = 무엇을 / 어떻게(파일) / 핵심 이슈→해결.

1. **드론 커스터마이징** — X3를 3배 스케일·15kg·올리브색·모터출력 조정.
   `models/agconav_drone/model.sdf`(scale/mass/material), world(`motorConstant`/`forceConstant`=`8.436e-05`).
   → 15kg인데 모터 그대로면 못 뜸 → 질량비만큼 모터상수 상향(T/W 유지).

2. **Nav2 연동(임시 테스트)** — `use_nav2` 배선 검증 후 임시 파일 삭제.
   임시 `nav2_common.yaml` 생성→`use_nav2:=true` 테스트→삭제. `agconav_sim.launch.py:_nav2_for`.
   → wheel/leg 스택이 executor 충돌 → nav2_bringup에 **`use_namespace:"True"`** 미전달이 원인 → 추가.

3. **드론·4륜 센서 장착** — OS1-32/IMU/GPS.
   드론 `model.sdf`(mount/scan 분리), 4륜 `config/clearpath_a300/robot.yaml`.
   → robot.yaml `ros_parameters` 들여쓰기 오류 크래시 → 하위 들여쓰기 수정. 센서가 떠 보임 → mesh 정점범위 계산해 표면 밀착.

4. **4족 센서 오버레이** — 외부 Go2 무수정으로 센서 부착.
   `urdf/leg/leg_sensors.xacro`(매크로) + `leg_with_sensors.urdf.xacro`(래퍼).
   → 원본 `imu_sensor` 이름 충돌 → `leg_imu_sensor`. 래퍼 주석 콜론이 robot_description yaml 파싱 깸 → 콜론 제거.

5. **스폰 넘어짐 해결** — Go2 생성 직후 넘어짐 방지.
   `go2_spawn.launch.py`(paused 스폰→load/configure→activate→unpause), `config/leg_controllers.yaml`(12관절 PID).
   → 컨트롤러 활성 전 물리 진행+관절0 → 넘어짐 → paused 시퀀싱+초기자세. yaml 미적용 → bringup/config 이동+`ros_control_file` 전달. `switch_controllers --switch-timeout` 정수변환 오류 → 옵션 생략.

6. **예제 환경** — 야외 씬 + GUI 카메라 + 확실 스폰.
   world `ground_plane`(평지 solid)+Fuel 오브젝트, `<gui> camera_pose`.
   → Sonoma terrain mesh 구멍 → 관통 낙하 → 평지+오브젝트. GUI 불완전 `<gui>` → 도움말 겹침 → 기본 gui.config 전체 세트.

7. **센서 브릿지** — gz 센서→계약 ROS 토픽(`/X/points`·`/X/imu`·`/X/gps`).
   신규 패키지 `agconav_gz_bridge/launch/sensor_bridge.launch.py`(parameter_bridge+remap).
   → wheel 계약 토픽 데이터 0 → clearpath `launch_enabled:false` → gz 토픽 직접 브리지.

8. **TF 네임스페이스** — 전역 `/tf`에 `leg/*`·`wheel/*` 접두어(멀티로봇 충돌 방지).
   `go2_spawn.launch.py`(SetRemap `/tf`→`/leg/tf`), `agconav_gz_bridge/tf_prefix_relay.py`.
   → CHAMP가 프레임 하드코딩→frame_prefix 불가→사설 tf 격리+relay. static 일부만 뜸→child 키로 누적 발행.
