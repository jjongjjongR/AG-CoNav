# AG-CoNav 시뮬레이션 재현 가이드 (by 이종헌)

> **목적** — 아무것도 설치돼 있지 않은 새 컴퓨터에서 이 저장소를 clone 한 뒤,
> 아래 순서대로 명령어만 따라 하면 **드론(X3) · 4륜(A300) · 4족(Go2) 세 로봇이 하나의
> Gazebo 월드에 동시에 뜨는 통합 시뮬레이션**을 그대로 재현할 수 있게 하는 실행 가이드다.
>
> 프로젝트 개요·아키텍처·모듈 규약은 [README.md](README.md), 기여 규칙은 [CONTRIBUTING.md](CONTRIBUTING.md) 참조.
> 이 문서는 **"클론하고 바로 돌리기"** 한 가지만 다룬다.

---

## 0. 현재 재현되는 범위

이 가이드대로 하면 아래 상태까지 도달한다 (Phase 2 1차 목표 = 완료).

```
[agconav_world 통합 월드]  ← Gazebo Harmonic 1개 인스턴스
├── x3      : 드론 (3배 스케일, 15kg, 올리브 군용색, 로터+모터 플러그인)
├── wheel   : Clearpath Husky A300 (4륜)
└── leg     : Unitree Go2 + CHAMP (4족)
```

- Gazebo 서버 **1개**, 월드 **1개**, `/clock` 소스 **1개**
- A300 / Go2 의 controller_manager 가 네임스페이스로 분리되어 충돌 없음
- (옵션) Nav2 는 `use_nav2:=true` 로 골격까지 기동 확인됨 — 단 지도·위치추정(모듈 B/F) 미완이라 실주행은 아직 X (→ §9)

---

## 1. 사전 요구사항 (버전 고정)

| 항목 | 값 | 비고 |
| --- | --- | --- |
| OS | **Ubuntu 24.04 LTS (Noble)** | 다른 버전 비권장 |
| ROS 2 | **Jazzy Jalisco** | LTS |
| 시뮬레이터 | **Gazebo Harmonic (gz-sim 8)** | ROS-Gz 벤더로 설치됨 |
| 4륜 모델 | Clearpath Simulator (apt) | `ros-jazzy-clearpath-simulator` |
| 4족 모델 | [RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy) | vcstool, 커밋 `edfb187…` 고정 |
| Python | 3.12 (시스템, **venv 안 씀**) | |
| 빌드 | `colcon build --symlink-install` | |

> 4족 저장소는 메쉬 포함 ~170MB라 이 git 히스토리에 넣지 않는다. `deps.repos`에 URL과
> 커밋 해시를 고정해두고 `vcstool`로 받는다(§4). 팀 전원이 **같은 커밋**을 쓰기 위함.

---

## 2. Step 1 — ROS 2 Jazzy + 시스템 패키지 설치

ROS 2 Jazzy 자체가 없다면 먼저 [공식 설치 문서](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)로 apt 소스를 등록한 뒤 아래를 실행한다. (이미 Jazzy가 있으면 이 블록만 실행)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-clearpath-simulator \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-pointcloud-to-laserscan \
  python3-colcon-common-extensions python3-vcstool python3-rosdep \
  python3-numpy python3-scipy python3-matplotlib python3-opencv
```

- `ros-jazzy-ros-gz` 가 **Gazebo Harmonic(gz-sim 8)** 을 벤더로 함께 끌어온다.
- `ros-jazzy-clearpath-simulator` 가 A300(Husky) 생성에 필요한 `clearpath_*` 패키지 전체를 설치한다.

설치 확인:

```bash
ros2 --version          # jazzy
gz sim --version        # Gazebo Sim, version 8.x
ros2 pkg prefix clearpath_gz   # /opt/ros/jazzy 가 나오면 OK
```

---

## 3. Step 2 — 저장소 clone

```bash
git clone <이 저장소 URL> AG-CoNav
cd AG-CoNav
```

이후 모든 명령은 **저장소 루트(`AG-CoNav/`)에서** 실행한다.

---

## 4. Step 3 — 외부 패키지(Go2 + CHAMP) 가져오기

`src/unitree_go2_ros2_jazzy/` 는 git에 커밋돼 있지 않다(.gitignore + vcstool 관리). `deps.repos`로 받는다.

```bash
vcs import src < deps.repos
```

가져와진 것 확인:

```bash
ls src/unitree_go2_ros2_jazzy
# champ  champ_base  champ_msgs  unitree_go2_description  unitree_go2_sim  ...
```

> 나중에 이 외부 저장소를 갱신하려면 `vcs pull src` 후 `deps.repos`의 `version`을 새 커밋 해시로
> 고쳐 커밋한다. **임의 갱신 금지** — 전원이 같은 커밋을 써야 한다.

---

## 5. Step 4 — 의존성 해결 (rosdep)

Go2/CHAMP 등 소스 패키지의 시스템 의존성을 자동 설치한다.

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 6. Step 5 — 환경 변수 (`~/.bashrc`)

팀 전원 동일하게 맞춘다(도메인 격리·RMW 통일).

```bash
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
source ~/.bashrc
```

---

## 7. Step 6 — 빌드

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

- 인자 없는 전체 빌드가 `agconav_*` 3개 패키지 + Go2/CHAMP 패키지 전부를 빌드한다.
- 빌드 대상은 `package.xml`이 있는 패키지뿐이다. `agconav_drone`·`agconav_navigation` 등
  README만 있는 스텁 모듈은 colcon이 자동으로 건너뛴다(정상).

> ⚠️ **Go2 빌드 주의** — `colcon build --packages-up-to unitree_go2_sim` 처럼 `--up-to`로 받으면
> 커뮤니티 패키지의 의존성 선언이 불완전해 `unitree_go2_description` 등이 빌드되지 않고
> 실행 시 `PackageNotFoundError`가 난다. **위처럼 인자 없이 전체 빌드**하거나, 굳이 선택 빌드하려면
> 아래처럼 명시한다:
> ```bash
> colcon build --symlink-install --packages-select \
>   champ_msgs champ champ_base unitree_go2_description unitree_go2_sim \
>   agconav_worlds agconav_description agconav_bringup
> ```

새 터미널을 열 때마다 아래 두 줄이 필요하다(또는 `~/.bashrc`에 추가):

```bash
source /opt/ros/jazzy/setup.bash
source ~/AG-CoNav/install/setup.bash
```

---

## 8. Step 7 — 실행 (원클릭)

```bash
ros2 launch agconav_bringup agconav_sim.launch.py
```

이 launch 하나가 다음을 순서대로 띄운다:

```
agconav_sim.launch.py
├── Gazebo Harmonic 1회 실행 + agconav_world 로드
├── /clock bridge (중앙 1개)
├── 드론 cmd_vel bridge (/drone/cmd_vel → X3)
├── Clearpath A300 spawn (wheel)
└── Unitree Go2 spawn (leg)
```

**주요 launch 인자** (`이름:=값`으로 전달):

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `use_sim_time` | `true` | Gazebo 시간 사용 |
| `use_nav2` | `false` | Nav2 스택 기동 여부 (→ §9) |
| `clearpath_setup_path` | `<repo>/config/clearpath_a300` | A300 `robot.yaml` 위치 |
| `nav2_params_file` | `<repo>/src/agconav_navigation/config/nav2_common.yaml` | Nav2 파라미터(모듈 C, 미구현) |

---

## 9. (옵션) Nav2 켜기 — 현재 상태

```bash
ros2 launch agconav_bringup agconav_sim.launch.py use_nav2:=true
```

- Nav2 스택은 `/wheel`, `/leg` 네임스페이스로 **분리 기동되고 controller/planner/costmap이 활성화**되는 것까지 확인됨.
- 다만 실제 주행은 **아직 불가**하다: 지도(`/wheel/nav_map` 등, 모듈 F)와 위치추정(모듈 B),
  그리고 `nav2_common.yaml`(모듈 C) 이 아직 없다. `use_nav2:=true`로 켜면 `map_server`가
  지도가 없어 실패하는데, 이는 **정상(선행 모듈 미완)**이며 통합 배선 자체는 검증돼 있다.
- 모듈 C 담당이 `src/agconav_navigation/config/nav2_common.yaml` 을 채우고 B/F가 완성되면 실주행이 된다.

---

## 10. Step 8 — 정상 동작 확인

시뮬레이션이 뜬 상태에서 **새 터미널**을 열고(`source` 2줄 후) 확인한다.

**(1) Gazebo에 세 로봇이 올라왔는지**

```bash
gz model --list
# ground_plane, x3, wheel/robot, leg 가 보이면 성공
```

**(2) ROS 노드/네임스페이스 분리 확인**

```bash
ros2 node list | grep -E "controller_manager"
# /wheel/controller_manager  (A300)
# /controller_manager 또는 /leg/... (Go2)  ← 서로 다른 이름이면 충돌 없이 분리된 것
```

**(3) 컨트롤러 활성화 확인 (A300)**

```bash
ros2 control list_controllers -c /wheel/controller_manager
# joint_state_broadcaster       active
# platform_velocity_controller  active
```

**(4) Go2 관절 상태 발행 확인**

```bash
ros2 topic echo /joint_states --once   # 12개 관절(lf/lh/rf/rh × hip/upper/lower)
ros2 run tf2_ros tf2_echo base_link lf_foot_link   # TF 변환값 출력
```

> RViz2 에서 Go2 다리가 처음 잠깐 안 보일 수 있다. controller ↔ `robot_state_publisher` 시작
> 순서 때문에 동적 TF가 늦게 채워지는 것이며 곧 정상화된다(오류 아님).

---

## 11. Step 9 — 로봇 움직여 보기

### 드론 (X3)

launch가 `/drone/cmd_vel`(geometry_msgs/Twist) → X3 브리지를 이미 걸어둔다. 상승시키기:

```bash
ros2 topic pub -r 10 /drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {z: 0.0}}"
```

정지:

```bash
ros2 topic pub -1 /drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

Gazebo 쪽에서 직접 명령하거나 상태를 보려면:

```bash
gz topic -t "/X3/gazebo/command/twist" -m gz.msgs.Twist -p "linear:{z:0.1}"
gz topic -e -t "/model/x3/odometry"
```

### 4륜 A300 (`TwistStamped` 주의)

A300은 `geometry_msgs/msg/TwistStamped`를 쓴다(Go2의 `Twist`와 다름).

```bash
ros2 topic pub -r 10 /wheel/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.3}, angular: {z: 0.0}}}"
```

### 4족 Go2 (CHAMP, `Twist`)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/leg/cmd_vel
```

---

## 12. 센서 브리지(LiDAR 등) 참고

`ros_gz_bridge`로 Gazebo 점군을 ROS2 `PointCloud2`로 변환한다. 문법: `토픽명@ROS2타입[Gazebo타입`
(`[` = Gazebo→ROS2 단방향).

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /drone/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  /wheel/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  /leg/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked
```

> `/clock`은 중앙 launch에서 이미 1개 브리지로 연결한다. 로봇별로 또 연결하지 말 것(중복 발행).
> `gz_ros2_control`이 이미 주는 데이터(예: Go2 `/joint_states`)는 다시 브리지하지 않는다.

---

## 13. 트러블슈팅 (실제로 겪은 것들)

| 증상 | 원인 | 해결(이미 저장소에 반영됨) |
| --- | --- | --- |
| `ROS parameters None must be a dictionary` | `robot.yaml`의 lidar3d `ros_parameters` 들여쓰기 오류 | `config/clearpath_a300/robot.yaml` 수정됨 |
| `XML_ERROR_PARSING_ELEMENT` (world L127) | SDF `<pose>` 닫는 태그 누락 | `agconav_integrated.sdf` 수정됨 |
| `Unable to find uri[model://agconav_drone]` | gz-sim은 `GZ_SIM_RESOURCE_PATH`를 봄 (classic용 `GAZEBO_MODEL_PATH` 아님) | launch에서 `GZ_SIM_RESOURCE_PATH` 설정 |
| `Failed to load system plugin [gz-sim-imu-sensor-system]` | 플러그인 이름 오타 | `gz-sim-imu-system` 으로 수정됨 |
| Nav2 `local_costmap already added to an executor` / `No critics defined` | nav2_bringup에 `use_namespace:=true` 안 넘겨 두 스택이 루트에서 충돌 | launch `_nav2_for`에 `use_namespace:"True"` 추가 |
| `PackageNotFoundError: unitree_go2_description` | `colcon --packages-up-to`가 Go2 의존성 못 물어옴 | 전체 빌드 또는 `--packages-select` 명시 (§7) |
| `launch configuration 'generate' does not exist` | Clearpath spawn을 `TimerAction`으로 감싸 launch context 수명 문제 | spawn을 최상위에서 직접 include (반영됨) |

**경고이지만 무시해도 되는 것들** (spawn/렌더 정상이면 차단 오류 아님):
`gz_frame_id`/`frame_id`/`noise` "not defined in SDF", `libEGL warning`.

**빌드/설치 확인 팁**: `--symlink-install`이라 launch·yaml 수정은 재빌드 없이 즉시 반영된다.
설치본을 찾을 땐 심볼릭 링크를 따라가도록 `find -L install/... `를 쓴다.

---

## 14. 부록 — 출처·버전 정리

| 구성요소 | 출처 | 버전/커밋 |
| --- | --- | --- |
| ROS 2 | apt (ros.org) | Jazzy Jalisco |
| Gazebo | `ros-jazzy-ros-gz` 벤더 | Harmonic / gz-sim 8.x |
| 4륜 A300 | apt `ros-jazzy-clearpath-simulator` | Jazzy |
| 4족 Go2 + CHAMP | github [RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy) | `edfb18772c12a159915770d057f0c82d3bb30e16` (deps.repos 고정) |
| 드론 X3 | Gazebo 공식 멀티콥터 예제(X3 UAV) 기반 → `agconav_description/models/agconav_drone` 로 로컬화 | 3배 스케일·15kg 커스텀 |

**한 줄 재현 요약** (환경 이미 갖춰졌을 때):

```bash
git clone <URL> AG-CoNav && cd AG-CoNav
vcs import src < deps.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
ros2 launch agconav_bringup agconav_sim.launch.py
```
