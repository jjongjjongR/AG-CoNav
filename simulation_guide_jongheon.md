# AG-CoNav 통합 시뮬레이션 재현 가이드

이 문서는 Ubuntu 24.04가 설치된 다른 PC에서 저장소를 내려받아 X3 드론,
Husky A300, Unitree Go2가 하나의 Gazebo 월드에 나타나는 상태까지 재현하는
방법을 설명한다.

처음 실행하는 사람은 **4장의 자동 설치 방법만 따라 하면 된다.**
2장과 3장은 프로젝트가 어떻게 구현됐는지 이해하거나 보고서에 설명할 때 참고한다.

---

## 1. 최종적으로 실행되는 것

통합 launch를 한 번 실행하면 Gazebo Harmonic 월드 하나에 로봇 세 대가 생성된다.

```text
agconav_world
├── X3          드론
├── wheel/robot Husky A300
└── leg         Unitree Go2
```

동시에 다음 ROS 2 연동이 실행된다.

- Gazebo 시뮬레이션 시간 `/clock`
- X3 명령 `/drone/cmd_vel`
- Go2 관절 상태 `/joint_states`와 controller
- A300 주행 상태 `/wheel/platform/odom`과 controller
- 세 로봇의 LiDAR, IMU, GPS 브리지
- Wheel과 Leg의 TF prefix relay

Nav2 namespace 구조는 임시 parameter와 map으로 기동을 시험했지만, 시험 파일은
검증 후 삭제했다. 따라서 현재 기본값은 `use_nav2:=false`이며 실제 Nav2 주행까지
완성됐다는 의미는 아니다.

---

## 2. 로봇 3종을 통합한 방법

### 2.1 핵심 개념

세 로봇은 처음부터 제공되는 형식이 서로 다르다. 하나의 형식으로 억지로 변환하지
않고 각 로봇의 기존 생성·제어 방식을 유지하면서, Gazebo만 한 번 실행하도록 합쳤다.

| 로봇 | 처음 받은 형태 | 통합 방법 |
| --- | --- | --- |
| X3 드론 | Gazebo 멀티콥터 SDF 예제 | X3 모델을 프로젝트에 로컬화하고 X3 비행 플러그인만 통합 월드에 배치 |
| Husky A300 | Clearpath 패키지와 `robot.yaml` | generator로 모델을 만든 뒤 실행 중인 공통 월드에 spawn |
| Unitree Go2 | URDF/Xacro, ros2_control, CHAMP 패키지 | Go2 전용 spawn launch에서 자체 Gazebo 실행을 제외하고 공통 월드에 spawn |

처음 보는 용어는 다음처럼 이해하면 된다.

- **SDF**: Gazebo의 월드, 모델, 물리 플러그인을 설명하는 파일
- **URDF/Xacro**: ROS에서 로봇의 링크와 관절 구조를 설명하는 파일
- **`robot.yaml`**: Clearpath generator에 A300 구성을 전달하는 설정 파일
- **Spawn**: 이미 실행 중인 Gazebo 월드에 로봇을 생성하는 것
- **Bridge**: Gazebo 메시지와 ROS 2 메시지를 서로 전달·변환하는 노드
- **Controller**: 바퀴 또는 관절에 실제 명령을 전달하는 ROS 2 control 구성요소

### 2.2 X3 드론

#### 출처와 변경 방식

Gazebo Harmonic의 `multicopter_velocity_control.sdf` 예제를 기반으로 했다. 원본
예제에는 X3와 X4가 함께 있지만, 프로젝트에서는 X3 모델과 X3 비행 플러그인만
가져왔다.

```text
Gazebo 멀티콥터 예제
├── X3 모델과 비행 플러그인 → 사용
└── X4 모델과 플러그인      → 제외
```

현재 X3 모델은 프로젝트 내부에 복사된 뒤 크기, 질량, 색상과 센서가
커스터마이징되어 있다.

- 모델: `src/agconav_description/models/agconav_drone/model.sdf`
- 월드 배치와 비행 플러그인: `src/agconav_worlds/worlds/agconav_integrated.sdf`

통합 월드는 로컬 모델을 `model://agconav_drone`으로 불러와 Gazebo 이름을 `X3`로
지정한다. 네 개의 motor plugin과 `MulticopterVelocityControl` plugin도 같은
SDF에 배치한다.

#### 명령 브리지

Gazebo 플러그인이 원래 사용하는 명령 토픽은 다음과 같다.

```text
/X3/gazebo/command/twist
```

`agconav_sim.launch.py`의 `drone_cmd_vel_bridge`가 이를 ROS 2의
`/drone/cmd_vel`로 연결한다.

```text
ROS 2 /drone/cmd_vel
          ↓ ros_gz_bridge
Gazebo /X3/gazebo/command/twist
```

### 2.3 Husky A300

#### 출처와 생성 방식

A300은 `ros-jazzy-clearpath-simulator` 패키지를 통해 설치한다. 이 패키지가
`clearpath_description`, `clearpath_config`, `clearpath_generator_gz`,
`clearpath_gz` 등 필요한 패키지를 함께 제공한다.

A300은 완성된 URDF를 프로젝트에서 직접 편집하지 않는다. 다음 설정을 읽은
Clearpath generator가 URDF, launch, controller parameter를 생성한다.

```text
config/clearpath_a300/robot.yaml
             ↓
Clearpath generator
             ↓
A300 URDF·launch·controller 설정
             ↓
현재 agconav_world에 spawn
```

`robot.yaml`에는 `/wheel` namespace, A300 attachment, LiDAR, IMU, GPS 구성이
들어 있다.

#### 공통 월드에 넣는 방법

Clearpath의 전체 simulation launch로 별도 Gazebo를 실행하지 않는다.
`agconav_sim.launch.py`가 다음 spawn launch만 include한다.

```text
clearpath_gz/launch/robot_spawn.launch.py
```

이때 `world:=agconav_world`, `generate:=true`를 전달한다. 결과는 다음과 같다.

```text
Gazebo 모델 : wheel/robot
ROS namespace: /wheel
주요 토픽   : /wheel/cmd_vel
              /wheel/platform/odom
              /wheel/tf
controller   : /wheel/controller_manager
```

#### PC마다 Clearpath 설치 위치가 다른 문제

Clearpath 설치 경로를 `/home/누구/...` 또는 `/opt/ros/...`로 고정하지 않는다.

- `clearpath_gz` 위치는 `get_package_share_directory("clearpath_gz")`로 찾는다.
- 프로젝트의 `robot.yaml`은 빌드 시
  `share/agconav_bringup/config/clearpath_a300/robot.yaml`에 설치한다.
- 통합 launch는 `get_package_share_directory("agconav_bringup")`로 그 파일을 찾는다.

따라서 사용자명, clone 위치, Clearpath의 실제 설치 prefix가 달라도 같은 실행 명령을
사용할 수 있다.

### 2.4 Unitree Go2

#### 출처와 패키지 구성

`RobInLabUJI/unitree_go2_ros2_jazzy` 저장소를 사용한다. 팀원이 서로 다른 버전을
받지 않도록 `deps.repos`에 commit
`edfb18772c12a159915770d057f0c82d3bb30e16`을 고정했다.

```text
unitree_go2_description → 몸체, 링크, 다리 URDF/Xacro와 mesh
unitree_go2_sim         → Gazebo 및 ros2_control 설정
champ_base              → 4족 보행 제어
champ_msgs              → CHAMP 메시지
```

#### 공통 월드에 넣는 방법

원본 launch에는 Go2 전용 Gazebo 실행이 포함되어 있다. 프로젝트의
`src/agconav_bringup/launch/go2_spawn.launch.py`는 Gazebo를 새로 실행하지 않고
다음 기능만 수행한다.

```text
Go2 원본 실행 구조
├── 별도 Gazebo 실행          → 제외
├── Go2 robot_description 생성 → 유지
├── 현재 월드에 spawn          → 유지
├── CHAMP 보행 제어            → 유지
└── ros2_control               → 유지
```

최종 인터페이스는 다음과 같다.

```text
Gazebo 모델      : leg
관절 상태        : /joint_states
이동 명령        : /cmd_vel
controller manager: /controller_manager
controllers      : joint_states_controller
                   joint_group_effort_controller
```

#### 센서 Overlay와 control patch의 차이

Go2 센서는 외부 모델에 센서 코드를 직접 섞지 않고
`leg_with_sensors.urdf.xacro`가 원본 Xacro를 include한 뒤 센서 Xacro를 덧붙이는
Overlay 방식이다.

반면 생성 직후 넘어지는 문제를 해결하려면 외부 Go2 Xacro의 ros2_control 초기값과
controller YAML 전달 부분을 바꿔야 했다. 이 변경은 작업자의 로컬 수정으로 남기지
않고 `patches/unitree_go2_ros2_jazzy.patch`에 저장했다. 자동 설치 스크립트가
고정 commit을 받은 뒤 이 patch를 적용한다.

따라서 정확한 설명은 다음과 같다.

- 센서 장착: 외부 모델을 감싸는 Xacro Overlay
- 초기 자세와 controller 경로: 고정 upstream commit에 추적 가능한 patch 적용

Go2 controller 설정은 `src/agconav_bringup/config/leg_controllers.yaml`에 있으며,
실제 12개 관절, effort 명령 인터페이스, position/velocity 상태 인터페이스와 PID를
정의한다.

---

## 3. 최종 통합 구조와 담당 파일

각 로봇의 원래 전체 launch를 실행하면 Gazebo가 여러 개 생긴다. 프로젝트에서는
Gazebo를 한 번만 실행하고 A300과 Go2를 그 월드에 추가한다.

```text
agconav_sim.launch.py
│
├── agconav_integrated.sdf로 Gazebo 1개 실행
│   └── X3와 환경 모델 포함
├── Clearpath robot_spawn.launch.py include
│   └── A300을 agconav_world에 spawn
├── go2_spawn.launch.py include
│   └── Go2를 agconav_world에 spawn
├── /clock과 X3 cmd_vel bridge
├── 세 로봇 sensor bridge
└── Wheel·Leg TF prefix relay
```

| 파일 | 역할 |
| --- | --- |
| `src/agconav_worlds/worlds/agconav_integrated.sdf` | 공통 월드, 환경, X3 배치와 비행 plugin |
| `src/agconav_description/models/agconav_drone/model.sdf` | 커스터마이징한 X3 몸체와 센서 |
| `config/clearpath_a300/robot.yaml` | A300 platform, namespace, attachment와 센서 구성 |
| `src/agconav_bringup/launch/agconav_sim.launch.py` | 전체 월드와 세 로봇, bridge를 묶는 최상위 launch |
| `src/agconav_bringup/launch/go2_spawn.launch.py` | 기존 Gazebo 없이 Go2 설명·CHAMP·controller·spawn 실행 |
| `src/agconav_bringup/config/leg_controllers.yaml` | Go2 12관절 controller와 PID 설정 |
| `src/agconav_gz_bridge/launch/sensor_bridge.launch.py` | 세 로봇 센서를 공통 ROS 2 토픽으로 bridge |
| `patches/unitree_go2_ros2_jazzy.patch` | Go2 초기 관절값과 controller 파일 전달 변경 |

---

## 4. 다른 PC에서 가장 쉽게 설치하기

### 4.1 준비할 것

- Ubuntu Desktop 24.04 LTS
- 인터넷 연결
- `sudo`를 사용할 수 있는 계정
- Gazebo GUI를 실행할 수 있는 그래픽 드라이버

ROS 2나 Gazebo를 미리 설치할 필요는 없다. 자동 설정 스크립트가 설치한다.

### 4.2 저장소 받기

터미널을 열고 다음 명령을 그대로 실행한다.

```bash
sudo apt update
sudo apt install -y git
git clone https://github.com/jjongjjongR/AG-CoNav.git
cd AG-CoNav
```

### 4.3 자동 설정

```bash
./scripts/setup_simulation.sh
```

중간에 시스템 패키지 설치를 위한 `sudo` 비밀번호를 물어볼 수 있다. 다운로드와
전체 빌드가 있으므로 PC와 네트워크에 따라 시간이 걸린다.

스크립트가 자동으로 수행하는 작업은 다음과 같다.

1. Ubuntu 24.04인지 확인
2. ROS 2 apt 저장소 등록
3. ROS 2 Jazzy, Gazebo, Clearpath, Nav2와 빌드 도구 설치
4. `deps.repos`의 고정 commit으로 Go2와 CHAMP 다운로드
5. Go2 초기 자세와 controller patch 적용
6. rosdep 의존성 설치
7. 환경 모델 5종과 X3 UAV를 Gazebo Fuel cache에 다운로드
8. 전체 workspace 빌드
9. ROS package, Clearpath YAML, Go2 patch, launch와 OpenGL 검증

모든 과정이 끝나면 다음 메시지가 나온다.

```text
[8/8] 완료
설정과 빌드 검증이 끝났습니다.
```

도중에 실패하면 마지막 `[ERROR]` 문장을 먼저 읽는다. 문제를 해결한 뒤 같은
`./scripts/setup_simulation.sh`를 다시 실행해도 된다. 이미 설치·다운로드·적용된
항목은 재사용한다.

### 4.4 시뮬레이션 실행

```bash
./scripts/run_simulation.sh
```

이 스크립트가 ROS 2와 현재 workspace를 자동으로 source하고 다음 환경도 설정한다.

```text
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

따라서 새 터미널마다 `source install/setup.bash`를 직접 입력할 필요가 없다.
Gazebo를 종료할 때는 실행한 터미널에서 `Ctrl+C`를 누른다.

---

## 5. 정상 실행 확인

Gazebo가 실행된 상태에서 새 터미널을 하나 더 연다. 확인 명령은 새 터미널에서도
프로젝트 실행 스크립트와 같은 ROS 환경이 필요하므로 먼저 다음을 입력한다.

```bash
cd AG-CoNav
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### 5.1 로봇 세 대가 같은 월드에 있는지

```bash
gz model --list
```

목록에서 최소한 다음 이름을 확인한다.

```text
X3
wheel/robot
leg
```

### 5.2 핵심 ROS 2 메시지가 수신되는지

각 명령은 메시지 하나를 받으면 자동으로 끝난다.

```bash
ros2 topic echo /clock --once
ros2 topic echo /joint_states --once
ros2 topic echo /wheel/platform/odom --once
```

- `/clock`: Gazebo 시간이 ROS 2로 전달됨
- `/joint_states`: Go2의 12개 관절 상태가 발행됨
- `/wheel/platform/odom`: A300 주행 상태가 발행됨

### 5.3 드론 명령 bridge가 연결됐는지

```bash
ros2 topic info /drone/cmd_vel -v
```

`Subscription count`가 1 이상이고 `drone_cmd_vel_bridge`가 보이면 ROS 명령을
Gazebo X3로 전달할 bridge가 연결된 것이다.

### 5.4 Go2와 A300 controller가 active인지

```bash
ros2 control list_controllers -c /controller_manager
ros2 control list_controllers -c /wheel/controller_manager
```

Go2에서는 다음 두 controller가 `active`여야 한다.

```text
joint_states_controller             active
joint_group_effort_controller       active
```

A300에서는 다음 controller가 `active`여야 한다.

```text
joint_state_broadcaster             active
platform_velocity_controller        active
```

### 5.5 센서 토픽 확인

```bash
ros2 topic list -t | grep -E '^/(drone|wheel|leg)/(points|imu|gps)'
```

공통 센서 계약은 다음과 같다. `X`는 `drone`, `wheel`, `leg` 중 하나다.

| 토픽 | ROS 2 메시지 타입 |
| --- | --- |
| `/X/points` | `sensor_msgs/msg/PointCloud2` |
| `/X/imu` | `sensor_msgs/msg/Imu` |
| `/X/gps` | `sensor_msgs/msg/NavSatFix` |

실제 값 한 개를 확인하려면 다음처럼 실행한다.

```bash
ros2 topic echo /drone/points --once
ros2 topic echo /wheel/imu --once
ros2 topic echo /leg/gps --once
```

### 5.6 TF prefix 확인

Wheel과 Leg 내부 프레임은 둘 다 원래 `odom`, `base_link` 같은 이름을 사용한다.
내부 TF를 `/wheel/tf`, `/leg/tf`로 격리한 뒤 relay가 전역 `/tf`에 `wheel/`,
`leg/` prefix를 붙인다.

```bash
ros2 run tf2_ros tf2_echo leg/base_link leg/lf_foot_link \
  --ros-args -p use_sim_time:=true
```

변환값이 계속 출력되면 Go2 TF가 연결된 것이다.

---

## 6. 로봇을 움직여 보기

### 6.1 X3 상승

```bash
ros2 topic pub -r 10 /drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {z: 0.0}}"
```

명령을 멈추려면 `Ctrl+C`를 누른다.

### 6.2 A300 전진

A300은 `Twist`가 아니라 `TwistStamped`를 사용한다.

```bash
ros2 topic pub -r 10 /wheel/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.3}, angular: {z: 0.0}}}"
```

### 6.3 Go2 조작

현재 Go2 이동 명령 토픽은 `/cmd_vel`이다.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 7. 자주 발생하는 오류

### `robot.yaml could not be found`

예전 launch가 설치된 launch 파일의 `__file__`에서 저장소 위치를 역산하면서 발생한
문제다. 현재 코드는 package share를 사용하므로 먼저 최신 commit인지 확인하고 다시
설정한다.

```bash
git pull
./scripts/setup_simulation.sh
```

설치된 파일을 직접 확인할 수도 있다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
test -f "$(ros2 pkg prefix agconav_bringup)/share/agconav_bringup/config/clearpath_a300/robot.yaml" \
  && echo OK
```

### `PackageNotFoundError: unitree_go2_description`

Go2 다운로드 또는 전체 빌드가 끝나지 않은 상태다.

```bash
./scripts/setup_simulation.sh
```

### Go2가 생성되자마자 넘어짐

Go2 patch 또는 `leg_controllers.yaml`이 설치되지 않았을 가능성이 크다. 자동 설정을
다시 실행한다. 실행 스크립트도 설치된 Go2 Xacro에 patch 내용이 있는지 검사한다.

```bash
./scripts/setup_simulation.sh
./scripts/run_simulation.sh
```

### `RTPS_TRANSPORT_SHM ... open_and_lock_file failed`

이전 시뮬레이션 또는 ROS 2 프로세스가 남은 상태에서 다시 실행하면 발생할 수 있다.
기존 Gazebo와 ROS launch 터미널을 `Ctrl+C`로 종료한 뒤 새 터미널에서 다시 실행한다.
무조건 `/dev/shm` 파일을 삭제하는 방식은 사용하지 않는다.

### Fuel 모델 다운로드 실패

건물, 나무 또는 X3 mesh를 내려받지 못한 것이다. 인터넷과 DNS 연결을 확인한 뒤
설정 스크립트를 다시 실행한다.

```bash
./scripts/setup_simulation.sh
```

### OpenGL 검증 실패 또는 Gazebo 화면이 검게 나옴

그래픽 드라이버 상태를 확인한다.

```bash
glxinfo -B
```

드라이버를 새로 설치했다면 PC를 재부팅한 뒤 다시 실행한다. GPU 드라이버는 하드웨어별로
다르므로 프로젝트 스크립트가 임의로 설치하지 않는다.

### Nav2를 켰더니 실패함

현재 저장소에는 시험 후 삭제한 Nav2 parameter와 map이 없다. 기본 실행처럼
`use_nav2:=false`를 사용해야 한다. 실제 Nav2 실행은 설치 가능한 parameter, map,
위치추정 TF가 준비된 이후 단계다.

---

## 8. 보고서에 사용할 수 있는 검증 문장

> Gazebo 모델 목록에서 X3 드론, Go2, A300이 하나의 `agconav_world`에 동시에
> 생성된 것을 확인하였다. ROS 2에서는 `/clock`, Go2의 `/joint_states`, A300의
> `/wheel/platform/odom` 메시지를 실제로 수신하였다. 또한 드론 명령 토픽
> `/drone/cmd_vel`에 Gazebo bridge가 연결되었으며, Go2와 A300의 controller가
> `active` 상태임을 확인하였다. 따라서 세 로봇의 모델 로드와 Gazebo–ROS 2 기본
> 연동이 정상적으로 수행되었다.

구현 방법은 다음처럼 설명할 수 있다.

> 각 로봇은 서로 다른 형태로 제공되었다. X3는 Gazebo SDF 예제를 기반으로 프로젝트에
> 로컬화한 모델, A300은 Clearpath의 `robot.yaml` 기반 자동 생성 방식, Go2는
> URDF/Xacro와 CHAMP 제어 패키지 형태였다. 각 로봇의 기존 모델과 제어 구조를
> 유지하되 개별 Gazebo 실행 부분은 사용하지 않았다. 하나의 `agconav_world`를
> 실행하고 A300과 Go2를 해당 월드에 동적으로 spawn함으로써 X3, A300, Go2가
> 하나의 Gazebo Harmonic 환경에서 동시에 실행되도록 구성하였다.

---

## 9. 한 번에 보는 실행 명령

새 PC에서 최초 한 번:

```bash
sudo apt update
sudo apt install -y git
git clone https://github.com/jjongjjongR/AG-CoNav.git
cd AG-CoNav
./scripts/setup_simulation.sh
```

설치가 끝난 뒤 실행할 때마다:

```bash
cd AG-CoNav
./scripts/run_simulation.sh
```
