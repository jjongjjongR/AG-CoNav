import math
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import Node, SetRemap

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    clearpath_setup_path = LaunchConfiguration("clearpath_setup_path")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_localization = LaunchConfiguration("use_localization")
    sequential_pipeline = LaunchConfiguration("sequential_pipeline")
    spawn_ground_early = LaunchConfiguration("spawn_ground_early")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    headless = LaunchConfiguration("headless")

    # 설치된 ROS2 패키지의 공유 디렉터리
    agconav_worlds_share = get_package_share_directory("agconav_worlds")
    agconav_bringup_share = get_package_share_directory("agconav_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    clearpath_gz_share = get_package_share_directory("clearpath_gz")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # 실행할 공용 Gazebo 월드 (성동구 실지형 heightmap).
    # world_file 인자로 다른 월드를 지정할 수 있다. 종단 검증에 쓴 것은
    # agconav_worlds/worlds/Seongdong_gu_aligned (축에 맞춰 회전시킨 전체 맵)로,
    # agconav_all.launch.py 가 그걸 기본값으로 쓴다. 빈 값이면 여기 기본값.
    # <world name>은 그대로 "Seongdong_gu"를 유지해야 clearpath spawn의
    # world:=Seongdong_gu 인자와 set_pose 서비스 경로가 그대로 맞는다.
    default_world_path = os.path.join(
        agconav_worlds_share,
        "worlds",
        "Seongdong_gu",
        "Seongdong_gu.world",
    )
    world_path = PythonExpression(
        ["'", LaunchConfiguration("world_file"), "' or '", default_world_path, "'"]
    )

    # 각 하위 launch 파일 경로
    gz_sim_launch_path = os.path.join(
        ros_gz_sim_share,
        "launch",
        "gz_sim.launch.py",
    )

    clearpath_spawn_launch_path = os.path.join(
        clearpath_gz_share,
        "launch",
        "robot_spawn.launch.py",
    )

    # leg 보행 컨트롤러. 기본은 **Unitree 공식 unitree_guide** 이식본이다.
    #
    # 근거(단계식 5~30도 경사로, 각 3회 실측):
    #   unitree_guide   20도까지 통과, 25도에서 전복
    #   RL robot_lab    15도까지 통과, 20도에서 정지
    #   RL legged_gym / himloco  평지에서도 못 걷는다
    #   CHAMP           경사에서 사실상 제자리걸음(문서 11)
    # 등판이 한 단계 높아 guide 로 정했다. 다만 한계에서의 실패 양상은 RL 이
    # 낫다 — RL 은 자세를 유지한 채 멈추고(이탈 0.13~0.62 m) guide 는 전복한다
    # (이탈 1.51~3.54 m). 그래서 주행성 지도의 경사 상한을 20도로 잡아 애초에
    # 25도 구간에 들어가지 않게 하는 것이 전제다(traversability_leg.yaml).
    #
    # !! 문서 11 의 "unitree_guide 전 구간 전복" 은 근거를 잃었다 !!
    # 그 측정은 관절 초기 자세 시딩이 깨진 상태에서 이뤄졌다. 시딩을 고치고
    # 다시 재니 정상 보행한다.
    # leg_controller:=guide / champ 로 바꿀 수 있다(비교·회귀 확인용).
    # !! 다만 guide 는 Nav2 종단 주행에서 전복한다. docs/14 참고. !!
    go2_spawn_launch_path = os.path.join(
        agconav_bringup_share,
        "launch",
        "go2_rl_spawn.launch.py",
    )
    go2_guide_spawn_launch_path = os.path.join(
        agconav_bringup_share,
        "launch",
        "go2_guide_spawn.launch.py",
    )
    go2_champ_spawn_launch_path = os.path.join(
        agconav_bringup_share,
        "launch",
        "go2_spawn.launch.py",
    )

    # 센서 브리지: gz 센서 토픽을 모듈 계약 이름(/X/points, /X/imu, /X/gps)으로 정합
    sensor_bridge_launch_path = os.path.join(
        get_package_share_directory("agconav_gz_bridge"),
        "launch",
        "sensor_bridge.launch.py",
    )

    declare_world_file = DeclareLaunchArgument(
        "world_file",
        default_value="",
        description="띄울 .world 절대경로. 비우면 agconav_worlds의 Seongdong_gu.",
    )
    declare_model_path = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="GZ_SIM_RESOURCE_PATH 앞에 덧붙일 모델 폴더. world_file이 "
        "기본 월드에 없는 model://을 참조할 때 필요하다.",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation time",
    )

    # 스폰 위치는 launch 인자로 덮어쓸 수 있다. 파일을 고치지 않고
    #   ros2 launch agconav_bringup agconav_sim.launch.py wheel_x:=-160 wheel_y:=148
    # 처럼 넘기면 된다. 원점 일대는 건물이 덮고 있어 로봇이 파묻히므로
    # 기본값은 개활지 (-159, 149) 기준 배치다.
    # 이 값들은 scripts/capture_poses.py --apply 로 갱신한다 (GUI에서 옮긴 뒤 실행).
    # 드론(X3)은 월드 파일의 <include>에 있어 여기서는 다루지 않는다.
    spawn_defaults = (
        ("wheel_x", "-159.6820", "A300(wheel) 스폰 x [m]"),
        ("wheel_y", "147.5380", "A300(wheel) 스폰 y [m]"),
        ("wheel_z", "6.1500", "A300(wheel) 스폰 z [m] — 지면 5.90 + 0.25"),
        ("wheel_yaw", "-0.4349", "A300(wheel) 스폰 heading [rad]"),
        ("leg_x", "-158.8710", "Go2(leg) 스폰 x [m]"),
        ("leg_y", "150.8310", "Go2(leg) 스폰 y [m]"),
        ("leg_z", "6.2500", "Go2(leg) 스폰 z [m] — 지면 5.85 + 0.40 (RL 원본 스폰 높이)"),
        ("leg_yaw", "-0.4613", "Go2(leg) 스폰 heading [rad]"),
        # leg 보행 컨트롤러 선택. 기본 guide = Unitree 공식 unitree_guide
        # 이식본. rl 은 학습 정책(rl_quadruped_controller), champ 는 옛 스택.
        ("leg_controller", "rl", "leg 보행 컨트롤러 (rl | guide | champ)"),
        # RL 정책 폴더. 7번 실험에서 robot_lab 만 실제로 걷고 등판했다
        # (legged_gym·himloco 는 평지에서도 전복).
        ("leg_policy", "robot_lab", "RL 정책 폴더 (robot_lab | legged_gym | himloco)"),
        # 7번 실험 §6 의 안정 최대. 1.5 를 주면 오히려 느려진다(0.697 m/s).
        # 실험 확정값. 명령 1.0 -> 실제 1.045 m/s (달성률 105%, 최대 기울기 18.7°).
        # 1.5 를 주면 오히려 0.697 m/s 로 느려진다 — 정책이 학습된 명령 범위를
        # 벗어나면 걸음이 무너진다. 근거: docs/11 §6.
        ("leg_max_speed", "1.0", "leg 명령 속도 상한 [m/s] — 실측 안정 최대"),
    )
    declare_spawn_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in spawn_defaults
    ]
    nav2_launch_path = os.path.join(
        nav2_bringup_share,
        "launch",
        "bringup_launch.py",
    )
    localization_launch_path = os.path.join(
        get_package_share_directory("agconav_localization"),
        "launch",
        "localization.launch.py",
    )

    # 소스/설치 경로를 역산하지 않는다. 일반 colcon build에서는 launch 파일이
    # install 아래로 복사되므로 __file__ 기준 "저장소 루트" 계산은 잘못된
    # install/agconav_bringup 경로를 만들 수 있다. robot.yaml은 CMake가 패키지
    # share/config/clearpath_a300에 설치하며, 어느 PC에서도 ament index로 찾는다.
    declare_clearpath_setup_path = DeclareLaunchArgument(
        "clearpath_setup_path",
        default_value=os.path.join(
            agconav_bringup_share,
            "config",
            "clearpath_a300",
        ),
        description="Directory containing the A300 robot.yaml",
    )

    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Gazebo GUI 없이 서버만 실행(-s). 장시간 자동 검증용.",
    )

    declare_use_nav2 = DeclareLaunchArgument(
        "use_nav2",
        default_value="true",
        description="Launch Nav2 for wheel/leg (needs /X/nav_map and localization TF)"
    )
    declare_sequential_pipeline = DeclareLaunchArgument(
        "sequential_pipeline",
        default_value="false",
        description=(
            "A→F 통합 실행에서는 Module F 완료 전까지 지상 로봇·B·C를 "
            "시작하지 않아 CPU 경합을 막는다."),
    )
    declare_spawn_ground_early = DeclareLaunchArgument(
        "spawn_ground_early",
        default_value="false",
        description=(
            "지상 로봇 물리 모델과 센서는 A 단계부터 월드에 올리되, "
            "Module B/C와 Nav2는 sequential_pipeline 단계 게이트 뒤에 시작한다."),
    )

    declare_use_localization = DeclareLaunchArgument(
        "use_localization",
        default_value="true",
        description="Launch EKF and Navsat nodes for Module B",
    )

    declare_nav2_params_file = DeclareLaunchArgument(
        "nav2_params_file",
        # 최종 Nav2 설정은 모듈 C가 아직 제공하지 않았다. 상대적인 소스 트리
        # 경로를 만들지 않고, 사용 시 명시적으로 전달하거나 향후 패키지에 설치한다.
        default_value=os.path.join(
            agconav_bringup_share,
            "config",
            "nav2_common.yaml",
        ),
        description="Common Nav2 params for both ground robots (module C; required when use_nav2=true)",
    )

    # !! libtorch 경로는 Gazebo 프로세스에 들어가야 한다 !!
    # rl_quadruped_controller 는 libtorch(C++)를 링크하는데, 그 컨트롤러를 여는
    # controller_manager 는 **Gazebo 프로세스 안**에서 돈다(gz_quadruped_hardware
    # 플러그인). 그래서 leg 스폰 그룹에만 환경변수를 걸면 소용이 없다.
    # 실제 증상:
    #   dlopen error: libc10.so: cannot open shared object file
    #   -> Failed loading controller rl_quadruped_controller
    # 최상위에서 설정해 Gazebo 를 포함한 모든 하위 프로세스가 물려받게 한다.
    _torch_lib = os.path.join(os.path.expanduser("~"), "libtorch", "lib")
    _ld = os.environ.get("LD_LIBRARY_PATH", "")
    set_torch_lib_path = SetEnvironmentVariable(
        "LD_LIBRARY_PATH", _torch_lib + (":" + _ld if _ld else ""))

    # Gazebo와 공용 월드는 여기서 한 번만 실행한다.
    # 로컬 모델(agconav_drone) 탐색 경로를 Gazebo에 알려준다.
    # ROS 2 Jazzy는 Gazebo Harmonic(gz-sim8)과 페어링되며, gz sim은
    # classic 전용 GAZEBO_MODEL_PATH가 아니라 GZ_SIM_RESOURCE_PATH를 읽는다.
    # 기존 값(예: /opt/ros/jazzy/share)을 유지하려고 앞에 우리 models 경로만 덧붙인다.
    # Seongdong_gu 월드는 agconav_worlds/models의 bump_*·ramp_*를 model://로
    # 참조하므로 그 경로도 함께 넣어야 한다. 빠지면 월드 로드가 실패한다.
    _gz_models_dirs = [
        os.path.join(get_package_share_directory("agconav_description"), "models"),
        os.path.join(agconav_worlds_share, "models"),
    ]
    _existing_gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if _existing_gz_resource_path:
        _gz_models_dirs.append(_existing_gz_resource_path)
    # world_file로 다른 월드를 띄울 때, 그 월드가 참조하는 모델이 위 두 경로에
    # 없을 수 있다. 경로에 없으면 Gazebo가 월드 로드 자체를 실패하고
    #   [Err] Error Code 14 ... Unable to find uri[model://<이름>]
    # 가 뜬 뒤, 월드가 안 떠서 wait_for_world가 죽고 모듈 A~F가 전부 연쇄로 죽는다.
    # 기본 월드와 정렬 월드가 쓰는 모델(agconav_drone, ramp_*, bump_*)은 위 두
    # 경로에 다 있으므로 model_path 없이도 뜬다.
    # world_file은 LaunchConfiguration이라 여기서 경로를 유추할 수 없으므로
    # 부르는 쪽이 model_path로 알려준다.
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[LaunchConfiguration("model_path"), os.pathsep,
               os.pathsep.join(_gz_models_dirs)],
    )

    # headless:=true 면 gz 서버만 띄우고 GUI는 생략한다(-s).
    # 성동구 월드에서 GUI 렌더가 CPU 1코어 가까이 쓰고 GPU도 센서 렌더와
    # 경합해서, 장시간 자동 검증(예: 드론 34분 경로비행)에서는 이걸 끄는 쪽이
    # 실시간계수(RTF)에 크게 유리하다. 사람이 볼 때 쓰는 기본값은 false다.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            "gz_args": [world_path, " -r"],
        }.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            "gz_args": [world_path, " -r -s"],
        }.items(),
        condition=IfCondition(headless),
    )

    # /clock은 중앙에서 한 번만 Gazebo → ROS2로 변환한다.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="agconav_clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    # (제거됨) wheel_clock_bridge — /clock을 /wheel/clock으로 한 번 더 중계하던 노드.
    # ROS 2에서 use_sim_time은 네임스페이스와 무관하게 항상 절대경로 /clock을
    # 구독한다. PushRosNamespace("wheel") 아래 있어도 마찬가지다. 실측: 실행 중인
    # 스택에서 /wheel/clock의 구독자는 0개였다(/clock은 23개 이상).
    # 그런데 Gazebo의 /clock은 초당 881건 발행된다. 아무도 안 듣는 토픽에 초당
    # 881건을 직렬화해 내보내느라 CPU 약 11%를 쓰고 있었다. 6코어 머신에서
    # 전체가 976%/1200%로 포화된 상황이라 그냥 버리는 몫이 아니었다.

    # X3의 Gazebo Twist 명령을 ROS2 /drone/cmd_vel로 연결한다.
    drone_cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="drone_cmd_vel_bridge",
        output="screen",
        arguments=[
            "/X3/gazebo/command/twist"
            "@geometry_msgs/msg/Twist]"
            "gz.msgs.Twist",
        ],
        remappings=[
            (
                "/X3/gazebo/command/twist",
                "/drone/cmd_vel",
            )
        ],
    )

    # 드론 TF. wheel/leg는 로봇 스택이 TF를 발행하지만 드론은 gz 모델뿐이라
    # 아무도 발행하지 않아 전역 /tf에 drone/* 프레임이 없었다(모듈 A가 점군을
    # map으로 변환 불가). 월드의 OdometryPublisher를 map -> drone/base_link로
    # 설정하고, 그 pose를 /tf로 브리지한다.
    drone_tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="drone_tf_bridge",
        output="screen",
        arguments=[
            "/model/X3/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[
            ("/model/X3/pose", "/tf"),
        ],
    )

    # wheel 센서 프레임 별칭.
    # clearpath는 시스템 xacro에서 gz_frame_id를 ${name}_sensor_link로 고정해
    # 접두어를 못 붙인다(/wheel/imu의 frame_id = imu_0_link). 반면 전역 /tf에는
    # tf_prefix_relay가 wheel/ 을 붙인 이름만 있어, EKF·navsat이 센서 메시지의
    # frame을 조회하지 못하고 IMU/GPS 입력을 통째로 버린다.
    # 접두어 있는 프레임에 같은 위치의 별칭을 달아 조회가 되게 한다.
    # 이 이름들은 로봇 간 고유해서(leg는 os1_lidar/base_link/gps_link) 충돌하지 않는다.
    wheel_frame_alias = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"wheel_{child}_alias_tf",
            output="log",
            arguments=[
                "--frame-id", f"wheel/{child}",
                "--child-frame-id", child,
            ],
        )
        for child in ("imu_0_link", "gps_0_link", "lidar3d_0_sensor_link")
    ]

    # 드론 센서는 base_link에 강체 고정이므로 정적 TF로 잇는다.
    # 값은 models/agconav_drone/model.sdf의 링크 pose와 동일하다.
    #
    # LiDAR(drone/base_link -> drone/os1_lidar)는 여기서 발행하지 않는다.
    # 모듈 A의 drone_sim_test.launch.py가 drone_os1_lidar_static_tf로 이미
    # 같은 값을 내보내며, 드론 소유는 모듈 A다(중복 발행 방지).
    # 따라서 이 시뮬만 단독 실행하면 drone/os1_lidar 프레임이 없다 —
    # 드론 점군을 map으로 변환하려면 모듈 A를 함께 띄워야 한다
    # (agconav_all.launch.py는 항상 함께 띄운다).
    # GPS는 모듈 A가 발행하지 않으므로 여기서 유지한다.
    drone_sensor_tf = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"drone_{name}_static_tf",
            output="log",
            arguments=[
                "--x", x, "--y", y, "--z", z,
                "--roll", roll, "--pitch", pitch, "--yaw", yaw,
                "--frame-id", "drone/base_link",
                "--child-frame-id", child,
            ],
        )
        for name, child, x, y, z, roll, pitch, yaw in (
            ("gps", "drone/gps_link", "0", "0", "0.172223", "0", "0", "0"),
        )
    ]

    # Clearpath 공식 구조와 동일하게 최상위에서 바로 include한다.
    # 내부 생성 체인이 끝난 뒤 A300 spawn이 실행된다.
    # Module B의 EKF 노드가 /wheel/tf에서 odom->base_link를 수신할 수 있도록,
    # 그리고 tf_prefix_relay가 작동할 수 있도록 전체 휠 스택의 TF를 격리한다.
    # clearpath가 띄우는 wheel spawner는 `--controller-manager-timeout 60`이 박혀
    # 있고(/opt/ros의 clearpath_control 런치라 우리가 못 고친다), 전체 스택을 같이
    # 띄우면 wheel.controller_manager가 60초 안에 못 뜬다. 실측(방식 4 종단):
    #   [wheel.controller_manager] Waiting for data on 'robot_description' topic
    #   [wheel.spawner_joint_state_broadcaster] FATAL: Could not contact service
    #     /wheel/controller_manager/list_controllers   -> 두 spawner 모두 사망
    # 그러면 /wheel/joint_states가 없어 robot_state_publisher가
    # wheel/odom -> wheel/base_link 를 못 내고, 이어서 wheel EKF가
    #   Could not obtain transform from wheel/odom->wheel/base_link
    # 로 map -> wheel/odom 을 영영 발행하지 못한다. 즉 모듈 B가 통째로 실패하고
    # 모듈 C·D·E까지 멈춘다.
    #
    # 그래서 넉넉한 제한 시간으로 한 번 더 붙인다. clearpath 쪽이 먼저 성공하면
    # 이쪽은 "already loaded"로 조용히 끝나므로 중복 부작용이 없다.
    # ROS_HOME을 따로 주는 이유는 spawner들이 공유하는 락 파일 때문이다 —
    # 같은 락을 쓰면 이 재시도가 leg spawner를 굶긴다.
    wheel_controller_retry = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "platform_velocity_controller",
            "--controller-manager", "/wheel/controller_manager",
            "--controller-manager-timeout", "300",
            "--switch-timeout", "300",
            "--service-call-timeout", "180",
        ],
        additional_env={
            "ROS_HOME": os.path.join(os.path.expanduser("~"), ".ros", "agconav_wheel_retry"),
        },
    )

    spawn_wheel = GroupAction([
        SetRemap('/tf', '/wheel/tf'),
        SetRemap('/tf_static', '/wheel/tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                clearpath_spawn_launch_path
            ),
            launch_arguments={
                "setup_path": clearpath_setup_path,
                "world": "Seongdong_gu",
                "use_sim_time": use_sim_time,
                "rviz": "false",
                "generate": "false",
                "x": LaunchConfiguration("wheel_x"),
                "y": LaunchConfiguration("wheel_y"),
                "z": LaunchConfiguration("wheel_z"),
                "yaw": LaunchConfiguration("wheel_yaw"),
            }.items(),
        ),
        # 여기서 platform_velocity_controller spawner를 따로 띄우지 않는다.
        # clearpath 스택이 PushRosNamespace('wheel') 아래에서 같은 spawner를 이미
        # 띄우며(상대 이름 controller_manager -> /wheel/controller_manager로 해결),
        # generate:=false 산출물이 정상화된 뒤로는 그쪽이 정상 동작한다.
        # 두 개를 같이 띄우면 먼저 붙은 쪽이 컨트롤러를 active로 만들고 나머지가
        # "can not be configured from 'active' state"로 죽는다 — 매 실행마다
        # 어느 쪽이 죽는지 달라지는 경쟁 상태이자 무의미한 ERROR였다.
    ])

    # Go2 원본 launch도 Gazebo와 spawn을 동시에 시작하는 구조다.
    # spawn-only 복사본이 기존 Gazebo의 create 서비스를 사용한다.
    # RL 판 (기본). 인자 구성이 CHAMP 판과 달라 따로 만든다.
    spawn_leg_rl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(go2_spawn_launch_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": "leg",
            "world_init_x": LaunchConfiguration("leg_x"),
            "world_init_y": LaunchConfiguration("leg_y"),
            "world_init_z": LaunchConfiguration("leg_z"),
            "world_init_heading": LaunchConfiguration("leg_yaw"),
            "model_folder": LaunchConfiguration("leg_policy"),
            "max_linear": LaunchConfiguration("leg_max_speed"),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("leg_controller"),
                              "' == 'rl'"])),
    )

    # Unitree 공식 판 (기본). model_folder 가 없다 — 학습 정책이 아니다.
    spawn_leg_guide = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(go2_guide_spawn_launch_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": "leg",
            "world_init_x": LaunchConfiguration("leg_x"),
            "world_init_y": LaunchConfiguration("leg_y"),
            "world_init_z": LaunchConfiguration("leg_z"),
            "world_init_heading": LaunchConfiguration("leg_yaw"),
            "max_linear": LaunchConfiguration("leg_max_speed"),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("leg_controller"),
                              "' == 'guide'"])),
    )

    # CHAMP 판 (leg_controller:=champ 일 때만).
    spawn_leg_champ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(go2_champ_spawn_launch_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_localization": use_localization,
            "rviz": "false",
            "robot_name": "leg",
            "world_init_x": LaunchConfiguration("leg_x"),
            "world_init_y": LaunchConfiguration("leg_y"),
            "world_init_z": LaunchConfiguration("leg_z"),
            "world_init_heading": LaunchConfiguration("leg_yaw"),
            "ros_control_file": os.path.join(
                agconav_bringup_share,
                "config",
                "leg_controllers.yaml",
            ),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("leg_controller"),
                              "' == 'champ'"])),
    )
    # gz_quadruped_hardware가 자기 모델의 설정된 센서만 읽도록 수정했으므로
    # RL/CHAMP 모두 같은 시점에 안전하게 스폰할 수 있다.
    spawn_leg = GroupAction([spawn_leg_guide, spawn_leg_rl, spawn_leg_champ])

    # 월드 로드 완료를 기다렸다가 spawn을 시작한다.
    # 성동구 월드는 heightmap과 건물 메시가 커서 로드가 오래 걸리는데,
    # ros_gz_sim의 create 노드가 먼저 뜨면 "Requesting list of world names"만
    # 반복하며 멈춘다(약 1/5 확률로 시뮬 자체가 기동 실패).
    wait_for_world = ExecuteProcess(
        cmd=[
            os.path.join(agconav_bringup_share, "scripts", "wait_for_world.sh"),
            "Seongdong_gu",
            "180",
        ],
        name="wait_for_world",
        output="screen",
    )

    # 대기가 끝난 뒤에만 두 로봇을 spawn한다.
    #
    # 종료코드를 반드시 본다. 예전에는 성공/실패 구분 없이 spawn을 시작해서,
    # 월드가 안 뜬 실행에서도 로봇 스택이 그대로 올라왔다. 그러면 Gazebo만
    # 죽어 있고 나머지 100여 개 노드는 멀쩡히 떠 있는 상태가 되어
    # "왜 아무것도 안 되지?"를 로그 1000줄에서 찾아야 한다.
    # 실측: gz sim이 아무 출력 없이 멈춘 실행이 10회 중 1회 있었다.
    # 이제는 그 자리에서 이유를 찍고 launch 전체를 내린다.
    def _on_world_wait_exit(event, context):                  # noqa: ARG001
        if event.returncode == 0:
            return _rest_of_stack()
        return [
            LogInfo(msg=(
                '[agconav_sim] 월드가 뜨지 않아 로봇 spawn을 건너뛰고 종료합니다. '
                'Gazebo(gz sim)가 응답하지 않았습니다 — 이전 실행의 gz 프로세스가 '
                '남아 있는지(pgrep -f "gz sim") 확인하고 다시 실행하세요.')),
            Shutdown(reason='world load timeout'),
        ]

    spawn_after_world = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_world,
            on_exit=_on_world_wait_exit,
        )
    )

    # 센서 브리지(공통): 세 로봇 센서를 모듈 계약 토픽 이름으로 정합해 발행한다.
    sensor_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_bridge_launch_path),
    )

    # leg 모델은 월드 준비 뒤에도 15초 늦게 생성된다. 공통 브리지를 그보다
    # 먼저 시작하면 Gazebo에는 아직 /leg/points/points가 없어서 Jazzy의
    # parameter_bridge가 해당 PointCloud2 연결을 만들지 못하는 실행이 있다.
    # 모델 생성 뒤 전용 브리지를 한 번 더 시작해 /leg/points 계약을 보장한다.
    leg_lidar_bridge_after_spawn = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="leg_lidar_bridge_after_spawn",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "config_file": os.path.join(
                get_package_share_directory("agconav_gz_bridge"),
                "config", "leg_lidar_lazy_bridge.yaml"),
        }],
    )

    # 지상 로봇 공통 Nav2, 설정은 하나, 로봇별 차이는 namespace, footprint뿐(README 4참고)
    def _nav2_for(namespace, stamped_cmd_vel, extra_rewrites=None):
        # 설정 파일은 하나(nav2_common.yaml)를 공유하되, cmd_vel 메시지 타입만
        # 로봇별로 덮어쓴다. Nav2의 TwistPublisher/TwistSubscriber는 노드
        # 파라미터 enable_stamped_cmd_vel로 Twist / TwistStamped를 고르는데,
        # 이 값이 로봇마다 달라야 한다(wheel=TwistStamped, leg=Twist).
        # leaf 키 이름으로 넘기면 파일 안의 같은 이름 파라미터를 전부 바꾼다.
        robot_params = RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites={
                'enable_stamped_cmd_vel': stamped_cmd_vel,
                **(extra_rewrites or {}),
            },
            convert_types=True,
        )
        return GroupAction(
            actions=[
                # Nav2를 전역 /tf에 붙인다.
                # nav2_bringup의 각 노드는 remappings=[('/tf','tf'), ...]를 달고
                # PushROSNamespace 아래에서 뜨므로 기본값으로는 사설 /{ns}/tf를 읽는다.
                # 그런데 사설 트리에는 clearpath/CHAMP의 **접두어 없는** 프레임
                # (odom, base_link)만 있고, Nav2 설정은 접두어 붙은 이름
                # ({ns}/odom, {ns}/base_link)과 map을 쓴다. 그래서 costmap이
                # 'Invalid frame ID "wheel/odom" ... frame does not exist'로 활성화에
                # 실패하고, lifecycle manager가 controller_server 활성화에서 멈춰
                # Nav2 스택 전체가 inactive로 남는다.
                # 전역 /tf에는 접두어 붙은 프레임과 map이 모두 있으므로 그쪽을 보게 한다.
                # GroupAction의 SetRemap은 노드 자체 remapping보다 먼저 적용돼 우선한다.
                SetRemap('/tf', '/tf'),
                SetRemap('/tf_static', '/tf_static'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_path),
                    launch_arguments={
                        "namespace": namespace,
                        # use_namespace가 True여야 PushROSNamespace가 적용되어
                        # wheel/leg 두 nav2 스택이 각자 네임스페이스로 분리된다.
                        # 빠지면 두 스택이 루트에 같은 이름으로 떠서 충돌한다.
                        "use_namespace": "True",
                        "use_sim_time": use_sim_time,
                        "params_file": robot_params,
                        "use_composition": "False",
                        "autostart": "True",
                        # 이 launch가 선언한 use_localization("true", 소문자)이
                        # launch configuration 상속으로 nav2 안까지 새어 들어가
                        # PythonExpression 평가 시 NameError를 낸다
                        # (name 'true' is not defined). 명시적으로 넘겨 차단한다.
                        # 값 자체도 False가 맞다 — map->odom TF는 모듈 B의 EKF가
                        # 발행하므로 nav2의 AMCL/map_server는 필요 없다.
                        "use_localization": "False",
                    }.items(),
                ),
                Node(
                    package="agconav_navigation",
                    executable="ground_segmentation_node",
                    name="ground_segmentation_node",
                    namespace=namespace,
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"odom_frame": "odom"}
                    ]
                ),
                Node(
                    package="agconav_navigation",
                    executable="navigation_complete_node",
                    name="navigation_complete",
                    namespace=namespace,
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}]
                )
            ],
            condition=IfCondition(use_nav2)
        )

    # wheel: twist_mux -> diff_drive_controller가 TwistStamped 전용.
    # leg  : CHAMP quadruped_controller가 Twist를 구독.
    nav2_wheel = _nav2_for("wheel", "true")
    nav2_leg = _nav2_for("leg", "false", {
        # !! leg 속도 상한은 **컨트롤러의 설계 한계**에 맞춰야 한다 !!
        # nav2_common.yaml 은 wheel 기준(max_vel_x 1.2)이다. leg 는 다르다.
        #   RL robot_lab      실측 안정 최대 1.045 m/s (docs/11 §6)
        #   unitree_guide     **설계 최대 0.4 m/s** — StateTrotting 이
        #                     v_cmd = invNormalize(ly, -0.4, 0.4) 로 축을 읽는다
        # 지금 기본 컨트롤러는 guide 이므로 0.4 다.
        #
        # 1.0 으로 두면 DWB 가 **로봇이 낼 수 없는 속도로 궤적을 평가**한다.
        # 로봇이 계획을 못 따라가 경로에서 벗어나고, 결국
        #   [leg.controller_server] Resulting plan has 0 poses in it.
        # 로 중단된다. 실측: 목표를 3회 전송했는데 매번 실패하고 로봇이
        # 스폰에서 43 m 엉뚱한 방향으로 이동해 있었다(위치추정은 정상이었다 —
        # 정답 (-219.46,112.93) vs EKF (-219.45,112.93)).
        # max_vel_x 만 고치면 max_speed_xy 에서 다시 잘리므로 함께 내린다.
        # **컨트롤러를 rl 로 되돌리면 이 값도 1.0 으로 되돌릴 것.**
        # 컨트롤러에 따라 자동으로 고른다. 하드코딩하면 컨트롤러를 바꿀 때
        # 같이 고쳐야 하는 함정이 된다.
        'max_vel_x': PythonExpression(
            ["'0.4' if '", LaunchConfiguration('leg_controller'),
             "' == 'guide' else '1.0'"]),
        'max_speed_xy': PythonExpression(
            ["'0.4' if '", LaunchConfiguration('leg_controller'),
             "' == 'guide' else '1.0'"]),
        # 병진이 느려진 만큼 회전도 낮춘다. guide 의 yaw 한계는 0.5 rad/s 다
        # (StateTrotting: w_yaw_limit = +-0.5). 1.5 로 두면 같은 이유로
        # 못 내는 회전을 계획한다.
        # !! guide 는 0.3 이다. 설계 한계 0.5 를 그대로 주면 안 된다 !!
        # StateTrotting 의 w_yaw_limit 이 +-0.5 rad/s 인데, 그건 **회전만 할 때**
        # 의 한계다. Nav2 는 병진 0.4 m/s 와 회전을 동시에 준다. 한계에서
        # 둘을 겹치면 균형 여유가 없어 그대로 전복한다 — 실측: roll 이
        # 78도 -> 140도 -> -53도 로 요동치며 구르고, controller_server 가
        # "Resulting plan has 0 poses in it" 로 중단했다.
        # 직진 시험(제 램프 하네스, 조향 <=0.35 rad/s)에서는 멀쩡히 걸었으므로
        # 컨트롤러 자체가 아니라 **동시 명령의 크기**가 문제다.
        'max_vel_theta': PythonExpression(
            ["'0.3' if '", LaunchConfiguration('leg_controller'),
             "' == 'guide' else '1.5'"]),
        # !! guide 는 가속도 제한도 낮춰야 한다 !!
        # nav2_common.yaml 은 wheel 기준으로 acc_lim_x 2.0 / acc_lim_theta 3.2 다.
        # 0.4 m/s 를 0.2 초 만에 붙이라는 뜻인데, 4족 보행은 한 걸음 주기가
        # 그보다 길어서 명령이 계단처럼 바뀌면 균형 제어가 못 따라간다.
        # 회전 상한만 0.3 으로 낮췄을 때도 여전히 전복했다
        # (roll 176도 -> 160도 -> 86도, "0 poses" 3건).
        # 보폭 주기 안에서 속도가 바뀌도록 완만하게 준다.
        'acc_lim_x': PythonExpression(
            ["'0.5' if '", LaunchConfiguration('leg_controller'),
             "' == 'guide' else '2.0'"]),
        'decel_lim_x': PythonExpression(
            ["'-0.5' if '", LaunchConfiguration('leg_controller'),
             "' == 'guide' else '-2.5'"]),
        'acc_lim_theta': PythonExpression(
            ["'0.6' if '", LaunchConfiguration('leg_controller'),
             "' == 'guide' else '3.2'"]),
        'decel_lim_theta': PythonExpression(
            ["'-0.6' if '", LaunchConfiguration('leg_controller'),
             "' == 'guide' else '-3.2'"]),
        # Go2는 제자리 최종 회전에서 위치를 다시 벗어나는 경향이 있다.
        # 모듈 D에는 도착 위치가 중요하고 최종 heading은 중요하지 않다.
        'yaw_goal_tolerance': '3.14',
        'xy_goal_tolerance': '0.35',
        # 보행 로봇은 제자리 방향 전환만으로도 wheel보다 오래 걸린다.
        # 기본 10초/0.5m 판정은 정상 회전 중에도 "진행 없음"으로 복구를
        # 시작하므로, 작은 보행을 진행으로 인정하고 회전 시간을 확보한다.
        'movement_time_allowance': '30.0',
        'required_movement_radius': '0.1',
    })

    nav2_wheel_recover = Node(
        package="agconav_navigation",
        executable="nav2_lifecycle_recover",
        name="wheel_nav2_lifecycle_recover",
        parameters=[{"target_namespace": "wheel"}],
        output="screen",
        condition=IfCondition(use_nav2),
    )
    nav2_leg_recover = Node(
        package="agconav_navigation",
        executable="nav2_lifecycle_recover",
        name="leg_nav2_lifecycle_recover",
        parameters=[{"target_namespace": "leg"}],
        output="screen",
        condition=IfCondition(use_nav2),
    )

    def _reset_early_leg(context):
        """Stand an early-spawned leg robot upright just before B/C start."""
        x = float(LaunchConfiguration("leg_x").perform(context))
        y = float(LaunchConfiguration("leg_y").perform(context))
        z = float(LaunchConfiguration("leg_z").perform(context))
        yaw = float(LaunchConfiguration("leg_yaw").perform(context))
        request = (
            f'name: "leg", position: {{x: {x}, y: {y}, z: {z}}}, '
            'orientation: {x: 0, y: 0, '
            f'z: {math.sin(yaw / 2.0)}, w: {math.cos(yaw / 2.0)}}}'
        )
        return [ExecuteProcess(
            cmd=[
                "gz", "service", "-s", "/world/Seongdong_gu/set_pose",
                "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "5000", "--req", request,
            ],
            output="screen",
        )]

    reset_early_leg = OpaqueFunction(function=_reset_early_leg)

    # Localization (EKF + navsat) for ground robots — Module B
    # localization.launch.py가 map→odom TF를 /{ns}/tf에 발행한다.
    def _localization_for(namespace_name, odom_topic):
        return GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(localization_launch_path),
                    launch_arguments={
                        "namespace": namespace_name,
                        "use_sim_time": use_sim_time,
                        "odom_topic": odom_topic,
                    }.items(),
                )
            ],
            condition=IfCondition(use_localization)
        )

    # wheel의 원본 odometry는 clearpath diff_drive_controller가
    # /wheel/platform/odom 으로 낸다(모듈 B launch 문서의 사용 예시와 동일).
    # /wheel/odom 에는 발행자가 없어 EKF의 odom0가 비고 map -> wheel/odom TF도
    # 발행되지 않는다.
    localization_wheel = _localization_for("wheel", "/wheel/platform/odom")
    localization_leg = _localization_for("leg", "/leg/odom")


    def _ground_models():
        """지상 로봇 물리 모델, 제어기, 센서 브리지."""
        return [
            *wheel_frame_alias,
            spawn_wheel,
            TimerAction(period=15.0, actions=[spawn_leg]),
            TimerAction(period=22.0, actions=[leg_lidar_bridge_after_spawn]),
            TimerAction(period=25.0, actions=[wheel_controller_retry]),
        ]

    def _ground_autonomy():
        """Module B 위치추정과 Module C/Nav2."""
        return [
            # A의 전체 500x500m 비행 동안 물리 모델을 함께 둔 경우, 낮은
            # real-time factor에서 CHAMP가 쓰러질 수 있다. F 완료 후 위치추정이
            # 시작되기 전에 원래 스폰 자세로 한 번 세우고 충분히 안정시킨다.
            TimerAction(
                period=2.0,
                actions=[reset_early_leg],
                condition=IfCondition(spawn_ground_early),
            ),
            TimerAction(period=30.0, actions=[
                localization_wheel,
                localization_leg,
            ]),
            TimerAction(
                period=170.0, actions=[nav2_wheel],
                condition=UnlessCondition(PythonExpression(
                    ["'", LaunchConfiguration("leg_controller"), "' == 'champ'"]))),
            TimerAction(
                period=190.0, actions=[nav2_leg],
                condition=UnlessCondition(PythonExpression(
                    ["'", LaunchConfiguration("leg_controller"), "' == 'champ'"]))),
            TimerAction(period=240.0, actions=[nav2_wheel_recover]),
            TimerAction(period=270.0, actions=[nav2_leg_recover]),
            TimerAction(
                period=40.0, actions=[nav2_wheel],
                condition=IfCondition(PythonExpression(
                    ["'", LaunchConfiguration("leg_controller"), "' == 'champ'"]))),
            TimerAction(
                period=55.0, actions=[nav2_leg],
                condition=IfCondition(PythonExpression(
                    ["'", LaunchConfiguration("leg_controller"), "' == 'champ'"]))),
        ]

    def _ground_stack():
        return [*_ground_models(), *_ground_autonomy()]

    stage_gate = Node(
        package="agconav_navigation",
        executable="pipeline_stage_gate",
        name="simulation_stage_gate",
        output="screen",
        condition=IfCondition(sequential_pipeline),
    )

    def _on_stage_gate_exit(event, context):  # noqa: ARG001
        if event.returncode == 0:
            actions = [LogInfo(msg='[agconav_sim] Module F 완료 — B와 C를 시작합니다.')]
            if context.perform_substitution(spawn_ground_early).lower() not in ('true', '1', 'yes'):
                actions.extend(_ground_models())
            actions.extend(_ground_autonomy())
            return actions
        return [Shutdown(reason='pipeline stage gate failed')]

    ground_after_stage_gate = RegisterEventHandler(
        OnProcessExit(target_action=stage_gate, on_exit=_on_stage_gate_exit),
        condition=IfCondition(sequential_pipeline),
    )

    def _rest_of_stack():
        """월드가 뜬 뒤에 시작할 것들.

        예전에는 Gazebo와 함께 한꺼번에 띄웠다. 그런데 성동구 월드는
        heightmap(717x665 m)과 건물 메시를 읽는 동안 CPU·GPU·디스크를 크게 쓰는데,
        같은 순간에 로봇 스택 2종 + Nav2 2세트 + 브리지 + 모듈 노드가 한꺼번에
        올라오면서 gz sim이 SDF 한 줄도 파싱하지 못한 채 멈추는 실행이 나왔다
        (10회 중 1~2회, 로그에 gazebo 출력이 3줄뿐). gz sim만 따로 돌리면
        같은 조건에서 12/12 정상이라, 기동 순간의 경합이 원인으로 보인다.
        월드가 응답하는 것을 확인한 뒤에 나머지를 올려 그 창을 없앤다.
        """
        # 한꺼번에 올리지 않고 의존 순서대로 나눠 올린다.
        # 동시에 띄우면 ros2_control spawner가 컨트롤러 활성화를 기다리다
        # 기본 제한시간(5초) 안에 못 끝내고 죽는 실행이 나온다
        #   [ERROR] Failed to activate controller : joint_state_broadcaster
        # 그러면 /wheel/joint_states가 없어 wheel/base_link TF가 통째로 사라진다.
        # clearpath가 만드는 spawner라 --switch-timeout을 우리가 넘길 수 없어서,
        # 대신 그 순간에 다른 스택이 CPU를 뺏지 않도록 시작 시점을 벌린다.
        # 순서는 데이터 의존성과도 일치한다:
        #   로봇(odom·joint) -> 센서 브리지(GPS·IMU) -> 위치추정(TF) -> Nav2
        return [
            clock_bridge,
            drone_cmd_vel_bridge,
            drone_tf_bridge,
            *drone_sensor_tf,
            # 하나의 bridge launch가 drone/wheel/leg 센서를 함께 정의한다.
            # 드론 점군도 여기서 나오므로 A 단계부터 반드시 실행해야 한다.
            # 지상 모델이 아직 없을 때의 wheel/leg bridge는 데이터가 없어
            # 대기만 하므로 CPU 비용은 사실상 없다.
            sensor_bridge,
            GroupAction(
                actions=_ground_models(),
                condition=IfCondition(PythonExpression([
                    "'", sequential_pipeline, "' == 'true' and '",
                    spawn_ground_early, "' == 'true'",
                ])),
            ),
            GroupAction(
                actions=_ground_stack(),
                condition=UnlessCondition(sequential_pipeline),
            ),
            stage_gate,
            ground_after_stage_gate,
        ]

    return LaunchDescription(
        [
            declare_world_file,
            declare_model_path,
            declare_use_sim_time,
            declare_clearpath_setup_path,
            declare_use_localization,
            declare_use_nav2,
            declare_sequential_pipeline,
            declare_spawn_ground_early,
            declare_headless,
            declare_nav2_params_file,
            *declare_spawn_args,
            set_gz_resource_path,
            set_torch_lib_path,
            # 처음에 띄우는 것은 Gazebo와 "월드 로드 대기" 둘뿐이다.
            # 나머지는 전부 _on_world_wait_exit에서 시작한다(위 주석 참고).
            gazebo,
            gazebo_headless,
            wait_for_world,
            spawn_after_world,
        ]
    )
