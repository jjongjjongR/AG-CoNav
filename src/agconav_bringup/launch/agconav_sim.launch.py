import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
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
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    headless = LaunchConfiguration("headless")

    # 설치된 ROS2 패키지의 공유 디렉터리
    agconav_worlds_share = get_package_share_directory("agconav_worlds")
    agconav_bringup_share = get_package_share_directory("agconav_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    clearpath_gz_share = get_package_share_directory("clearpath_gz")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # 실행할 공용 Gazebo 월드 (성동구 실지형 heightmap).
    # world_file 인자로 다른 월드를 지정할 수 있다 — 테스트용으로 잘라낸 축소
    # 월드(agconav_test_worlds)에서 전체 스택을 돌릴 때 쓴다. 빈 값이면 기본값.
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

    go2_spawn_launch_path = os.path.join(
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
        "기본 월드에 없는 model://을 참조할 때 필요하다 "
        "(예: 방식 4 월드의 agconav_drone_dynamic).",
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
        ("leg_z", "6.1500", "Go2(leg) 스폰 z [m] — 지면 5.85 + 0.30 (기립 높이)"),
        ("leg_yaw", "-0.4613", "Go2(leg) 스폰 heading [rad]"),
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
    # 없을 수 있다. 예: 방식 4(velocity)용 Seongdong_gu_100x100_dynamic 은
    # model://agconav_drone_dynamic 을 참조하는데 그건 agconav_test_worlds/models
    # 에 있다. 경로에 없으면 Gazebo가 월드 로드 자체를 실패하고
    #   [Err] Error Code 14 ... Unable to find uri[model://agconav_drone_dynamic]
    # 가 뜬 뒤, 월드가 안 떠서 wait_for_world가 죽고 모듈 A~F가 전부 연쇄로 죽는다.
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
    spawn_leg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            go2_spawn_launch_path
        ),
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
    )

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

    # 지상 로봇 공통 Nav2, 설정은 하나, 로봇별 차이는 namespace, footprint뿐(README 4참고)
    def _nav2_for(namespace, stamped_cmd_vel):
        # 설정 파일은 하나(nav2_common.yaml)를 공유하되, cmd_vel 메시지 타입만
        # 로봇별로 덮어쓴다. Nav2의 TwistPublisher/TwistSubscriber는 노드
        # 파라미터 enable_stamped_cmd_vel로 Twist / TwistStamped를 고르는데,
        # 이 값이 로봇마다 달라야 한다(wheel=TwistStamped, leg=Twist).
        # leaf 키 이름으로 넘기면 파일 안의 같은 이름 파라미터를 전부 바꾼다.
        robot_params = RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites={'enable_stamped_cmd_vel': stamped_cmd_vel},
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
    nav2_leg = _nav2_for("leg", "false")

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
            *wheel_frame_alias,
            # wheel 스택을 가장 먼저, 혼자 올린다.
            # clearpath의 ros2_control spawner 2개는 락 하나를 공유하고,
            # 락을 잡은 쪽이 /wheel/controller_manager를 최대 60초 기다린다.
            # 그 60초 안에 CM이 못 뜨면
            #   [FATAL] Could not contact service /wheel/controller_manager/list_controllers
            # 로 죽고, 락을 못 잡은 나머지도 같이 무너진다. 그러면
            # /wheel/joint_states가 없어 wheel/base_link TF가 통째로 사라진다.
            # CM은 A300 모델이 Gazebo에 스폰되고 robot_description을 받은 뒤에야
            # 뜨므로, 그 구간에 다른 스택이 CPU를 뺏지 않게 하는 것이 핵심이다.
            spawn_wheel,
            TimerAction(period=15.0, actions=[spawn_leg]),
            # clearpath spawner가 60초에 죽고 난 뒤 우리 쪽이 이어받는다.
            TimerAction(period=25.0, actions=[wheel_controller_retry]),
            TimerAction(period=30.0, actions=[
                sensor_bridge,
                localization_wheel,
                localization_leg,
            ]),
            # Nav2는 위치추정 TF가 있어야 costmap이 활성화되므로 마지막이다.
            # 다만 너무 늦추면 모듈 C(지면 분할)도 같이 늦어져 점검 시점에
            # points_filtered가 비어 있는다 — wait_ready.py가 그것까지
            # 기다리도록 해서 시점 의존을 없앴다.
            TimerAction(period=40.0, actions=[nav2_wheel, nav2_leg]),
        ]

    return LaunchDescription(
        [
            declare_world_file,
            declare_model_path,
            declare_use_sim_time,
            declare_clearpath_setup_path,
            declare_use_localization,
            declare_use_nav2,
            declare_headless,
            declare_nav2_params_file,
            *declare_spawn_args,
            set_gz_resource_path,
            # 처음에 띄우는 것은 Gazebo와 "월드 로드 대기" 둘뿐이다.
            # 나머지는 전부 _on_world_wait_exit에서 시작한다(위 주석 참고).
            gazebo,
            gazebo_headless,
            wait_for_world,
            spawn_after_world,
        ]
    )
