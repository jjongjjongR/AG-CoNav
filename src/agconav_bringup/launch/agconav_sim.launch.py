import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    clearpath_setup_path = LaunchConfiguration("clearpath_setup_path")
    use_nav2 = LaunchConfiguration("use_nav2")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    # 설치된 ROS2 패키지의 공유 디렉터리
    agconav_worlds_share = get_package_share_directory("agconav_worlds")
    agconav_bringup_share = get_package_share_directory("agconav_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    clearpath_gz_share = get_package_share_directory("clearpath_gz")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # 실행할 공용 Gazebo 월드
    world_path = os.path.join(
        agconav_worlds_share,
        "worlds",
        "agconav_integrated.sdf",
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

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation time",
    )
    nav2_launch_path = os.path.join(
        nav2_bringup_share,
        "launch",
        "bringup_launch.py",
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

    declare_use_nav2 = DeclareLaunchArgument(
        "use_nav2",
        default_value="false",
        description="Launch Nav2 for wheel/leg (needs /X/nav_map and localization TF)"
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
    _gz_models_dir = os.path.join(
        get_package_share_directory("agconav_description"), "models"
    )
    _existing_gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=(
            _gz_models_dir + os.pathsep + _existing_gz_resource_path
            if _existing_gz_resource_path
            else _gz_models_dir
        ),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            "gz_args": [world_path],
        }.items(),
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

    # Clearpath 공식 구조와 동일하게 최상위에서 바로 include한다.
    # 내부 생성 체인이 끝난 뒤 A300 spawn이 실행된다.
    spawn_wheel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            clearpath_spawn_launch_path
        ),
        launch_arguments={
            "setup_path": clearpath_setup_path,
            "world": "agconav_world",
            "use_sim_time": use_sim_time,
            "rviz": "false",
            "generate": "true",
            "x": "-5.0",
            "y": "0.0",
            "z": "0.3",
            "yaw": "0.0",
        }.items(),
    )

    # Go2 원본 launch도 Gazebo와 spawn을 동시에 시작하는 구조다.
    # spawn-only 복사본이 기존 Gazebo의 create 서비스를 사용한다.
    spawn_leg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            go2_spawn_launch_path
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz": "false",
            "robot_name": "leg",
            "world_init_x": "5.0",
            "world_init_y": "0.0",
            "world_init_z": "0.25",
            "world_init_heading": "3.14159",
            "ros_control_file": os.path.join(
                agconav_bringup_share,
                "config",
                "leg_controllers.yaml",
            ),
        }.items(),
    )

    # 센서 브리지(공통): 세 로봇 센서를 모듈 계약 토픽 이름으로 정합해 발행한다.
    sensor_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_bridge_launch_path),
    )

    # 지상 로봇 공통 Nav2, 설정은 하나, 로봇별 차이는 namespace, footprint뿐(README 4참고)
    def _nav2_for(namespace):
        return GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_path),
                    launch_arguments={
                        "namespace": namespace,
                        # use_namespace가 True여야 PushROSNamespace가 적용되어
                        # wheel/leg 두 nav2 스택이 각자 네임스페이스로 분리된다.
                        # 빠지면 두 스택이 루트에 같은 이름으로 떠서 충돌한다.
                        "use_namespace": "True",
                        "use_sim_time": use_sim_time,
                        "params_file": nav2_params_file,
                        "use_composition": "False",
                        "autostart": "True",
                    }.items(),
                )
            ],
            condition=IfCondition(use_nav2)
        )

    nav2_wheel = _nav2_for("wheel")
    nav2_leg = _nav2_for("leg")


    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_clearpath_setup_path,
            declare_use_nav2,
            declare_nav2_params_file,
            set_gz_resource_path,
            gazebo,
            clock_bridge,
            drone_cmd_vel_bridge,
            spawn_wheel,
            spawn_leg,
            sensor_bridge,
            nav2_wheel,
            nav2_leg,
        ]
    )
