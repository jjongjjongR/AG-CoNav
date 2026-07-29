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

    # 이 launch 파일 자신의 실제 경로(--symlink-install이므로 src/ 원본을 가리킴)를 기준으로
    # 저장소 루트의 config/clearpath_a300을 찾는다. 클론 위치(~/projects/AG-CoNav 등)에
    # 의존하지 않아 다른 팀원 컴퓨터에서도 그대로 동작한다.
    _repo_root = os.path.abspath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "..")
    )
    declare_clearpath_setup_path = DeclareLaunchArgument(
        "clearpath_setup_path",
        default_value=os.path.join(_repo_root, "config", "clearpath_a300"),
        description="Directory containing the A300 robot.yaml",
    )

    declare_use_nav2 = DeclareLaunchArgument(
        "use_nav2",
        default_value="false",
        description="Lunch Nav2 for wheel/leg (needs /X/nav_map and localization TF)"
    )

    declare_nav2_params_file = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(_repo_root, "src", "agconav_navigation", "config", "nav2_common.yaml"),
        description="Common Nav2 params for both ground robots (module C)",
    )

    # Gazebo와 공용 월드는 여기서 한 번만 실행한다.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            "gz_args": [world_path, " -r"],
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
            "x": "-8.0",
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
            "world_init_x": "8.0",
            "world_init_y": "0.0",
            "world_init_z": "0.45",
            "world_init_heading": "3.14159",
        }.items(),
    )

    # 지상 로봇 공통 Nav2, 설정은 하나, 로봇별 차이는 namespace, footprint뿐(README 4참고)
    def _nav2_for(namespace):
        return GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_path),
                    launch_arguments={
                        "namespace": namespace,
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
            gazebo,
            clock_bridge,
            drone_cmd_vel_bridge,
            spawn_wheel,
            spawn_leg,
            nav2_wheel,
            nav2_leg,
        ]
    )
