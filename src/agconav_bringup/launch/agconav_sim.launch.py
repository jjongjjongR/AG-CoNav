import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    clearpath_setup_path = LaunchConfiguration("clearpath_setup_path")

    # 설치된 ROS2 패키지의 공유 디렉터리
    agconav_worlds_share = get_package_share_directory("agconav_worlds")
    agconav_bringup_share = get_package_share_directory("agconav_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    clearpath_gz_share = get_package_share_directory("clearpath_gz")

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

    declare_clearpath_setup_path = DeclareLaunchArgument(
        "clearpath_setup_path",
        default_value=PathJoinSubstitution(
            [
                EnvironmentVariable("HOME"),
                "projects",
                "AG-CoNav",
                "config",
                "clearpath_a300",
            ]
        ),
        description="Directory containing the A300 robot.yaml",
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

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_clearpath_setup_path,
            gazebo,
            clock_bridge,
            drone_cmd_vel_bridge,
            spawn_wheel,
            spawn_leg,
        ]
    )
