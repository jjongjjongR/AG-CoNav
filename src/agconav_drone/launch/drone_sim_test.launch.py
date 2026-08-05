"""
drone_sim_test.launch.py

drone_path_player + drone_pose_controller를 함께 띄우는 테스트용 launch 파일.
path_file(경로 YAML)은 두 노드의 yaml 설정을 그대로 불러온 뒤, 이 launch 파일이
get_package_share_directory("agconav_drone")로 계산한 실제 install 경로로 덮어써서
사용자별 절대경로 하드코딩(예: /home/yeonj/...) 없이 어느 PC에서도 동작하게 한다.

기본값은 world_name:=Seongdong_gu (repo에 있는 유일한 world) 이지만, 노드 자체
(drone_pose_controller.py)에는 이 기본값이 없다 - world_name은 필수 파라미터이고,
이 launch 인자 하나만 바꾸면 다른 world로도 그대로 재사용할 수 있게 하기 위함이다.

launch_gazebo:=false 로 주면 Gazebo/world/clock 브리지는 띄우지 않고 두 드론 노드만
띄운다 (예: agconav_bringup의 전체 시뮬레이션이 이미 떠 있어 Gazebo를 중복 실행하면
안 되는 경우). set_pose 서비스 브리지는 agconav_bringup 쪽에 아직 없으므로
launch_gazebo 값과 무관하게 항상 띄운다.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    agconav_drone_share = get_package_share_directory("agconav_drone")
    agconav_worlds_share = get_package_share_directory("agconav_worlds")
    agconav_description_share = get_package_share_directory("agconav_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_name = LaunchConfiguration("world_name")
    entity_name = LaunchConfiguration("entity_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_gazebo = LaunchConfiguration("launch_gazebo")

    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value="Seongdong_gu",
        description="Gazebo world 이름 (drone_pose_controller의 필수 파라미터로 그대로 전달됨)",
    )
    declare_entity_name = DeclareLaunchArgument(
        "entity_name",
        default_value="X3",
        description="Gazebo 드론(X3) 모델 이름",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation time",
    )
    declare_launch_gazebo = DeclareLaunchArgument(
        "launch_gazebo",
        default_value="true",
        description="Gazebo/world/clock 브리지를 이 launch에서 함께 띄울지 여부 "
        "(다른 곳에서 이미 Gazebo가 떠 있다면 false)",
    )

    # worlds/<world_name>/<world_name>.world 관례를 따른다 (agconav_worlds 레이아웃).
    world_path = PathJoinSubstitution(
        [agconav_worlds_share, "worlds", world_name, [world_name, ".world"]]
    )

    _gz_models_dirs = [
        os.path.join(agconav_description_share, "models"),
        os.path.join(agconav_worlds_share, "models"),
    ]
    _existing_gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if _existing_gz_resource_path:
        _gz_models_dirs.append(_existing_gz_resource_path)
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(_gz_models_dirs),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world_path, " -r"]}.items(),
        condition=IfCondition(launch_gazebo),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="drone_sim_test_clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        condition=IfCondition(launch_gazebo),
    )

    # ros_gz_interfaces/srv/SetEntityPose <-> gz world set_pose 서비스 브리지.
    # UserCommands 시스템 플러그인(Seongdong_gu.world에 이미 로드됨)이 제공하는
    # 네이티브 gz-transport 서비스를 ROS2 서비스로 노출한다.
    set_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="drone_set_pose_bridge",
        output="screen",
        arguments=[
            ["/world/", world_name, "/set_pose@ros_gz_interfaces/srv/SetEntityPose"]
        ],
    )

    path_player_yaml = os.path.join(
        agconav_drone_share, "config", "drone_path_player.yaml"
    )
    path_file = PathJoinSubstitution(
        [agconav_drone_share, "config", "path.yaml"]
    )

    drone_path_player = Node(
        package="agconav_drone",
        executable="drone_path_player",
        name="drone_path_player",
        output="screen",
        parameters=[
            path_player_yaml,
            {"path_file": path_file, "use_sim_time": use_sim_time},
        ],
    )

    pose_controller_yaml = os.path.join(
        agconav_drone_share, "config", "drone_pose_controller.yaml"
    )

    drone_pose_controller = Node(
        package="agconav_drone",
        executable="drone_pose_controller",
        name="drone_pose_controller",
        output="screen",
        parameters=[
            pose_controller_yaml,
            {
                "world_name": world_name,
                "entity_name": entity_name,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        [
            declare_world_name,
            declare_entity_name,
            declare_use_sim_time,
            declare_launch_gazebo,
            set_gz_resource_path,
            gazebo,
            clock_bridge,
            set_pose_bridge,
            drone_path_player,
            drone_pose_controller,
        ]
    )
