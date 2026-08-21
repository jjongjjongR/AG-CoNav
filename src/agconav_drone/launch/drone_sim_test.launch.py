"""
drone_sim_test.launch.py

모듈 A(드론 지도 생성)를 띄우는 launch 파일. 드론은 drone_velocity_follower가
실제 로터 추력으로 몬다(월드의 MulticopterVelocityControl).

SetEntityPose 순간이동 방식(drone_path_player + drone_pose_controller)은 폐기했다.
물리엔진이 운동을 보지 못해 IMU가 죽고(실측 gyro 최대 0.0013 rad/s) 자세가 항상
수평이라 스캔 시야가 연직으로만 고정됐다. 실제 비행으로 바꾸면 기체가 기울고
오르내리며 근거리 반사를 더 얻는다 — 커버리지 78.7 -> 98.9%, 높이 오차 sigma
0.1175 -> 0.0839 m. 두 노드의 진입점은 비교 실험용으로 남아 있다.

path_file(경로 YAML)은 이 launch 파일이 get_package_share_directory로 계산한 실제
install 경로로 덮어써서, 사용자별 절대경로 하드코딩 없이 어느 PC에서도 동작한다.

launch_gazebo:=false 로 주면 Gazebo/clock/twist 브리지는 띄우지 않는다 (예:
agconav_bringup의 전체 시뮬레이션이 이미 떠 있는 경우). 그때도 enable 브리지는
필요하므로 이쪽에서 올린다 — agconav_sim은 twist만 브리지한다.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    declare_path_file = DeclareLaunchArgument(
        "path_file",
        default_value="",
        description="드론 경로 YAML 절대경로. 비우면 agconav_drone/config/path.yaml.",
    )
    # 순항 속도. **6.0 이 확정값이다**(문서 10 종단 테스트).
    #   v10 x 6 -> wheel 계획 실패 19건, 18.5 m 후 ABORTED
    #   v6  x 3 -> 계획 실패 0건, 191.5 m 주행 SUCCEEDED
    # 문서 9 도 최적 5.2~6.0 m/s 로 같은 결론이고, 6 -> 8 m/s 사이에서
    # 고도가 0.61 -> 3.00 m 로 흔들려 지도에 그대로 잡음이 실린다.
    declare_cruise_speed = DeclareLaunchArgument(
        "cruise_speed",
        default_value="6.0",
        description="드론 순항 속도 지령 [m/s]",
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

    # 월드의 MulticopterVelocityControl 은 enableSubTopic 으로 True 를 먼저 받아야
    # twist 를 받아들인다. 이걸 안 띄우면 twist 를 아무리 보내도 로터가 안 돈다.
    # twist 브리지(/drone/cmd_vel)는 launch_gazebo:=false 일 때 agconav_sim 이
    # 이미 띄우므로, 여기서는 단독 실행일 때만 함께 올린다.
    drone_cmd_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="drone_cmd_bridge",
        output="screen",
        condition=IfCondition(launch_gazebo),
        arguments=[
            "/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/X3/enable@std_msgs/msg/Bool]gz.msgs.Boolean",
        ],
        remappings=[
            ("/X3/gazebo/command/twist", "/drone/cmd_vel"),
            ("/X3/enable", "/drone/enable"),
        ],
    )

    # agconav_sim 과 함께 뜰 때는 twist 브리지가 그쪽에 있고 enable 만 없다.
    drone_enable_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="drone_enable_bridge",
        output="screen",
        condition=UnlessCondition(launch_gazebo),
        arguments=["/X3/enable@std_msgs/msg/Bool]gz.msgs.Boolean"],
        remappings=[("/X3/enable", "/drone/enable")],
    )

    # 기본은 agconav_drone/config/path.yaml (전체 월드용). 축소 테스트 월드에서는
    # 그 월드의 heightmap 범위로 다시 생성한 경로를 path_file 인자로 넘긴다.
    path_file = PythonExpression(
        ["'", LaunchConfiguration("path_file"), "' or '",
         os.path.join(agconav_drone_share, "config", "path.yaml"), "'"]
    )

    # 실제 로터 추력으로 경로를 난다. SetEntityPose 순간이동
    # (drone_path_player + drone_pose_controller) 은 폐기했다 — 물리엔진이 운동을
    # 보지 못해 IMU 가 죽고 자세가 항상 수평이라 스캔 시야가 고정됐다.
    # 파라미터 확정값과 근거: docs/3. 최적 드론 움직임.md
    drone_velocity_follower = Node(
        package="agconav_drone",
        executable="drone_velocity_follower",
        name="drone_velocity_follower",
        output="screen",
        parameters=[{
            "path_file": path_file,
            "cruise_speed_mps": ParameterValue(
                LaunchConfiguration("cruise_speed"), value_type=float),
            "use_sim_time": use_sim_time,
        }],
    )

    # agconav_description/models/agconav_drone/model.sdf의 os1_lidar_mount/
    # os1_lidar 링크 pose(둘 다 model 프레임 = drone/base_link 기준, relative_to
    # 없음)에서 그대로 가져온 고정 오프셋. 드론은 URDF/xacro + robot_state_publisher가
    # 아니라 SDF로 직접 스폰되는 구조라 이 구간을 자동으로 발행해주는 노드가 없어서
    # drone_elevation_mapper가 쓸 map -> ... -> drone/os1_lidar TF 체인을 완성하려면
    # 이 static publisher가 필요하다.
    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="drone_os1_lidar_static_tf",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "-0.175406",
            "--roll", "0", "--pitch", "1.5708", "--yaw", "0",
            "--frame-id", "drone/base_link",
            "--child-frame-id", "drone/os1_lidar",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    elevation_mapper_yaml = os.path.join(
        agconav_drone_share, "config", "drone_elevation_mapper.yaml"
    )

    drone_elevation_mapper = Node(
        package="agconav_drone",
        executable="drone_elevation_mapper",
        name="drone_elevation_mapper",
        output="screen",
        parameters=[
            elevation_mapper_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    elevation_map_saver_yaml = os.path.join(
        agconav_drone_share, "config", "elevation_map_saver.yaml"
    )

    elevation_map_saver = Node(
        package="agconav_drone",
        executable="elevation_map_saver",
        name="elevation_map_saver",
        output="screen",
        parameters=[
            elevation_map_saver_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            declare_world_name,
            declare_entity_name,
            declare_use_sim_time,
            declare_launch_gazebo,
            declare_path_file,
            declare_cruise_speed,
            set_gz_resource_path,
            gazebo,
            clock_bridge,
            drone_cmd_bridge,
            drone_enable_bridge,
            drone_velocity_follower,
            lidar_static_tf,
            drone_elevation_mapper,
            elevation_map_saver,
        ]
    )
