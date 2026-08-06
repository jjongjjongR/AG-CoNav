import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetRemap

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_localization = LaunchConfiguration("use_localization", default="false")
    base_frame = "base_link"

    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim").find("unitree_go2_sim")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")
    
    joints_config = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        get_package_share_directory("agconav_bringup"),
        "config",
        "leg_controllers.yaml",
    )
    gait_config = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    links_config = os.path.join(unitree_go2_sim, "config/links/links.yaml")
    # 원본 Go2 xacro 대신, 센서 오버레이를 얹은 래퍼(agconav_description)를 사용한다.
    default_model_path = os.path.join(
        get_package_share_directory("agconav_description"),
        "urdf/leg/leg_with_sensors.urdf.xacro",
    )
    default_world_path = os.path.join(unitree_go2_description, "worlds/default.sdf")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="leg", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.25")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    declare_description_path = DeclareLaunchArgument(
        "unitree_go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )
    
    # Description nodes and parameters. 같은 control 파일을 robot_state_publisher,
    # CHAMP URDF 파서, Gazebo ros2_control 플러그인에 일관되게 전달한다.
    robot_description_content = Command([
        "xacro ",
        LaunchConfiguration("unitree_go2_description_path"),
        " ros_control_file:=",
        LaunchConfiguration("ros_control_file"),
    ])
    robot_description = {"robot_description": robot_description_content}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )
    
    # CHAMP controller nodes
    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": robot_description_content},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"publish_foot_contacts": False},
            {"close_loop_odom": True},
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": True},
            {"urdf": robot_description_content},
            joints_config,
            links_config,
            gait_config,
        ],
        # CHAMP state_estimation도 imu/data를 구독하므로 오버레이 imu(/leg/imu)로 연결
        remappings=[("imu/data", "leg/imu")],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": base_frame},
            {"use_sim_time": use_sim_time},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "base_to_footprint.yaml",
            ),
        ],
        # 외부 champ yaml의 imu0(imu/data)은 dict override가 안 먹으므로
        # remapping으로 오버레이 imu(/leg/imu)에 연결한다.
        remappings=[
            ("odometry/filtered", "odom/local"),
            ("imu/data", "leg/imu"),
        ],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"base_link_frame": "base_footprint"},
            {"odom_frame": "odom"},
            {"world_frame": "odom"},
            {"publish_tf": True},
            {"frequency": 50.0},
            {"two_d_mode": True},
            {"odom0": "odom/raw"},
            {"odom0_config": [False, False, False, False, False, False, True, True, False, False, False, True, False, False, False]},
            {"imu0": "leg/imu"},
            {"imu0_config": [False, False, False, False, False, True, False, False, False, False, False, True, False, False, False]},
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    # map_to_odom_tf_node was removed to guarantee it does not run.
    # Go2 URDF connection (base_footprint -> base_link)  
    base_footprint_to_base_link_tf_node = Node(
        package='tf2_ros',
        name='base_footprint_to_base_link_tf_node',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'
        ],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(unitree_go2_sim, "rviz/rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        # parameters=[{"use_sim_time": use_sim_time}]
    )
    
    # Spawn robot in Gazebo Sim
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('world_init_x'),
            '-y', LaunchConfiguration('world_init_y'),
            '-z', LaunchConfiguration('world_init_z'),
            '-Y', LaunchConfiguration('world_init_heading')
        ],
    )
    
    # Bridge ROS 2 topics to Gazebo Sim
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Gazebo to ROS
            # AG-CoNav: go2 원본 imu(/imu/data)는 os1 오버레이 imu(/leg/imu)로 대체한다.
            # CHAMP EKF 두 개의 imu0도 /leg/imu로 재배선했으므로 원본 imu 브리지는 비활성화.
            # '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/leg/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # AG-CoNav: go2 원본 3D LiDAR/카메라는 os1_lidar 오버레이(leg/points 등)로 대체되어 비활성화.
            # 브리지를 끊으면 velodyne/lidar_l1은 구독자가 없어(always_on 미설정=기본 false)
            # raycasting 자체가 멈춰 GPU 부하가 준다. rgb_camera는 always_on=1이라 렌더는 계속되나
            # ROS로는 나가지 않는다. 완전 정지는 외부 저장소 xacro 수정이 필요해 하지 않는다.
            # '/velodyne_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # '/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # '/velodyne_points@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # '/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image',
            
            # ROS to Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
        ],
    )
    
    # 시뮬레이션이 -r(unpaused)로 시작하므로 바로 active로 로드한다.
    controller_loader = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_states_controller",
            "joint_group_effort_controller",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "120",
        ],
    )
    
    # leg 스택은 CHAMP가 프레임 이름을 하드코딩(base_link 등)해서 frame_prefix를 못 쓴다.
    # 그래서 leg의 모든 노드 /tf·/tf_static 를 사설 토픽(/leg/tf)으로 remap해 격리한다.
    # 내부(CHAMP/EKF)는 루트 프레임 그대로 정상 동작하고, 전역 /tf 오염을 막는다.
    # 전역 /tf에는 tf_prefix_relay가 leg/* 접두어를 붙여 따로 발행한다(agconav_gz_bridge).
    leg_stack = GroupAction([
        SetRemap('/tf', '/leg/tf'),
        SetRemap('/tf_static', '/leg/tf_static'),

        # Gazebo and robot nodes first
        robot_state_publisher_node,
        gazebo_spawn_robot,
        gazebo_bridge,

        # CHAMP controller nodes
        quadruped_controller_node,
        state_estimator_node,

        # EKF nodes for localization
        base_to_footprint_ekf,
        footprint_to_odom_ekf,

        # TF publishers for frame connections
        base_footprint_to_base_link_tf_node,

        # Controller loader — 시뮬레이션이 unpaused이므로 바로 활성화.
        controller_loader,

        # Visualization (only if rviz flag is set)
        rviz2,
    ])

    return LaunchDescription(
        [
            # Launch arguments
            declare_use_sim_time,
            DeclareLaunchArgument("use_localization", default_value="false"),
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path,

            leg_stack,
        ]
    )
