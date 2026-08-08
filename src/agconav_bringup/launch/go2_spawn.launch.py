import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

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
    # ParameterValue(value_type=str)로 감싸는 것이 중요하다.
    # 감싸지 않으면 launch_ros가 xacro 출력(URDF 원문)을 YAML로 파싱하려 들고,
    # 주석 등에 "낱말: 낱말" 꼴이 하나라도 있으면
    #   "Unable to parse the value of parameter robot_description as yaml"
    # 예외로 **launch 전체가 즉시 종료**된다. 원래 Go2 xacro에 우연히 그런
    # 문자열이 없어 통과하고 있었을 뿐이라, URDF에 주석 한 줄만 추가해도
    # 시뮬이 통째로 안 뜨는 지뢰였다(실측: 미사용 센서 비활성화 주석에서 발생).
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("unitree_go2_description_path"),
            " ros_control_file:=",
            LaunchConfiguration("ros_control_file"),
        ]),
        value_type=str,
    )
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
        # CHAMP는 상대 이름 "cmd_vel/smooth"를 구독하는데 leg 스택은 루트
        # 네임스페이스에서 뜨므로 그대로 두면 /cmd_vel/smooth가 된다.
        # 원본 launch는 이를 /cmd_vel로 돌려놨지만, 계약(topics.md §3)은
        # /leg/cmd_vel 이고 모듈 C(Nav2)도 네임스페이스 안에서 /leg/cmd_vel로
        # 낸다. /cmd_vel로 두면 발행자가 없어 leg가 영영 움직이지 않는다.
        remappings=[("/cmd_vel/smooth", "/leg/cmd_vel")],
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
            # yaml 뒤에 둬야 champ 기본값(50 Hz)을 덮는다.
            # footprint_to_odom_ekf와 같은 이유로 30 Hz로 낮춘다 —
            # 이 월드에서는 50 Hz 주기를 못 지켜 "Failed to meet update rate"가 난다.
            {"frequency": 30.0},
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
            # 50 Hz면 주기(20 ms) 안에 못 끝내고
            #   "Failed to meet update rate! Took 0.064 seconds"
            # 를 간헐적으로 낸다(성동구 월드 + 라이다 3대라 머신이 포화 상태).
            # 이 EKF 출력 /leg/odom을 받는 쪽은 모듈 B의 EKF(30 Hz)와 TF 소비자들뿐이라
            # 30 Hz면 충분하다. 라이다가 10 Hz이므로 TF 해상도도 남는다.
            {"frequency": 30.0},
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
    #
    # additional_env의 ROS_HOME이 핵심이다.
    # ros2_control spawner는 `$ROS_HOME/locks/ros2-control-controller-spawner.lock`
    # 하나를 **모든 spawner가 공유**하고, 그 락을 잡은 채로 자기 controller_manager를
    # 기다린다(락 획득이 CM 대기보다 먼저다). 다른 spawner는 20초 x 5회 + 3초 x 4
    # = 최대 112초만 기다리다 exit 1로 죽는다.
    # leg는 `--controller-manager-timeout 120`이라 최대 120초를 잡고 있을 수 있어,
    # leg의 controller_manager가 늦게 뜨는 실행에서는 wheel의 spawner
    # (joint_state_broadcaster / platform_velocity_controller)가 락을 못 얻고 죽는다.
    # 그러면 /wheel/joint_states가 없어 robot_state_publisher가 TF를 못 내고
    # wheel/base_link 자체가 사라진다 — 실행마다 되기도 하고 안 되기도 한 원인.
    # 두 로봇은 서로 다른 controller_manager를 쓰므로 락을 공유할 이유가 없다.
    # leg 쪽만 락 디렉터리를 분리해 경쟁을 없앤다.
    controller_loader = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_states_controller",
            "joint_group_effort_controller",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "120",
            # 서비스 호출 자체의 제한 시간. 기본 10초로는 부족하다.
            # controller_manager를 "찾는" 시간(--controller-manager-timeout)과 별개로,
            # load_controller 요청에 응답이 오기까지의 시간이다. Go2는 관절이 12개라
            # gz_ros_control이 하드웨어를 초기화하는 동안 CM이 서비스 콜백을 못 돌린다.
            # 실측: 10초를 넘겨
            #   [FATAL] Failed loading controller joint_states_controller
            # 로 spawner가 죽고, 그러면 leg에 컨트롤러가 없어 CHAMP 명령이 관절까지
            # 못 간다(= /leg/cmd_vel을 넣어도 로봇이 0.000 m 움직인다).
            # 10회 중 1회 재현됐다.
            "--service-call-timeout", "60",
        ],
        additional_env={
            "ROS_HOME": os.path.join(os.path.expanduser("~"), ".ros", "agconav_leg"),
        },
    )
    
    # leg 스택은 CHAMP가 프레임 이름을 하드코딩(base_link 등)해서 frame_prefix를 못 쓴다.
    # 그래서 leg의 모든 노드 /tf·/tf_static 를 사설 토픽(/leg/tf)으로 remap해 격리한다.
    # 내부(CHAMP/EKF)는 루트 프레임 그대로 정상 동작하고, 전역 /tf 오염을 막는다.
    # 전역 /tf에는 tf_prefix_relay가 leg/* 접두어를 붙여 따로 발행한다(agconav_gz_bridge).
    leg_stack = GroupAction([
        SetRemap('/tf', '/leg/tf'),
        SetRemap('/tf_static', '/leg/tf_static'),
        # 모듈 B 구성요소 4: CHAMP의 원본 odometry는 /leg/odom 이어야 한다.
        # 기본값(전역 /odom)이면 wheel과 이름이 겹치고 EKF의 odom0 입력이 비어
        # map -> leg/odom TF가 발행되지 않는다.
        SetRemap('/odom', '/leg/odom'),

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
