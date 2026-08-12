import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_localization = LaunchConfiguration("use_localization")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    world = LaunchConfiguration("world")

    agconav_worlds_share = get_package_share_directory("agconav_worlds")
    agconav_bringup_share = get_package_share_directory("agconav_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="leg_test_bumps.world",
        description="World file to load (e.g., leg_test_bumps.world or leg_test_ramps.world)",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation time",
    )

    declare_use_nav2 = DeclareLaunchArgument(
        "use_nav2",
        default_value="True",
        description="Launch Nav2 for leg"
    )

    declare_use_localization = DeclareLaunchArgument(
        "use_localization",
        default_value="True",
        description="Launch EKF and Navsat nodes for Module B",
    )

    declare_walking_algo = DeclareLaunchArgument(
        "walking_algo",
        default_value="champ",
        description="Walking algorithm to use: champ or nmpc"
    )

    declare_nav2_params_file = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(agconav_bringup_share, "config", "leg_test_nav2.yaml"),
        description="Common Nav2 params for leg",
    )

    # Leg spawn arguments - start at origin
    spawn_defaults = (
        ("leg_x", "0.0", "Go2(leg) spawn x [m]"),
        ("leg_y", "0.0", "Go2(leg) spawn y [m]"),
        ("leg_z", "0.5", "Go2(leg) spawn z [m]"),
        ("leg_yaw", "0.0", "Go2(leg) spawn heading [rad]"),
    )
    declare_spawn_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in spawn_defaults
    ]

    _gz_models_dirs = [
        os.path.join(get_package_share_directory("agconav_description"), "models"),
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
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": [os.path.join(agconav_worlds_share, "worlds", ""), world, " -r"]}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="agconav_clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    spawn_leg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(agconav_bringup_share, "launch", "leg_test_spawn.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_localization": use_localization,
            "walking_algo": LaunchConfiguration("walking_algo"),
            "rviz": "False", # Disable default go2 rviz, we will use Nav2's RViz
            "robot_name": "leg",
            "world_init_x": LaunchConfiguration("leg_x"),
            "world_init_y": LaunchConfiguration("leg_y"),
            "world_init_z": LaunchConfiguration("leg_z"),
            "world_init_heading": LaunchConfiguration("leg_yaw"),
            "ros_control_file": os.path.join(agconav_bringup_share, "config", "leg_controllers.yaml"),
        }.items(),
    )

    # Leg specific Sensor Bridge and TF Relay (OS1 Pointcloud + GPS)
    # Excluded Velodyne explicitly as requested.
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='leg_sensor_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/leg/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/leg/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        ],
        remappings=[
            # We don't need to remap since both Gazebo and ROS use /leg/points now
        ],
    )

    leg_tf_relay = Node(
        package='agconav_gz_bridge',
        executable='tf_prefix_relay',
        name='leg_tf_prefix_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'prefix': 'leg',
            'input_tf': '/leg/tf',
            'input_tf_static': '/leg/tf_static',
            'shared_frames': ['map'],
        }],
    )

    # Localization (EKF)
    localization_leg = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("agconav_localization"), "launch", "localization.launch.py")),
                launch_arguments={
                    "namespace": "leg",
                    "use_sim_time": use_sim_time,
                    "odom_topic": "/leg/odom",
                }.items(),
            )
        ],
        condition=IfCondition(use_localization)
    )

    # Navigation (Nav2 + RViz2 + Ground Segmentation)
    nav2_leg = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")),
                launch_arguments={
                    "namespace": "leg",
                    "use_namespace": "True",
                    "use_sim_time": use_sim_time,
                    "use_localization": "True", # Explicitly set to avoid Nav2 Python eval issues
                    "params_file": nav2_params_file,
                    "use_composition": "False",
                    "use_collision_monitor": "False",
                    "autostart": "True",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, "launch", "rviz_launch.py")),
                launch_arguments={
                    "namespace": "leg",
                    "use_namespace": "True",
                    "rviz_config": os.path.join(nav2_bringup_share, "rviz", "nav2_default_view.rviz")
                }.items(),
            ),
            Node(
                package="agconav_navigation",
                executable="ground_segmentation_node",
                name="ground_segmentation_node",
                namespace="leg",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, {"odom_frame": "odom"}]
            ),
            Node(
                package="agconav_navigation",
                executable="navigation_complete_node",
                name="navigation_complete",
                namespace="leg",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}]
            )
        ],
        condition=IfCondition(use_nav2)
    )

    return LaunchDescription(
        [
            declare_world,
            declare_use_sim_time,
            declare_use_localization,
            declare_use_nav2,
            declare_walking_algo,
            declare_nav2_params_file,
            *declare_spawn_args,
            set_gz_resource_path,
            gazebo,
            clock_bridge,
            spawn_leg,
            sensor_bridge,
            leg_tf_relay,
            localization_leg,
            nav2_leg,
        ]
    )
