import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('agconav_localization')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace (e.g., wheel or leg)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_odom_topic_cmd = DeclareLaunchArgument(
        'odom_topic',
        default_value='odom',
        description='Local odometry topic for EKF odom0 input')

    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg_dir, 'config', 'navsat.yaml')

    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace,
        output='screen',
        parameters=[
            ekf_config,
            {
                'use_sim_time': use_sim_time,
                'odom0': LaunchConfiguration('odom_topic'),
            }
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered'),
            ('set_pose', 'set_pose'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Navsat Transform Node
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        namespace=namespace,
        output='screen',
        parameters=[
            navsat_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('gps/fix', 'gps'),               # Subscribe to /X/gps
            ('imu', 'imu'),                   # Subscribe to /X/imu
            ('odometry/filtered', 'odometry/filtered'), # Subscribe to EKF output
            ('odometry/gps', 'odometry/gps'), # Publish to /X/odometry/gps
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_odom_topic_cmd)
    ld.add_action(ekf_node)
    ld.add_action(navsat_transform_node)

    return ld
