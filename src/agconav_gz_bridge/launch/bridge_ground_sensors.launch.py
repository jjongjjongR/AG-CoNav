import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('agconav_gz_bridge')

    # YAML 설정 파일 경로 지정
    wheel_sensors_config = os.path.join(pkg_dir, 'config', 'wheel_sensors_bridge.yaml')
    leg_sensors_config = os.path.join(pkg_dir, 'config', 'leg_sensors_bridge.yaml')
    wheel_lidar_config = os.path.join(pkg_dir, 'config', 'wheel_lidar_bridge.yaml')
    leg_lidar_config = os.path.join(pkg_dir, 'config', 'leg_lidar_bridge.yaml')

    # 1. Wheel 센서 (IMU, GPS, Odom)
    wheel_sensors_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_wheel_sensors',
        namespace='wheel',
        parameters=[
            {'config_file': wheel_sensors_config},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # 2. Leg 센서 (IMU, GPS, Odom)
    leg_sensors_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_leg_sensors',
        namespace='leg',
        parameters=[
            {'config_file': leg_sensors_config},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # 3. Wheel 라이다
    wheel_lidar_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_wheel_lidar',
        namespace='wheel',
        parameters=[
            {'config_file': wheel_lidar_config},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # 4. Leg 라이다
    leg_lidar_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_leg_lidar',
        namespace='leg',
        parameters=[
            {'config_file': leg_lidar_config},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    return LaunchDescription([
        wheel_sensors_node,
        leg_sensors_node,
        wheel_lidar_node,
        leg_lidar_node
    ])