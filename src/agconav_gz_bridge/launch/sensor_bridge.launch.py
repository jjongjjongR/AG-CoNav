"""AG-CoNav 센서 브리지 — gz 센서 토픽을 모듈 계약 이름으로 정합한다.

세 로봇 모두 gz 센서를 ros_gz_bridge로 gz→ROS 변환하며 계약 이름으로 remap한다.
- drone/leg : 우리가 SDF/xacro로 직접 붙인 gz 센서.
- wheel     : clearpath a300 센서. robot.yaml에서 launch_enabled:false 라
              clearpath는 ROS로 안 올리므로, gz 토픽(/wheel/sensors/...)을 직접 브리지.

계약 이름 근거: 모듈 A(/drone/points), 모듈 B(/X/imu, /X/gps/fix), 모듈 C(/X/points).
※ leg IMU(/leg/imu)는 CHAMP EKF 의존성 때문에 go2_spawn.launch.py에서 이미 브리지됨.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='agconav_sensor_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # --- drone ---
            '/drone/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/drone/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            # --- leg (imu는 go2_spawn에서 이미 /leg/imu로 브리지됨) ---
            '/leg/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/leg/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            # --- wheel (clearpath gz 센서를 직접 브리지) ---
            '/wheel/sensors/lidar3d_0/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/wheel/sensors/imu_0/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/wheel/sensors/gps_0/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        ],
        remappings=[
            ('/drone/points/points', '/drone/points'),
            ('/drone/gps', '/drone/gps/fix'),
            ('/leg/points/points', '/leg/points'),
            ('/leg/gps', '/leg/gps/fix'),
            ('/wheel/sensors/lidar3d_0/scan/points', '/wheel/points'),
            ('/wheel/sensors/imu_0/data', '/wheel/imu'),
            ('/wheel/sensors/gps_0/navsat', '/wheel/gps/fix'),
        ],
    )

    return LaunchDescription([sensor_bridge])
