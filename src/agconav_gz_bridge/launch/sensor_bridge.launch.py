"""AG-CoNav 센서 브리지 — gz 센서 토픽을 모듈 계약 이름으로 정합한다.

세 로봇 모두 gz 센서를 ros_gz_bridge로 gz→ROS 변환하며 계약 이름으로 remap한다.
- drone/leg : 우리가 SDF/xacro로 직접 붙인 gz 센서.
- wheel     : clearpath a300 센서. robot.yaml에서 launch_enabled:false 라
              clearpath는 ROS로 안 올리므로, gz 토픽(/wheel/sensors/...)을 직접 브리지.

계약 이름 근거: README §5 (/X/points, /X/imu, /X/gps) 및 모듈 C(/X/points).
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
            # LiDAR 점군: gz의 <topic>/points → 계약 /X/points
            ('/drone/points/points', '/drone/points'),
            ('/leg/points/points', '/leg/points'),
            ('/wheel/sensors/lidar3d_0/scan/points', '/wheel/points'),
            # GPS: 계약 이름 /X/gps (topics.md 센서 표 기준).
            # drone/leg는 gz 토픽이 이미 /X/gps라 remap 불필요.
            ('/wheel/sensors/gps_0/navsat', '/wheel/gps'),
            # IMU: wheel만 clearpath 이름 → 계약 /wheel/imu (drone/imu, leg/imu는 그대로)
            ('/wheel/sensors/imu_0/data', '/wheel/imu'),
        ],
    )

    # ---- TF prefix 리레이 : 사설 /X/tf(루트 프레임) → 전역 /tf(X/* 접두어) ----
    # 각 로봇 스택은 /leg/tf, /wheel/tf 에서 루트 프레임으로 내부 동작(CHAMP/clearpath 무손상).
    # 리레이가 공유 map만 빼고 접두어를 붙여 전역 /tf에 통합 뷰를 발행(모듈 E·RViz·크로스로봇용).
    leg_tf_relay = Node(
        package='agconav_gz_bridge',
        executable='tf_prefix_relay',
        name='leg_tf_prefix_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'prefix': 'leg',
            'input_tf': '/leg/tf',
            'input_tf_static': '/leg/tf_static',
            'shared_frames': ['map'],
        }],
    )
    wheel_tf_relay = Node(
        package='agconav_gz_bridge',
        executable='tf_prefix_relay',
        name='wheel_tf_prefix_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'prefix': 'wheel',
            'input_tf': '/wheel/tf',
            'input_tf_static': '/wheel/tf_static',
            'shared_frames': ['map'],
        }],
    )

    return LaunchDescription([sensor_bridge, leg_tf_relay, wheel_tf_relay])
