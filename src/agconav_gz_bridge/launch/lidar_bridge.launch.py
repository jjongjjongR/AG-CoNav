import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # agconav_navigation 패키지 디렉토리 경로 가져오기
    pkg_dir = get_package_share_directory('agconav_navigation')

    # 1. leg와 wheel의 라이다 YAML 설정 파일 경로 각각 지정
    leg_config = os.path.join(pkg_dir, 'config', 'leg_lidar_bridge.yaml')
    wheel_config = os.path.join(pkg_dir, 'config', 'wheel_lidar_bridge.yaml')

    # 2. leg 라이다 브릿지 노드 생성
    leg_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_leg_lidar',
        parameters=[{'config_file': leg_config}],
        output='screen'
    )

    # 3. wheel 라이다 브릿지 노드 생성
    wheel_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_wheel_lidar',
        parameters=[{'config_file': wheel_config}],
        output='screen'
    )

    # 4. 두 노드를 모두 LaunchDescription에 담아서 반환
    return LaunchDescription([
        leg_bridge_node,
        wheel_bridge_node
    ])