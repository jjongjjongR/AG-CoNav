"""모듈 F 실행 묶음 — 드론 2.5D 지도 → wheel/leg 주행 가능 맵 + 저장.

지형 특성 계산 → wheel·leg 판정 → (완료 상태 둘 다 True) → 지도 저장 제어 →
map_saver_server. map_saver_server는 lifecycle 노드라 nav2_lifecycle_manager가
autostart로 configure·activate 시킨다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = os.path.join(
    get_package_share_directory('agconav_traversability'), 'config')


def generate_launch_description():
    terrain_feature_calculator = Node(
        package='agconav_traversability',
        executable='terrain_feature_calculator',
        name='terrain_feature_calculator',
        output='screen',
        parameters=[os.path.join(CONFIG, 'terrain_feature_calculator.yaml')],
    )

    # wheel·leg는 같은 노드를 통과 기준 yaml만 달리해서 각각 실행한다.
    verdictors = [
        Node(
            package='agconav_traversability',
            executable='traversability_verdictor',
            name=f'traversability_verdictor_{robot}',
            output='screen',
            parameters=[os.path.join(CONFIG, f'traversability_{robot}.yaml')],
        )
        for robot in ('wheel', 'leg')
    ]

    map_save_coordinator = Node(
        package='agconav_traversability',
        executable='map_save_coordinator',
        name='map_save_coordinator',
        output='screen',
        parameters=[os.path.join(CONFIG, 'map_save_coordinator.yaml')],
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[os.path.join(CONFIG, 'map_saver.yaml')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_saver'],
        }],
    )

    return LaunchDescription([
        terrain_feature_calculator,
        *verdictors,
        map_save_coordinator,
        map_saver,
        lifecycle_manager,
    ])
