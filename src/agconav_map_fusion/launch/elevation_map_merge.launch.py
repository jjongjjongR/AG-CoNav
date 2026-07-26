"""design.md 10: launches module E.

Unlike agconav_ground_mapping, all of design.md 6-1~6-5's responsibilities
(collect, validate, build output grid, merge, save) were folded into a
single node -- map_merge_collector -- so there is exactly one Node action
here and no per-robot namespace. This node's input/output topics are already
absolute (design.md "구현 시 참고사항": single system-wide node, not a
per-namespace reusable one).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'agconav_map_fusion'


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

    # README 3.4: every node runs with use_sim_time true, Gazebo /clock is
    # the only time source.
    use_sim_time = {'use_sim_time': True}

    map_merge_collector = Node(
        package=PACKAGE_NAME,
        executable='map_merge_collector',
        name='map_merge_collector',
        parameters=[
            os.path.join(config_dir, 'map_merge_collector.yaml'),
            use_sim_time,
        ],
        output='screen',
    )

    return LaunchDescription([map_merge_collector])
