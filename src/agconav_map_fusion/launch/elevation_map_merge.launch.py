"""design.md 10: launches module E's 3 nodes.

design.md 6-1~6-5 / "노드 간 연결 방식": collect+validate (incl. grid
alignment), merge, and save are 3 separate nodes (processes), connected only
by topics (map_merge_collector -> /merged/merge_trigger ->
elevation_map_merger -> /merged/elevation_map -> merged_elevation_map_saver)
-- not by any in-process function call. None of the 3 run per-robot
namespaces; all subscribe/publish on absolute topics (design.md "구현 시
참고사항": single system-wide instances, not per-namespace reusable nodes).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'agconav_map_fusion'
# design.md 7-1: (executable, config-file-name) for each of the 3 nodes.
NODE_SPECS = (
    ('map_merge_collector', 'map_merge_collector.yaml'),
    ('elevation_map_merger', 'elevation_map_merger.yaml'),
    ('merged_elevation_map_saver', 'merged_elevation_map_saver.yaml'),
)


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

    # README 3.4: every node runs with use_sim_time true, Gazebo /clock is
    # the only time source.
    use_sim_time = {'use_sim_time': True}

    nodes = [
        Node(
            package=PACKAGE_NAME,
            executable=executable,
            name=executable,
            parameters=[
                os.path.join(config_dir, config_file),
                use_sim_time,
            ],
            output='screen',
        )
        for executable, config_file in NODE_SPECS
    ]

    return LaunchDescription(nodes)
