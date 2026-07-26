"""design.md 8: launches module D's 4 custom nodes for both wheel and leg.

ros_gz_bridge (wheel/leg) is owned by agconav_gz_bridge (CONTRIBUTING 4) and
is intentionally not launched here -- this file only brings up the 4 nodes
this package owns: ground_pointcloud_collector, ground_lidar_tf_transformer,
ground_elevation_mapper, ground_elevation_map_saver.

Each node is launched twice, once per robot, in the `wheel`/`leg` namespace,
running the exact same code (design.md: "wheel과 leg는 반드시 같은 노드
구조(코드)를 네임스페이스로만 구분"). Only the parameter YAML file differs.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'agconav_ground_mapping'
ROBOTS = ('wheel', 'leg')

# design.md 5-1: (executable, config-file-suffix) for each of the 4 nodes.
NODE_SPECS = (
    ('ground_pointcloud_collector', 'pointcloud_collector'),
    ('ground_lidar_tf_transformer', 'tf_transformer'),
    ('ground_elevation_mapper', 'elevation_mapper'),
    ('ground_elevation_map_saver', 'elevation_map_saver'),
)


def _robot_nodes(robot, config_dir):
    # README 3.4: every node runs with use_sim_time true, Gazebo /clock is
    # the only time source.
    use_sim_time = {'use_sim_time': True}
    return [
        Node(
            package=PACKAGE_NAME,
            executable=executable,
            name=executable,
            namespace=robot,
            parameters=[
                os.path.join(config_dir, f'{robot}_{config_suffix}.yaml'),
                use_sim_time,
            ],
            output='screen',
        )
        for executable, config_suffix in NODE_SPECS
    ]


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

    nodes = []
    for robot in ROBOTS:
        nodes.extend(_robot_nodes(robot, config_dir))

    return LaunchDescription(nodes)
