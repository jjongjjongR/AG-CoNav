"""design.md 8: launches module D's 5 custom nodes for both wheel and leg.

ros_gz_bridge (wheel/leg) is owned by agconav_gz_bridge (CONTRIBUTING 4) and
is intentionally not launched here -- this file only brings up the 5 nodes
this package owns: ground_pointcloud_collector, ground_lidar_tf_transformer,
ground_elevation_mapper, ground_elevation_map_saver,
ground_completion_status_publisher.

Each node is launched twice, once per robot, in the `wheel`/`leg` namespace,
running the exact same code (design.md: "wheel과 leg는 반드시 같은 노드
구조(코드)를 네임스페이스로만 구분"). Only the parameter YAML file differs.

README 3.4: with Gazebo up, /clock is the only time source and this should
be run with `use_sim_time:=true`. It is a launch argument (default `false`)
rather than hardcoded so it can also be run against a system-clock source
with no Gazebo up (e.g. test/publish_fake_lidar.py) -- with use_sim_time
forced true and no /clock publisher, every timer in this package
(ground_pointcloud_collector's check_period_sec, ground_elevation_mapper's
publish_period_sec, ...) never fires.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = 'agconav_ground_mapping'
ROBOTS = ('wheel', 'leg')

# design.md 5-1: (executable, config-file-suffix) for each of the 5 nodes.
NODE_SPECS = (
    ('ground_pointcloud_collector', 'pointcloud_collector'),
    ('ground_lidar_tf_transformer', 'tf_transformer'),
    ('ground_elevation_mapper', 'elevation_mapper'),
    ('ground_elevation_map_saver', 'elevation_map_saver'),
    ('ground_completion_status_publisher', 'completion_status_publisher'),
)


def _robot_nodes(robot, config_dir, use_sim_time):
    return [
        Node(
            package=PACKAGE_NAME,
            executable=executable,
            name=executable,
            namespace=robot,
            parameters=[
                os.path.join(config_dir, f'{robot}_{config_suffix}.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        )
        for executable, config_suffix in NODE_SPECS
    ]


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=(
            'Use /clock (Gazebo) as the time source. Set true when Gazebo is '
            'up, leave false (default) for system-clock testing without it.'
        ),
    )
    # ParameterValue coerces the launch argument's string ("true"/"false")
    # into an actual bool, since use_sim_time is declared bool-typed.
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    nodes = []
    for robot in ROBOTS:
        nodes.extend(_robot_nodes(robot, config_dir, use_sim_time))

    return LaunchDescription([use_sim_time_arg, *nodes])
