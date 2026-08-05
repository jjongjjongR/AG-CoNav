"""Saves the merged elevation map to an mcap rosbag2 on receipt.

design.md 6-5 / 9-7: a single, system-wide node (not per-namespace), the
last stage of module E's 3-node topic pipeline (see design.md "노드 간 연결
방식"). elevation_map_merger only ever publishes /merged/elevation_map once
(design.md: the merge itself fires exactly once, ever), so this node's own
arrival-of-a-message IS the completion signal -- unlike
agconav_ground_mapping's ground_elevation_map_saver, no separate
completion/trigger topic is needed here.

The mcap save logic (rosbag2_py.SequentialWriter) mirrors
agconav_ground_mapping's ground_elevation_map_saver, reimplemented rather
than imported (CONTRIBUTING 5: modules only couple through topics, not by
importing another package's internals).
"""

import os

from grid_map_msgs.msg import GridMap
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py


class MergedElevationMapSaver(Node):
    """Writes /merged/elevation_map to mcap the moment it arrives."""

    def __init__(self):
        super().__init__('merged_elevation_map_saver')

        # design.md "구현 시 참고사항": absolute topic, single system-wide node.
        self.declare_parameter('input_topic', '/merged/elevation_map')
        self.declare_parameter('output_directory', 'maps')
        self.declare_parameter('map_name', 'merged_elevation_map')
        self.declare_parameter('output_format', 'mcap')

        self._input_topic = self.get_parameter('input_topic').value
        self._output_directory = self.get_parameter('output_directory').value
        self._map_name = self.get_parameter('map_name').value
        self._output_format = self.get_parameter('output_format').value

        # design.md 9-5: elevation_map_merger publishes reliable /
        # transient_local / keep_last / depth 1 -- match that here.
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map_sub = self.create_subscription(
            GridMap, self._input_topic, self._map_callback, map_qos)

        self.get_logger().info(
            f'input_topic="{self._input_topic}", '
            f'output="{os.path.join(self._output_directory, self._map_name)}" '
            f'({self._output_format})')

    def _map_callback(self, msg):
        self._save_merged_map(msg)

    def _save_merged_map(self, merged_map):
        """Serialize merged_map to an mcap rosbag2 (design.md 6-5 / 9-7).

        Never raises out of this method -- a save failure must not take
        down a node whose only job is reacting to further incoming maps.
        """
        bag_path = os.path.join(self._output_directory, self._map_name)
        # Independent restart guard, alongside elevation_map_merger's own
        # (see that node's _merge_done): don't trust that side alone to
        # prevent a second save, so check here too -- if the output already
        # exists, a previous run already saved it, so skip rather than
        # overwrite.
        if os.path.exists(bag_path):
            self.get_logger().warn(
                f'"{bag_path}" already exists -- not overwriting (likely a '
                'restart after a previous successful save), skipping this save.')
            return
        try:
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id=self._output_format),
                rosbag2_py.ConverterOptions('', ''),
            )
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=0,
                name=self._input_topic,
                type='grid_map_msgs/msg/GridMap',
                serialization_format='cdr',
            ))
            stamp = merged_map.header.stamp
            timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            writer.write(self._input_topic, serialize_message(merged_map), timestamp_ns)
            del writer  # flush/close the bag now, rather than at GC time
        except Exception as ex:
            self.get_logger().error(f'failed to save merged map to "{bag_path}": {ex}')
            return

        self.get_logger().info(f'saved merged map to "{bag_path}" ({self._output_format})')


def main(args=None):
    rclpy.init(args=args)
    node = MergedElevationMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
