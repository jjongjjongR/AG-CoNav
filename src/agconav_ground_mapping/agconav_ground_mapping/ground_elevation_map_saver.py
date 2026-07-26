"""Saves a robot's latest elevation map to an mcap rosbag2 on completion.

design.md 4-4 / 6-6 / 7-5: this node is launched once per robot (wheel/leg),
each in its own namespace, running the exact same code. It keeps the most
recently received elevation_map (GridMap) and, when a navigation_complete
signal arrives, serializes that map into a single-topic rosbag2 (mcap).
Saving happens directly from the completion topic's callback, not via a
service call -- a manual save Service (std_srvs/Trigger) is available too,
for debugging/reproduction, but stays off unless explicitly enabled.
"""

import os

from grid_map_msgs.msg import GridMap
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class GroundElevationMapSaver(Node):
    """Buffers the latest elevation_map and writes it to mcap on completion."""

    def __init__(self):
        super().__init__('ground_elevation_map_saver')

        # design.md 7-5: map_name defaults from the launch namespace, e.g.
        # /wheel -> "wheel_elevation_map", /leg -> "leg_elevation_map".
        namespace = self.get_namespace().strip('/')
        default_map_name = f'{namespace}_elevation_map' if namespace else 'elevation_map'

        # CONTRIBUTING 5: relative topic names, resolved by the launch
        # namespace. use_sim_time is declared automatically by rclpy.Node,
        # not redeclared here (README 3.4: set true from launch).
        self.declare_parameter('input_topic', 'elevation_map')
        self.declare_parameter('completion_topic', 'navigation_complete')
        # design.md 7-5 marks the completion message type "확정 필요" and says
        # to assume std_msgs/msg/Bool for now. This parameter only records
        # that assumption for visibility; the subscribed type is fixed in
        # code below, since switching it dynamically isn't needed yet.
        self.declare_parameter('completion_type', 'std_msgs/msg/Bool')
        self.declare_parameter('output_directory', 'maps')
        self.declare_parameter('map_name', default_map_name)
        self.declare_parameter('output_format', 'mcap')
        self.declare_parameter('enable_manual_save_service', False)

        self._input_topic = self.get_parameter('input_topic').value
        self._completion_topic = self.get_parameter('completion_topic').value
        self._output_directory = self.get_parameter('output_directory').value
        self._map_name = self.get_parameter('map_name').value
        self._output_format = self.get_parameter('output_format').value

        self._latest_elevation_map = None

        # design.md 7-4: elevation_map is published reliable / transient_local
        # / keep_last / depth 1 -- match that here.
        elevation_map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # design.md 7-5: assume reliable / transient_local until the external
        # navigation module's actual QoS is confirmed.
        completion_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._elevation_map_sub = self.create_subscription(
            GridMap, self._input_topic, self._elevation_map_callback, elevation_map_qos)
        self._completion_sub = self.create_subscription(
            Bool, self._completion_topic, self._completion_callback, completion_qos)

        self._save_service = None
        if self.get_parameter('enable_manual_save_service').value:
            self._save_service = self.create_service(
                Trigger, 'save_elevation_map', self._manual_save_callback)

        self.get_logger().info(
            f'input_topic="{self._input_topic}", '
            f'completion_topic="{self._completion_topic}" '
            f'(type assumed: {self.get_parameter("completion_type").value}), '
            f'output="{os.path.join(self._output_directory, self._map_name)}" '
            f'({self._output_format})')

    def _elevation_map_callback(self, msg):
        self._latest_elevation_map = msg

    def _completion_callback(self, msg):
        # design.md 6-6: the completion topic's callback triggers the save
        # directly, no service call involved. Only an actual "complete"
        # signal (data == True) triggers it.
        if msg.data:
            self._save_elevation_map()

    def _manual_save_callback(self, request, response):
        response.success, response.message = self._save_elevation_map()
        return response

    def _save_elevation_map(self):
        if self._latest_elevation_map is None:
            message = 'no elevation_map received yet, nothing to save.'
            self.get_logger().warn(message)
            return False, message

        bag_path = os.path.join(self._output_directory, self._map_name)
        try:
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id=self._output_format),
                rosbag2_py.ConverterOptions('', ''),
            )
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=self._input_topic,
                type='grid_map_msgs/msg/GridMap',
                serialization_format='cdr',
            ))
            stamp = self._latest_elevation_map.header.stamp
            timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            writer.write(
                self._input_topic,
                serialize_message(self._latest_elevation_map),
                timestamp_ns,
            )
            del writer  # flush/close the bag now, rather than at GC time
        except Exception as ex:
            message = f'failed to save elevation map to "{bag_path}": {ex}'
            self.get_logger().error(message)
            return False, message

        message = f'saved elevation map to "{bag_path}"'
        self.get_logger().info(message)
        return True, message


def main(args=None):
    rclpy.init(args=args)
    node = GroundElevationMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
