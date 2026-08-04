"""Saves a robot's elevation map to an mcap rosbag2 as soon as it arrives.

design.md 4-2 / 6-4 / 6-6 / 7-3: this node is launched once per robot
(wheel/leg), each in its own namespace, running the exact same code. It no
longer subscribes to navigation_status itself -- ground_elevation_mapper now
only publishes elevation_map once, exactly when navigation reports
completion (design.md 4-1), so receiving elevation_map already means
"move complete + final map ready". Saving happens directly from that
callback, not via a service call -- a manual save Service (std_srvs/Trigger)
is available too, for debugging/reproduction, but stays off unless
explicitly enabled.

This replaces the previous design where saver and mapper each independently
subscribed to the completion topic: with no ordering guarantee between two
independent subscribers, saver could try to save before mapper had published
the final map -- the same class of race previously solved by folding
ground_completion_status_publisher into this node (design.md 4-2 note
below). This time the fix runs the other direction: the publish side
(mapper) only emits once it has the completion signal, and the save side
(saver) treats that single emission itself as the trigger.

design.md 4-2 / 6-7 / 7-5: once the save actually succeeds, this node also
publishes elevation_map_status=True itself, in the same callback, after the
file is on disk. This absorbs the responsibility that used to live in the
now-removed ground_completion_status_publisher node, which republished
navigation_complete independently and raced against this node's save --
elevation_map_status could go out before the bag file existed.
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

        # design.md 6-7: map_name defaults from the launch namespace, e.g.
        # /wheel -> "wheel_elevation_map", /leg -> "leg_elevation_map".
        namespace = self.get_namespace().strip('/')
        default_map_name = f'{namespace}_elevation_map' if namespace else 'elevation_map'

        # CONTRIBUTING 5: relative topic names, resolved by the launch
        # namespace. use_sim_time is declared automatically by rclpy.Node,
        # not redeclared here (README 3.4: set true from launch).
        self.declare_parameter('input_topic', 'elevation_map')
        self.declare_parameter('output_directory', 'maps')
        self.declare_parameter('map_name', default_map_name)
        self.declare_parameter('output_format', 'mcap')
        self.declare_parameter('enable_manual_save_service', False)
        # design.md 4-2 / 6-7: status_topic used to belong to the now-removed
        # ground_completion_status_publisher node.
        self.declare_parameter('status_topic', 'elevation_map_status')

        self._input_topic = self.get_parameter('input_topic').value
        self._output_directory = self.get_parameter('output_directory').value
        self._map_name = self.get_parameter('map_name').value
        self._output_format = self.get_parameter('output_format').value
        self._status_topic = self.get_parameter('status_topic').value

        self._latest_elevation_map = None

        # design.md 7-3: elevation_map is published reliable / transient_local
        # / keep_last / depth 1 -- match that here.
        elevation_map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # design.md 7-5: elevation_map_status is reliable / transient_local /
        # keep_last / depth 1.
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._elevation_map_sub = self.create_subscription(
            GridMap, self._input_topic, self._elevation_map_callback, elevation_map_qos)
        self._status_pub = self.create_publisher(Bool, self._status_topic, status_qos)

        self._save_service = None
        if self.get_parameter('enable_manual_save_service').value:
            self._save_service = self.create_service(
                Trigger, 'save_elevation_map', self._manual_save_callback)

        self.get_logger().info(
            f'input_topic="{self._input_topic}", '
            f'output="{os.path.join(self._output_directory, self._map_name)}" '
            f'({self._output_format}), '
            f'status_topic="{self._status_topic}"')

    def _elevation_map_callback(self, msg):
        # design.md 6-4 / 6-6: elevation_map is now published exactly once, by
        # ground_elevation_mapper, only after navigation_status reports
        # completion -- so receiving it here already means "move complete +
        # final map ready", and triggers the save directly, no separate
        # completion topic or service call involved.
        self._latest_elevation_map = msg
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
                id=0,
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
        self._status_pub.publish(Bool(data=True))
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
