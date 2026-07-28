"""Republishes an external navigation-complete signal as elevation_map_status.

design.md 4-5 / 6-7 / 7-6: this node is launched once per robot (wheel/leg),
each in its own namespace, running the exact same code. It subscribes to
navigation_complete (published by an external navigation module) and, on a
complete signal, publishes elevation_map_status so the map-fusion module and
verification can tell accumulation is done. It does not touch accumulation
itself -- that stays in ground_elevation_mapper.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


class GroundCompletionStatusPublisher(Node):
    """Publishes elevation_map_status=True when navigation_complete fires."""

    def __init__(self):
        super().__init__('ground_completion_status_publisher')

        # CONTRIBUTING 5: relative topic names, resolved by the launch
        # namespace (e.g. /wheel/navigation_complete -> /wheel/elevation_map_status).
        self.declare_parameter('completion_topic', 'navigation_complete')
        self.declare_parameter('status_topic', 'elevation_map_status')

        completion_topic = self.get_parameter('completion_topic').value
        status_topic = self.get_parameter('status_topic').value

        # design.md 7-6: elevation_map_status is reliable + transient_local,
        # depth 1. navigation_complete is assumed to match (design.md 7-5).
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._completion_sub = self.create_subscription(
            Bool, completion_topic, self._completion_callback, status_qos)
        self._status_pub = self.create_publisher(Bool, status_topic, status_qos)

        self.get_logger().info(
            f'completion_topic="{completion_topic}" -> status_topic="{status_topic}"')

    def _completion_callback(self, msg):
        # design.md 4-5 / 7-6: publish completion status when the external
        # navigation module reports the robot's move is done.
        if msg.data:
            self._status_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = GroundCompletionStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
