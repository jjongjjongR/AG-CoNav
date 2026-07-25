"""Subscribes to a robot's LiDAR PointCloud2 topic and watches for data dropout.

design.md 4-1: this node is launched once per robot (wheel/leg), each in its
own namespace, running the exact same code. It only stores the latest
PointCloud2 and reports whether the topic is still alive; it does not
republish or transform the cloud.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class GroundPointCloudCollector(Node):
    """Stores the latest PointCloud2 for one robot and checks receipt."""

    def __init__(self):
        super().__init__('ground_pointcloud_collector')

        # CONTRIBUTING 5: use a relative topic name, resolved by the
        # launch-provided namespace (e.g. /wheel/points, /leg/points).
        self.declare_parameter('points_topic', 'points')
        self.declare_parameter('data_timeout_sec', 2.0)
        self.declare_parameter('check_period_sec', 1.0)

        points_topic = self.get_parameter('points_topic').value
        self._data_timeout = Duration(
            seconds=self.get_parameter('data_timeout_sec').value)

        # design.md 7-1: best effort / volatile / keep last / depth 5.
        points_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._latest_cloud = None
        self._last_received = None

        self._points_sub = self.create_subscription(
            PointCloud2, points_topic, self._cloud_callback, points_qos)

        check_period = self.get_parameter('check_period_sec').value
        self._check_timer = self.create_timer(
            check_period, self._check_data_received)

        self.get_logger().info(
            f'Listening for point clouds on "{points_topic}"')

    def _cloud_callback(self, msg):
        self._latest_cloud = msg
        self._last_received = self.get_clock().now()

    def get_latest_cloud(self):
        """Return the most recently received PointCloud2, or None."""
        return self._latest_cloud

    def _check_data_received(self):
        if self._last_received is None:
            self.get_logger().warn('no point cloud received yet.')
            return
        elapsed = self.get_clock().now() - self._last_received
        if elapsed > self._data_timeout:
            self.get_logger().warn(
                f'no point cloud in the last {elapsed.nanoseconds / 1e9:.2f}s '
                '(topic may be stalled).')


def main(args=None):
    rclpy.init(args=args)
    node = GroundPointCloudCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
