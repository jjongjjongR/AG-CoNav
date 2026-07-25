"""Transforms a robot's raw LiDAR PointCloud2 into the map frame.

design.md 4-2 / 6-3 / 6-4 / 7-2 / 7-3: this node is launched once per robot
(wheel/leg), each in its own namespace, running the exact same code. It
subscribes to the raw sensor cloud directly (not via ground_pointcloud_collector,
which is monitoring-only and does not republish), looks up the sensor-to-map
TF at the cloud's own stamp, and republishes the cloud in the map frame with
the original stamp preserved.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class GroundLidarTfTransformer(Node):
    """Transforms one robot's PointCloud2 from its sensor frame into map."""

    def __init__(self):
        super().__init__('ground_lidar_tf_transformer')

        # CONTRIBUTING 5: relative topic names, resolved by the launch namespace
        # (e.g. /wheel/points -> /wheel/points_map, /leg/points -> /leg/points_map).
        self.declare_parameter('points_topic', 'points')
        self.declare_parameter('points_map_topic', 'points_map')
        # README 3.1: single global frame `map`.
        self.declare_parameter('target_frame', 'map')

        points_topic = self.get_parameter('points_topic').value
        points_map_topic = self.get_parameter('points_map_topic').value
        self._target_frame = self.get_parameter('target_frame').value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # design.md 7-1 / 7-3: best effort / volatile / keep last / depth 5,
        # for both the raw input cloud and the map-frame output cloud.
        points_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._points_sub = self.create_subscription(
            PointCloud2, points_topic, self._points_callback, points_qos)
        self._points_map_pub = self.create_publisher(
            PointCloud2, points_map_topic, points_qos)

        self.get_logger().info(
            f'Transforming "{points_topic}" -> "{points_map_topic}" '
            f'(target_frame="{self._target_frame}")')

    def _points_callback(self, msg):
        source_frame = msg.header.frame_id
        try:
            # design.md 7-2: look up the TF at the cloud's own measurement
            # stamp. No wait timeout: if it isn't available yet, drop this
            # cloud rather than blocking the single-threaded executor.
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, source_frame, Time.from_msg(msg.header.stamp))
        except TransformException as ex:
            self.get_logger().warn(
                f'TF lookup failed for "{source_frame}" -> '
                f'"{self._target_frame}" at {msg.header.stamp.sec}.'
                f'{msg.header.stamp.nanosec:09d}s, dropping cloud: {ex}')
            return

        # design.md 6-4 / 7-3: only frame_id changes, original stamp is kept.
        cloud_map = do_transform_cloud(msg, transform)
        cloud_map.header.stamp = msg.header.stamp
        cloud_map.header.frame_id = self._target_frame
        self._points_map_pub.publish(cloud_map)


def main(args=None):
    rclpy.init(args=args)
    node = GroundLidarTfTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
