#!/usr/bin/env python3
"""Manual validation script: feeds fake wheel/leg LiDAR + TF into the pipeline.

Not a pytest test (no `test_` prefix, not collected by pytest) and not a
production node -- it exists purely so a developer can run
`ground_pointcloud_collector` / `ground_lidar_tf_transformer` /
`ground_elevation_mapper` against something without needing wheel/leg's real
Gazebo + LiDAR bridge up. Run it directly:

    python3 test/publish_fake_lidar.py

At 2 Hz it publishes a small random PointCloud2 on /wheel/points and
/leg/points (frame_id "wheel/os1_lidar" / "leg/os1_lidar"), matching
ground_pointcloud_collector's and ground_lidar_tf_transformer's points_qos
(best effort / volatile / keep last / depth 5, design.md 7-1). It also
statically broadcasts, once, the map -> {robot}/odom -> {robot}/base_link ->
{robot}/os1_lidar TF chain for both robots (near-origin, arbitrary offsets)
so ground_lidar_tf_transformer's lookup_transform succeeds. After 5 seconds
it publishes Bool(True) once on each robot's navigation_complete topic,
matching ground_elevation_map_saver's completion_topic subscription QoS
(reliable / transient_local / keep last / depth 1, design.md 7-6).

The node keeps spinning after publishing so points keep flowing and the
static TF stays available; stop it with Ctrl+C.
"""

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header
from tf2_ros import StaticTransformBroadcaster

ROBOTS = ('wheel', 'leg')
POINTS_HZ = 2.0
NUM_POINTS = 40
POINT_SPREAD = 1.0  # meters, x/y drawn from [-POINT_SPREAD, POINT_SPREAD]
POINT_HEIGHT = 0.5  # meters, z drawn from [0, POINT_HEIGHT]
NAV_COMPLETE_DELAY_SEC = 5.0

# Arbitrary near-origin offsets per TF chain link, distinct per robot so the
# two chains are easy to tell apart in RViz2/tf2_echo. (x, y, z) meters,
# identity rotation throughout.
TF_CHAIN_OFFSETS = {
    'wheel': {
        'odom': (0.0, 0.0, 0.0),        # map -> wheel/odom
        'base_link': (0.5, 0.0, 0.0),   # wheel/odom -> wheel/base_link
        'os1_lidar': (0.0, 0.0, 0.3),   # wheel/base_link -> wheel/os1_lidar
    },
    'leg': {
        'odom': (0.0, 1.0, 0.0),        # map -> leg/odom
        'base_link': (0.3, 0.0, 0.0),   # leg/odom -> leg/base_link
        'os1_lidar': (0.0, 0.0, 0.25),  # leg/base_link -> leg/os1_lidar
    },
}


def _build_static_transforms(stamp):
    """Return the 6 TransformStamped messages for both robots' TF chains."""
    transforms = []
    for robot, offsets in TF_CHAIN_OFFSETS.items():
        chain = (
            ('map', f'{robot}/odom', offsets['odom']),
            (f'{robot}/odom', f'{robot}/base_link', offsets['base_link']),
            (f'{robot}/base_link', f'{robot}/os1_lidar', offsets['os1_lidar']),
        )
        for parent, child, (x, y, z) in chain:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            transforms.append(t)
    return transforms


def _build_fake_points(rng):
    """Return NUM_POINTS random (x, y, z) points, in meters, near the origin."""
    xs = rng.uniform(-POINT_SPREAD, POINT_SPREAD, size=NUM_POINTS)
    ys = rng.uniform(-POINT_SPREAD, POINT_SPREAD, size=NUM_POINTS)
    zs = rng.uniform(0.0, POINT_HEIGHT, size=NUM_POINTS)
    return np.stack((xs, ys, zs), axis=-1).tolist()


class FakeLidarPublisher(Node):
    """Publishes fake wheel/leg point clouds, static TF, and delayed nav-complete."""

    def __init__(self):
        super().__init__('publish_fake_lidar')

        # ground_pointcloud_collector's / ground_lidar_tf_transformer's
        # points_qos (design.md 7-1): best effort / volatile / keep last / depth 5.
        points_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # ground_elevation_map_saver's completion_topic subscription QoS
        # (design.md 7-6): reliable / transient_local / keep last / depth 1.
        nav_complete_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._rng = np.random.default_rng()
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self._points_pubs = {
            robot: self.create_publisher(PointCloud2, f'/{robot}/points', points_qos)
            for robot in ROBOTS
        }
        self._nav_complete_pubs = {
            robot: self.create_publisher(
                Bool, f'/{robot}/navigation_complete', nav_complete_qos)
            for robot in ROBOTS
        }

        self._publish_static_tf()
        self._points_timer = self.create_timer(1.0 / POINTS_HZ, self._publish_fake_clouds)
        self._nav_complete_timer = self.create_timer(
            NAV_COMPLETE_DELAY_SEC, self._publish_nav_complete_once)

    def _publish_static_tf(self):
        stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(_build_static_transforms(stamp))
        self.get_logger().info(
            'published static TF: map -> {wheel,leg}/odom -> .../base_link -> '
            '.../os1_lidar')

    def _publish_fake_clouds(self):
        stamp = self.get_clock().now().to_msg()
        for robot in ROBOTS:
            header = Header(stamp=stamp, frame_id=f'{robot}/os1_lidar')
            cloud = create_cloud_xyz32(header, _build_fake_points(self._rng))
            self._points_pubs[robot].publish(cloud)

    def _publish_nav_complete_once(self):
        self._nav_complete_timer.cancel()
        for robot in ROBOTS:
            self._nav_complete_pubs[robot].publish(Bool(data=True))
            self.get_logger().info(f'{robot}: published navigation_complete=True')
        self.get_logger().info(
            'both navigation_complete topics published. keeping node alive so '
            'points keep flowing -- Ctrl+C to stop.')


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
