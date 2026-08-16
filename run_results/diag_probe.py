#!/usr/bin/env python3
"""P0 5m/0deg 조건 진단용 1회성 스크립트 -- 원시 로컬프레임 점과 map변환 점을 함께 덤프."""
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points
from rclpy.time import Time

P0 = np.array([-25.0, -100.0, 1.4177])


class Diag(Node):
    def __init__(self):
        super().__init__('diag_probe')
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/cmd_pose', qos_pub)
        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(PointCloud2, '/drone/points', self._cb, qos_sub)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.msg = None

    def _cb(self, msg):
        self.msg = msg

    def teleport(self, pos, quat):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        for _ in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat
            self.pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        t_end = time.time() + 2.0
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = Diag()
    t_end = time.time() + 5.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)

    pos = (P0[0], P0[1], P0[2] + 5.0)
    quat = (0.0, 0.0, 0.0, 1.0)
    print(f'teleport to {pos}')
    node.teleport(pos, quat)

    # actual TF-reported base_link/lidar pose vs commanded
    try:
        t_base = node.tf_buffer.lookup_transform('map', 'drone/base_link', Time())
        print('actual base_link in map:', t_base.transform.translation)
    except TransformException as ex:
        print('base_link TF fail:', ex)
    try:
        t_lidar = node.tf_buffer.lookup_transform('map', 'drone/os1_lidar', Time())
        print('actual os1_lidar in map:', t_lidar.transform.translation, t_lidar.transform.rotation)
    except TransformException as ex:
        print('lidar TF fail:', ex)

    node.msg = None
    t_end = time.time() + 3.0
    while time.time() < t_end and node.msg is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    msg = node.msg
    if msg is None:
        print('NO POINTCLOUD RECEIVED')
        return
    print('frame_id:', msg.header.frame_id)
    raw = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    finite = raw[np.isfinite(raw).all(axis=1)]
    print('n_points total(finite):', finite.shape[0])
    r = np.linalg.norm(finite, axis=1)
    print('local range: min={:.3f} max={:.3f} mean={:.3f}'.format(r.min(), r.max(), r.mean()))
    print('local z: min={:.3f} max={:.3f} mean={:.3f}'.format(finite[:, 2].min(), finite[:, 2].max(), finite[:, 2].mean()))
    # histogram of local range
    hist, edges = np.histogram(r, bins=10)
    for c, e0, e1 in zip(hist, edges[:-1], edges[1:]):
        print(f'  range [{e0:.2f},{e1:.2f}): {c}')

    transform = node.tf_buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
    mapped = transform_points(finite, transform.transform)
    d_xy = np.linalg.norm(mapped[:, :2] - P0[:2], axis=1)
    print('map z near sensor nadir (d_xy<2m): n=', (d_xy < 2.0).sum())
    near = mapped[d_xy < 2.0]
    if near.shape[0]:
        print('  z min/max/mean:', near[:, 2].min(), near[:, 2].max(), near[:, 2].mean())
    within05 = mapped[d_xy <= 0.5]
    print('map z within 0.5m of P0: n=', within05.shape[0])
    if within05.shape[0]:
        print('  z min/max/mean:', within05[:, 2].min(), within05[:, 2].max(), within05[:, 2].mean())
        dxy2 = d_xy[d_xy <= 0.5]
        print('  d_xy min/max/mean:', dxy2.min(), dxy2.max(), dxy2.mean())

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
