#!/usr/bin/env python3
"""/drone/points를 몇 장 구독해 x/y/z 범위와 finite 비율을 바로 출력하고 종료."""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy


class Probe(Node):
    def __init__(self, n_msgs):
        super().__init__('probe_points_once')
        self.n_msgs = n_msgs
        self.count = 0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(PointCloud2, '/drone/points', self.cb, qos)

    def cb(self, msg):
        self.count += 1
        pts = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        finite = np.isfinite(pts).all(axis=1)
        n_fin = int(finite.sum())
        n_tot = len(pts)
        print(f"--- msg {self.count}: total={n_tot} finite={n_fin} "
             f"({100*n_fin/max(n_tot,1):.2f}%)")
        if n_fin:
            fp = pts[finite]
            print(f"    finite x: {fp[:,0].min():.3f}..{fp[:,0].max():.3f}  "
                 f"y: {fp[:,1].min():.3f}..{fp[:,1].max():.3f}  "
                 f"z: {fp[:,2].min():.3f}..{fp[:,2].max():.3f}")
            dist = np.linalg.norm(fp, axis=1)
            print(f"    finite range(from sensor): {dist.min():.3f}..{dist.max():.3f}")
        if self.count >= self.n_msgs:
            rclpy.shutdown()


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    rclpy.init()
    node = Probe(n)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
