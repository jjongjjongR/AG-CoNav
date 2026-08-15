#!/usr/bin/env python3
"""drone_elevation_mapper._accumulate()와 동일한 파이프라인(isfinite 필터 ->
min_range 자기반사 필터 -> TF 변환)을 그대로 재현해, map 프레임으로 변환된 점들이
1) 센서 사거리(170m) 밖인지, 2) 사거리 안인데 방향상 100x100 박스 경계를 넘는지
구분한다. N개 스캔을 모아 분석 후 종료."""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points
from rclpy.time import Time

BOX = (-38.60, 61.40, -166.10, -66.10)
SENSOR_MAX_RANGE = 170.0
MIN_RANGE = 2.5  # drone_elevation_mapper.py 기본값(코드 확인, yaml에 미명시)


class Probe(Node):
    def __init__(self, n_msgs, min_range):
        super().__init__('probe_transformed')
        self.n_msgs = n_msgs
        self.min_range = min_range
        self.count = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(PointCloud2, '/drone/points', self.cb, qos)
        self.stats = {"n_scans": 0, "n_raw": 0, "n_finite": 0, "n_after_minrange": 0,
                      "n_over_sensor_range": 0, "n_out_of_box_but_in_range": 0,
                      "max_range_seen": 0.0, "worst_out_of_box": None}

    def cb(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException as ex:
            print(f"TF fail: {ex}")
            return

        raw = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        n_raw = raw.shape[0]
        finite = raw[np.isfinite(raw).all(axis=1)]
        n_finite = finite.shape[0]
        if n_finite == 0:
            return
        ranges = np.linalg.norm(finite, axis=1)
        after_minrange = finite[ranges >= self.min_range]
        ranges_after = ranges[ranges >= self.min_range]
        if after_minrange.shape[0] == 0:
            return

        mapped = transform_points(after_minrange, transform.transform)
        xa, xb, ya, yb = BOX
        out = (mapped[:, 0] < xa) | (mapped[:, 0] > xb) | (mapped[:, 1] < ya) | (mapped[:, 1] > yb)
        n_over_range = int((ranges_after > SENSOR_MAX_RANGE).sum())
        n_out = int(out.sum())

        self.count += 1
        self.stats["n_scans"] += 1
        self.stats["n_raw"] += n_raw
        self.stats["n_finite"] += n_finite
        self.stats["n_after_minrange"] += after_minrange.shape[0]
        self.stats["n_over_sensor_range"] += n_over_range
        self.stats["n_out_of_box_but_in_range"] += int((out & (ranges_after <= SENSOR_MAX_RANGE)).sum())
        self.stats["max_range_seen"] = max(self.stats["max_range_seen"], float(ranges_after.max()))

        if out.any():
            idx = np.argmax(np.where(out, ranges_after, -1))
            wx, wy, wz = mapped[idx]
            wr = ranges_after[idx]
            cand = (float(wx), float(wy), float(wz), float(wr))
            if (self.stats["worst_out_of_box"] is None
                    or wr > self.stats["worst_out_of_box"][3]):
                self.stats["worst_out_of_box"] = cand

        print(f"scan {self.count}: raw={n_raw} finite={n_finite} "
             f"after_minrange={after_minrange.shape[0]} "
             f"out_of_box={n_out} over_sensor_range={n_over_range} "
             f"max_range_this_scan={ranges_after.max():.2f}")

        if self.count >= self.n_msgs:
            print("\n=== SUMMARY ===")
            for k, v in self.stats.items():
                print(f"  {k}: {v}")
            rclpy.shutdown()


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    min_range = float(sys.argv[2]) if len(sys.argv) > 2 else MIN_RANGE
    rclpy.init()
    node = Probe(n, min_range)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
