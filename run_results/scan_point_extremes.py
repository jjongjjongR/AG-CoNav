#!/usr/bin/env python3
"""bag의 /drone/points 전체를 스트리밍으로 훑어 x,y,z 전역 min/max와,
100x100 박스 밖으로 벗어난 점의 비율/최대 이탈거리를 구한다.
메모리에 다 올리지 않고 메시지 단위로 처리한다."""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py.point_cloud2 import read_points_numpy

BOX = (-38.60, 61.40, -166.10, -66.10)  # xa,xb,ya,yb


def main():
    bag_dir = sys.argv[1]
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topic = "/drone/points"
    msg_type = get_message(type_map[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    gxmin = gymin = gzmin = 1e18
    gxmax = gymax = gzmax = -1e18
    n_msgs = 0
    n_pts_total = 0
    n_out_of_box = 0
    max_dist_out = 0.0
    worst_pt = None

    xa, xb, ya, yb = BOX
    while reader.has_next():
        (t, data, stamp) = reader.read_next()
        msg = deserialize_message(data, msg_type)
        n_msgs += 1
        try:
            pts = read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        except Exception:
            continue
        if pts.size == 0:
            continue
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            continue
        n_pts_total += len(pts)
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        gxmin = min(gxmin, float(x.min())); gxmax = max(gxmax, float(x.max()))
        gymin = min(gymin, float(y.min())); gymax = max(gymax, float(y.max()))
        gzmin = min(gzmin, float(z.min())); gzmax = max(gzmax, float(z.max()))

        out = (x < xa - 1.0) | (x > xb + 1.0) | (y < ya - 1.0) | (y > yb + 1.0)
        n_out = int(out.sum())
        n_out_of_box += n_out
        if n_out:
            dist = np.maximum.reduce([
                np.maximum(xa - x, 0), np.maximum(x - xb, 0),
                np.maximum(ya - y, 0), np.maximum(y - yb, 0)])
            idx = int(np.argmax(dist))
            if dist[idx] > max_dist_out:
                max_dist_out = float(dist[idx])
                worst_pt = (float(x[idx]), float(y[idx]), float(z[idx]))

        if n_msgs % 500 == 0:
            print(f"  ...{n_msgs} msgs, {n_pts_total} pts so far, out_of_box={n_out_of_box}",
                  file=sys.stderr)

    print(f"messages: {n_msgs}, total points: {n_pts_total}")
    print(f"x range: {gxmin:.3f} .. {gxmax:.3f}  (box x[{xa},{xb}])")
    print(f"y range: {gymin:.3f} .. {gymax:.3f}  (box y[{ya},{yb}])")
    print(f"z range: {gzmin:.3f} .. {gzmax:.3f}")
    print(f"points outside box(+1m margin): {n_out_of_box} ({100*n_out_of_box/max(n_pts_total,1):.4f}%)")
    print(f"worst outlier: {worst_pt}, max distance outside box: {max_dist_out:.3f} m")


if __name__ == "__main__":
    main()
