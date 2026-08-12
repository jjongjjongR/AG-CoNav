#!/usr/bin/env python3
"""Accumulate the drone's scans into a map-frame cloud, registered by timestamp.

The difference from module A is only in how a scan gets its pose. Module A asks
tf2 for map -> drone/os1_lidar and takes what the buffer gives; here the drone
pose series is interpolated -- linearly in position, slerp in rotation -- to the
exact `header.stamp` of each cloud. At 8 m/s a 10 ms pose mismatch is 8 cm on
the ground, so which pose a scan is paired with is not a detail.

Output is an Nx4 .npy: x, y, z, scan index. The scan index is what lets
eval_map.py separate "different passes disagree" from "the reference is wrong".

    python3 build_map.py bags/drone_scan_100x100 /tmp/cloud.npy
    python3 build_map.py --pose-topic /model/X3/pose bags/... /tmp/cloud.npy
"""

from __future__ import annotations

import argparse
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

# drone/base_link -> drone/os1_lidar, from models/agconav_drone/model.sdf:
# the lidar link sits 0.175406 m below base and is pitched +90 deg.
T_BASE_LIDAR_Q = (0.0, 0.7071068, 0.0, 0.7071068)
T_BASE_LIDAR_P = np.array([0.0, 0.0, -0.175406])


def quat_to_R(q):
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def slerp(q0, q1, t):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    q0 /= np.linalg.norm(q0)
    q1 /= np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0.0:                      # take the short way round
        q1, d = -q1, -d
    if d > 0.9995:                   # nearly identical; lerp is exact enough
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th0 = np.arccos(d)
    th = th0 * t
    q2 = q1 - q0 * d
    q2 /= np.linalg.norm(q2)
    return q0 * np.cos(th) + q2 * np.sin(th)


def read_cloud(msg):
    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    p = a[:, :12].copy().view(np.float32).reshape(-1, 3).astype(np.float64)
    return p[np.isfinite(p).all(1)]


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def collect(bag, pose_topic, child):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    times, quats, trans, scans = [], [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == pose_topic:
            m = deserialize_message(data, TFMessage)
            for tr in m.transforms:
                if child and tr.child_frame_id != child:
                    continue
                q, p = tr.transform.rotation, tr.transform.translation
                times.append(stamp_of(tr))
                quats.append((q.x, q.y, q.z, q.w))
                trans.append((p.x, p.y, p.z))
        elif topic == '/drone/points':
            m = deserialize_message(data, PointCloud2)
            pts = read_cloud(m)
            if len(pts):
                scans.append((stamp_of(m), pts))
    return np.array(times), np.array(quats), np.array(trans), scans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--pose-topic', default='/tf')
    ap.add_argument('--child', default='drone/base_link',
                    help='transform to follow; blank to take every transform')
    args = ap.parse_args()

    times, quats, trans, scans = collect(args.bag, args.pose_topic, args.child)
    if not len(times):
        raise SystemExit('pose 를 찾지 못했습니다: topic=%s child=%s'
                         % (args.pose_topic, args.child))
    order = np.argsort(times)
    times, quats, trans = times[order], quats[order], trans[order]
    print('pose %d개 (%.2f ~ %.2f s), 스캔 %d장'
          % (len(times), times[0], times[-1], len(scans)))

    Rbl = quat_to_R(T_BASE_LIDAR_Q)
    out, ids, skipped, lags = [], [], 0, []
    for k, (ts, pts) in enumerate(scans):
        i = np.searchsorted(times, ts)
        if i == 0 or i >= len(times):
            skipped += 1
            continue
        t0, t1 = times[i - 1], times[i]
        u = 0.0 if t1 <= t0 else (ts - t0) / (t1 - t0)
        lags.append(min(ts - t0, t1 - ts))
        R = quat_to_R(slerp(quats[i - 1], quats[i], u))
        p = trans[i - 1] * (1 - u) + trans[i] * u
        world = (R @ ((Rbl @ pts.T).T + T_BASE_LIDAR_P).T).T + p
        out.append(world)
        ids.append(np.full(len(world), k))

    cloud = np.hstack([np.vstack(out), np.concatenate(ids)[:, None]])
    np.save(args.out, cloud)
    lags = np.array(lags)
    print('보간 사용: 스캔 %d장 (범위 밖 %d장 제외)' % (len(out), skipped))
    print('  스캔 시각과 가장 가까운 pose 표본 사이 간격: 중앙값 %.4f s, 최대 %.4f s'
          % (np.median(lags), lags.max()))
    print('  → 보간이 없었다면 8 m/s 기준 최대 %.3f m 오차' % (8 * lags.max()))
    print('저장 %s  (%d점)' % (args.out, len(cloud)))


if __name__ == '__main__':
    main()
