#!/usr/bin/env python3
"""GLIM(+GPS 사후결합) 5단계 -- GLIM 원시 궤적(traj_lidar.txt, TUM format)에
GPS 앵커로 드리프트 보정을 적용해, 보정된 궤적을 별도 파일로 저장한다.

방식(run_results/PROGRESS.md "3-3. GPS 사후 결합 알고리즘 확정" 참고):
  P_raw(t)       : GLIM 원시 pose (traj_lidar.txt에서 선형/slerp 보간)
  C_k            : GPS 앵커 시각 t_k에서의 보정 transform
                   translation = GPS_ENU(t_k) - P_raw(t_k).translation
                   rotation = identity (GPS는 orientation 정보가 없음 --
                   안전한 기본값)
  C(t)           : 앵커 사이 SE3 보간(translation lerp, rotation slerp)
  P_corrected(t) = C(t) @ P_raw(t)   (world-frame 보정이므로 왼쪽 곱)

    python3 glim_gps_correct.py <gps_bag_dir> <traj_lidar.txt> <out_corrected.npz>
"""
import sys

import numpy as np
import pymap3d as pm
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import NavSatFix

# Seongdong_gu_100x100_dynamic.world의 <spherical_coordinates> 기준점
# (run_results/PROGRESS.md "3-2. GPS 좌표 변환 확인" 참고).
LAT0, LON0, ALT0 = 37.54233814881853, 127.06050643805561, 15.400000000001455


def load_traj(path):
    """TUM format(t x y z qx qy qz qw) -> (t[N], trans[N,3], quat[N,4] xyzw)."""
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    t = data[:, 0]
    trans = data[:, 1:4]
    quat = data[:, 4:8]
    order = np.argsort(t)
    return t[order], trans[order], quat[order]


def load_gps(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/drone/gps']))
    ts, enu = [], []
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        msg = deserialize_message(data, NavSatFix)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        e, n, u = pm.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, LAT0, LON0, ALT0)
        ts.append(stamp)
        enu.append((e, n, u))
    ts = np.array(ts)
    enu = np.array(enu)
    order = np.argsort(ts)
    return ts[order], enu[order]


def interp_pose(t_query, t_arr, trans_arr, quat_arr):
    """t_arr/trans_arr/quat_arr(정렬됨)에서 t_query 시각의 pose를 선형/slerp
    보간. t_query가 범위 밖이면 가장 가까운 끝값으로 클램프."""
    t_query = np.clip(t_query, t_arr[0], t_arr[-1])
    idx = np.searchsorted(t_arr, t_query, side='right') - 1
    idx = np.clip(idx, 0, len(t_arr) - 2)
    t0, t1 = t_arr[idx], t_arr[idx + 1]
    with np.errstate(divide='ignore', invalid='ignore'):
        alpha = np.where(t1 > t0, (t_query - t0) / np.maximum(t1 - t0, 1e-9), 0.0)
    trans = trans_arr[idx] * (1 - alpha)[:, None] + trans_arr[idx + 1] * alpha[:, None]
    # quaternion slerp (per-sample, scipy Slerp는 단일 궤적만 지원하므로 루프)
    quat = np.empty((len(t_query), 4))
    for i in range(len(t_query)):
        r0 = Rotation.from_quat(quat_arr[idx[i]])
        r1 = Rotation.from_quat(quat_arr[idx[i] + 1])
        key_times = [0.0, 1.0] if t1[i] > t0[i] else [0.0, 1.0 + 1e-9]
        slerp = Slerp(key_times, Rotation.concatenate([r0, r1]))
        quat[i] = slerp([alpha[i]]).as_quat()[0]
    return trans, quat


def main():
    bag_dir, traj_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]

    t_raw, trans_raw, quat_raw = load_traj(traj_path)
    print(f'GLIM 원시 궤적: {len(t_raw)}개 pose, t=[{t_raw[0]:.2f},{t_raw[-1]:.2f}]')

    t_gps, enu_gps = load_gps(bag_dir)
    print(f'GPS: {len(t_gps)}개 샘플, t=[{t_gps[0]:.2f},{t_gps[-1]:.2f}]')

    # GPS 앵커 시각들에서의 GLIM 원시 위치를 구해 보정 오프셋(translation만) 계산.
    trans_at_gps, _ = interp_pose(t_gps, t_raw, trans_raw, quat_raw)
    offset_at_gps = enu_gps - trans_at_gps  # C_k.translation, rotation=identity

    # 각 GLIM 원시 pose 시각 t_raw[i]에서, 그 시각을 감싸는 두 GPS 앵커 사이의
    # 보정 오프셋을 lerp(rotation 보정은 항상 identity라 slerp 결과도 identity
    # -- PROGRESS.md 3-3절 근거)해서 적용.
    t_gps_c = np.clip(t_raw, t_gps[0], t_gps[-1])
    idx = np.searchsorted(t_gps, t_gps_c, side='right') - 1
    idx = np.clip(idx, 0, len(t_gps) - 2)
    tg0, tg1 = t_gps[idx], t_gps[idx + 1]
    with np.errstate(divide='ignore', invalid='ignore'):
        alpha = np.where(tg1 > tg0, (t_gps_c - tg0) / np.maximum(tg1 - tg0, 1e-9), 0.0)
    offset = offset_at_gps[idx] * (1 - alpha)[:, None] + offset_at_gps[idx + 1] * alpha[:, None]

    trans_corrected = trans_raw + offset  # rotation은 원시 그대로(보정 rotation=identity)

    residual = np.linalg.norm(trans_at_gps - enu_gps, axis=1)
    print(f'보정 전 GPS-GLIM 잔차: mean={residual.mean():.3f}m max={residual.max():.3f}m')

    np.savez(out_path, t=t_raw, trans=trans_corrected, quat=quat_raw)
    print(f'저장: {out_path} ({len(t_raw)}개 보정된 pose)')


if __name__ == '__main__':
    main()
