#!/usr/bin/env python3
"""GLIM+GPS 6단계 -- 보정된 궤적(glim_gps_correct.py 출력)으로 원본 LiDAR
스캔을 map 좌표로 옮겨 셀별 단순 평균(sum/count, 칼만필터 아님) 누적한다.
drone_elevation_mapper.py는 건드리지 않고 완전히 별도로 만든다(지시사항).

메모리 안전 설계는 run_results/build_methodB_cloud.py의 패턴을 따른다:
전체 점을 리스트에 들고 있지 않고, 격자 자체(sum/count 배열, map 크기
고정 예상되므로 미리 큰 배열을 할당)에 스캔마다 바로 누적한다 -- 점
배열을 저장할 필요가 없어 build_methodB_cloud.py보다도 메모리 부담이
적다.

    python3 glim_gps_build_map.py <gps_bag_dir> <corrected_traj.npz> <out.npz>
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2

RES = 0.10
# BOX: run_results/gt_traversable.py와 동일한 평가영역(surface_model.py의 BOX
# 상수, 100x100 월드 지형 전체) -- 격자를 그 영역에 맞춰 고정 크기로 미리
# 할당해 동적 확장 로직 없이 안전하게 처리한다.
sys.path.insert(0, '/home/hyunwoo-chae/AG-CoNav-test_main/run_results')
from surface_model import BOX  # noqa: E402


def load_corrected(path):
    d = np.load(path)
    return d['t'], d['trans'], d['quat']


def interp_pose(t_query, t_arr, trans_arr, quat_arr):
    t_query = np.clip(t_query, t_arr[0], t_arr[-1])
    idx = np.searchsorted(t_arr, t_query, side='right') - 1
    idx = np.clip(idx, 0, len(t_arr) - 2)
    t0, t1 = t_arr[idx], t_arr[idx + 1]
    alpha = 0.0 if t1 <= t0 else (t_query - t0) / (t1 - t0)
    trans = trans_arr[idx] * (1 - alpha) + trans_arr[idx + 1] * alpha
    r0 = Rotation.from_quat(quat_arr[idx])
    r1 = Rotation.from_quat(quat_arr[idx + 1])
    key = [0.0, 1.0] if t1 > t0 else [0.0, 1.0 + 1e-9]
    from scipy.spatial.transform import Slerp
    slerp = Slerp(key, Rotation.concatenate([r0, r1]))
    quat = slerp([alpha]).as_quat()[0]
    return trans, quat


def main():
    bag_dir, traj_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]

    t_arr, trans_arr, quat_arr = load_corrected(traj_path)
    print(f'보정된 궤적: {len(t_arr)}개, t=[{t_arr[0]:.2f},{t_arr[-1]:.2f}]')

    xa, xb, ya, yb = BOX
    n_rows = int(np.ceil((xb - xa) / RES))
    n_cols = int(np.ceil((yb - ya) / RES))
    sum_z = np.zeros((n_rows, n_cols), dtype=np.float64)
    count = np.zeros((n_rows, n_cols), dtype=np.int64)
    print(f'격자 {n_rows}x{n_cols} 할당 완료')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/drone/points']))

    n_scans = n_pts_total = n_oob = 0
    MIN_RANGE = 2.5  # drone_elevation_mapper.py와 동일: 기체 자기반사 제거
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        from sensor_msgs.msg import PointCloud2
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] == 0:
            continue
        ranges = np.linalg.norm(pts, axis=1)
        pts = pts[ranges >= MIN_RANGE]
        if pts.shape[0] == 0:
            continue

        trans, quat = interp_pose(stamp, t_arr, trans_arr, quat_arr)
        R = Rotation.from_quat(quat).as_matrix()
        world = (R @ pts.T).T + trans
        n_scans += 1
        n_pts_total += world.shape[0]

        row = np.floor((world[:, 0] - xa) / RES).astype(np.int64)
        col = np.floor((world[:, 1] - ya) / RES).astype(np.int64)
        inb = (row >= 0) & (row < n_rows) & (col >= 0) & (col < n_cols)
        n_oob += int((~inb).sum())
        row, col, z = row[inb], col[inb], world[inb, 2]

        flat = row * n_cols + col
        np.add.at(sum_z.reshape(-1), flat, z)
        np.add.at(count.reshape(-1), flat, 1)

        if n_scans % 1000 == 0:
            print(f'  스캔 {n_scans}개 처리, 점 {n_pts_total}개', file=sys.stderr)

    elevation = np.where(count > 0, sum_z / np.maximum(count, 1), np.nan)
    print(f'스캔 {n_scans}개, 점 {n_pts_total}개(영역 밖 {n_oob}개 제외)')
    print(f'유효 셀 {int((count > 0).sum())}/{n_rows * n_cols} '
          f'({100 * (count > 0).sum() / (n_rows * n_cols):.2f}%)')

    np.savez(out_path, elevation=elevation, count=count,
             xa=xa, ya=ya, res=RES, n_rows=n_rows, n_cols=n_cols)
    print(f'저장: {out_path}')


if __name__ == '__main__':
    main()
