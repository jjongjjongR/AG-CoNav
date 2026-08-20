#!/usr/bin/env python3
"""FAST-LIO2 실험 3단계 -- FAST-LIO2 궤적(traj_lidar.txt, TUM 포맷, LiDAR
프레임 pose)으로 원본 LiDAR 점을 map 좌표로 옮기고, 셀(0.10m)별 단순평균
(sum/count, 칼만필터 아님)으로 누적한다. pure_glim_build_map.py와 동일한
설계(_grow_to_fit 성장 격자 + max_grid_cells 크래시 방지 상한, 초기 1회
좌표계 정합)를 FAST-LIO 궤적 파일 구조(traj_imu.txt 없이 traj_lidar.txt
하나뿐 -- FAST-LIO는 GLIM과 달리 odometry 토픽으로만 pose를 주고 별도
파일저장을 안 함, PROGRESS.md 참고)에 맞춰 독립적으로 재구현했다.
drone_elevation_mapper.py는 건드리지 않는다(지시사항).

**초기 1회 프레임 정합(GPS 사후결합과는 다르다, GLIM 실험과 동일한 근거)**:
FAST-LIO의 map/odom 프레임도 GLIM처럼 첫 pose를 원점 부근으로 잡는 자체
좌표계라 world(GT) 좌표계와 상수 SE3 오프셋이 있다. Umeyama(스케일 고정,
궤적 초반 align_window_s초만 사용)로 딱 1회만 정합하고 이후 절대
다시 맞추지 않는다 -- 로컬 SLAM 좌표계를 세계 좌표계에 등록하는 절차일
뿐 정확도 개선이 아니다. traj_lidar.txt 하나로 정합 피팅과 점 변환을
둘 다 한다(GT의 base_link와 LiDAR 사이 수 cm 오프셋은 초반 5초짜리 성긴
정합에는 무시 가능한 근사).

    python3 fastlio_build_map.py <bag_dir> <traj_lidar.txt> <out.npz> \
        [t_min] [t_max] [max_grid_cells] [align_window_s]
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

RES = 0.10
MIN_RANGE = 2.5  # 프로젝트 전체와 동일: 기체 자기반사 제거


def stamp_to_sec(s):
    return s.sec + s.nanosec * 1e-9


def load_traj(path):
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    order = np.argsort(data[:, 0])
    data = data[order]
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def interp_pose(t_query, t_arr, trans_arr, quat_arr):
    if t_query <= t_arr[0]:
        idx = 0
    elif t_query >= t_arr[-1]:
        idx = len(t_arr) - 2
    else:
        idx = np.searchsorted(t_arr, t_query, side='right') - 1
    idx = int(np.clip(idx, 0, len(t_arr) - 2))
    t0, t1 = t_arr[idx], t_arr[idx + 1]
    alpha = 0.0 if t1 <= t0 else float(np.clip((t_query - t0) / (t1 - t0), 0.0, 1.0))
    trans = trans_arr[idx] * (1 - alpha) + trans_arr[idx + 1] * alpha
    r0 = Rotation.from_quat(quat_arr[idx])
    r1 = Rotation.from_quat(quat_arr[idx + 1])
    slerp = Slerp([0.0, 1.0], Rotation.concatenate([r0, r1]))
    quat = slerp([alpha]).as_quat()[0]
    return trans, quat


def load_gt_prefix(bag_dir, t_max_rel):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    ts, xs, ys, zs = [], [], [], []
    t0 = None
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != '/tf':
            continue
        msg = deserialize_message(data, TFMessage)
        for tr in msg.transforms:
            if tr.child_frame_id != 'drone/base_link':
                continue
            t = stamp_to_sec(tr.header.stamp)
            if t0 is None:
                t0 = t
            ts.append(t)
            xs.append(tr.transform.translation.x)
            ys.append(tr.transform.translation.y)
            zs.append(tr.transform.translation.z)
        if t0 is not None and (ts[-1] - t0) > t_max_rel:
            break
    order = np.argsort(ts)
    return np.array(ts)[order], np.array(xs)[order], np.array(ys)[order], np.array(zs)[order]


def umeyama(src, dst, fix_scale=True):
    mu_src, mu_dst = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_src, dst - mu_dst
    cov = dst_c.T @ src_c / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1
    R = U @ S @ Vt
    s = 1.0 if fix_scale else np.trace(np.diag(D) @ S) / ((src_c ** 2).sum() / len(src))
    t = mu_dst - s * R @ mu_src
    return R, t, s


def align_traj_to_gt(bag_dir, t_arr, trans_arr, quat_arr, align_window_s):
    gt_t, gt_x, gt_y, gt_z = load_gt_prefix(bag_dir, align_window_s + 5.0)
    mask = t_arr <= (t_arr[0] + align_window_s)
    q_t = t_arr[mask]
    gi_x = np.interp(q_t, gt_t, gt_x)
    gi_y = np.interp(q_t, gt_t, gt_y)
    gi_z = np.interp(q_t, gt_t, gt_z)
    src = trans_arr[mask]
    dst = np.stack([gi_x, gi_y, gi_z], axis=1)
    R, t, s = umeyama(src, dst, fix_scale=True)
    print(f'초기 프레임 정합: 궤적 초반 {mask.sum()}개 점(t<={t_arr[0]+align_window_s:.1f}s), '
          f'R=\n{R}\nt={t}')

    new_trans = (R @ trans_arr.T).T + t
    R_align_rot = Rotation.from_matrix(R)
    new_quat = (R_align_rot * Rotation.from_quat(quat_arr)).as_quat()
    return new_trans, new_quat


class GrowableMeanGrid:
    """pure_glim_build_map.py와 동일한 _grow_to_fit 로직 독립 재구현."""

    def __init__(self, resolution, max_grid_cells):
        self.res = resolution
        self.max_grid_cells = max_grid_cells
        self.sum_z = None
        self.count = None
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.n_dropped_batches = 0
        self.n_dropped_points = 0
        self.max_requested_cells = 0

    def _grow_to_fit(self, row_idx, col_idx):
        if self.sum_z is None:
            min_row, max_row = int(row_idx.min()), int(row_idx.max())
            min_col, max_col = int(col_idx.min()), int(col_idx.max())
            n_rows = max_row - min_row + 1
            n_cols = max_col - min_col + 1
            total_cells = n_rows * n_cols
            self.max_requested_cells = max(self.max_requested_cells, total_cells)
            if total_cells > self.max_grid_cells:
                print(f'[grid-cap] grid would grow to {total_cells} cells (init), '
                      f'exceeding max_grid_cells={self.max_grid_cells}, dropping this batch',
                      file=sys.stderr)
                return None, None
            self.sum_z = np.zeros((n_rows, n_cols), dtype=np.float64)
            self.count = np.zeros((n_rows, n_cols), dtype=np.int64)
            self.origin_x += min_row * self.res
            self.origin_y += min_col * self.res
            return row_idx - min_row, col_idx - min_col

        n_rows, n_cols = self.sum_z.shape
        pad_before_row = max(0, -int(row_idx.min()))
        pad_after_row = max(0, int(row_idx.max()) - (n_rows - 1))
        pad_before_col = max(0, -int(col_idx.min()))
        pad_after_col = max(0, int(col_idx.max()) - (n_cols - 1))

        if pad_before_row or pad_after_row or pad_before_col or pad_after_col:
            new_n_rows = n_rows + pad_before_row + pad_after_row
            new_n_cols = n_cols + pad_before_col + pad_after_col
            total_cells = new_n_rows * new_n_cols
            self.max_requested_cells = max(self.max_requested_cells, total_cells)
            if total_cells > self.max_grid_cells:
                print(f'[grid-cap] grid would grow to {total_cells} cells '
                      f'({new_n_rows}x{new_n_cols}), exceeding '
                      f'max_grid_cells={self.max_grid_cells}, dropping this batch',
                      file=sys.stderr)
                return None, None
            pad_width = ((pad_before_row, pad_after_row), (pad_before_col, pad_after_col))
            self.sum_z = np.pad(self.sum_z, pad_width, constant_values=0.0)
            self.count = np.pad(self.count, pad_width, constant_values=0)
            self.origin_x -= pad_before_row * self.res
            self.origin_y -= pad_before_col * self.res
            row_idx = row_idx + pad_before_row
            col_idx = col_idx + pad_before_col

        return row_idx, col_idx

    def add(self, xs, ys, zs):
        ox = self.origin_x if self.sum_z is not None else 0.0
        oy = self.origin_y if self.sum_z is not None else 0.0
        row_idx = np.floor((xs - ox) / self.res).astype(np.int64)
        col_idx = np.floor((ys - oy) / self.res).astype(np.int64)
        row_idx, col_idx = self._grow_to_fit(row_idx, col_idx)
        if row_idx is None:
            self.n_dropped_batches += 1
            self.n_dropped_points += len(xs)
            return
        flat = row_idx * self.sum_z.shape[1] + col_idx
        np.add.at(self.sum_z.reshape(-1), flat, zs)
        np.add.at(self.count.reshape(-1), flat, 1)

    def elevation(self):
        return np.where(self.count > 0, self.sum_z / np.maximum(self.count, 1), np.nan)


def main():
    bag_dir, traj_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    t_min = float(sys.argv[4]) if len(sys.argv) > 4 else -np.inf
    t_max = float(sys.argv[5]) if len(sys.argv) > 5 else np.inf
    max_grid_cells = int(sys.argv[6]) if len(sys.argv) > 6 else 30_000_000
    align_window_s = float(sys.argv[7]) if len(sys.argv) > 7 else 5.0

    t_arr, trans_arr, quat_arr = load_traj(traj_path)
    print(f'FAST-LIO2 궤적(정합 전): {len(t_arr)}개 포즈, t=[{t_arr[0]:.2f},{t_arr[-1]:.2f}]')

    if align_window_s > 0:
        trans_arr, quat_arr = align_traj_to_gt(bag_dir, t_arr, trans_arr, quat_arr, align_window_s)
    else:
        print('align_window_s<=0 -- 초기 프레임 정합 건너뜀')

    grid = GrowableMeanGrid(RES, max_grid_cells)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/drone/points']))

    n_scans = n_pts_total = n_skipped_scans = 0
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp < t_min or stamp > t_max:
            continue
        if stamp < t_arr[0] or stamp > t_arr[-1]:
            n_skipped_scans += 1
            continue

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

        grid.add(world[:, 0], world[:, 1], world[:, 2])

        if n_scans % 2000 == 0:
            print(f'  스캔 {n_scans}개 처리, 점 {n_pts_total}개', file=sys.stderr)

    elevation = grid.elevation()
    print(f'스캔 {n_scans}개(궤적 범위 밖이라 건너뜀 {n_skipped_scans}개), 점 {n_pts_total}개')
    if grid.sum_z is not None:
        n_rows, n_cols = grid.sum_z.shape
        n_valid = int((grid.count > 0).sum())
        print(f'격자 {n_rows}x{n_cols}={n_rows*n_cols}, 유효 셀 {n_valid} '
              f'({100*n_valid/(n_rows*n_cols):.2f}%)')
    print(f'max_grid_cells 상한: {max_grid_cells} (요청된 최대 셀 수: {grid.max_requested_cells})')
    if grid.n_dropped_batches:
        print(f'*** 안전장치 발동: {grid.n_dropped_batches}개 배치({grid.n_dropped_points}개 점) '
              f'격자 크기 상한 초과로 버려짐 -- 궤적이 튀었다는 증거로 기록 ***')
    else:
        print('격자 크기 상한 발동 없음.')

    np.savez(out_path, elevation=elevation,
             count=(grid.count if grid.count is not None else np.zeros((0, 0))),
             origin_x=grid.origin_x, origin_y=grid.origin_y, res=RES,
             n_dropped_batches=grid.n_dropped_batches,
             n_dropped_points=grid.n_dropped_points,
             max_requested_cells=grid.max_requested_cells,
             max_grid_cells=max_grid_cells,
             n_scans=n_scans, n_skipped_scans=n_skipped_scans, n_pts_total=n_pts_total)
    print(f'저장: {out_path}')


if __name__ == '__main__':
    main()
