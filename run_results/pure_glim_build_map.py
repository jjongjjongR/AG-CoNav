#!/usr/bin/env python3
"""순수 GLIM 표준 재현 실험, 3단계 -- GLIM 궤적(traj_lidar.txt, TUM 포맷)으로
원본 LiDAR 점을 map 좌표로 옮기고, 셀(0.10m)별 단순 평균(sum/count, 칼만필터
아님)으로 누적한다. drone_elevation_mapper.py는 건드리지 않고 완전히 별도로
만든다(지시사항).

grid_math.py와 동일한 규약: origin_x/origin_y는 격자의 min-corner 월드좌표,
row는 +x 방향, col은 +y 방향으로 자란다.

_grow_to_fit 격자 확장은 drone_elevation_mapper.py의 동명 메서드와 동일한
패딩 로직을 그대로 재구현한다(칼만 variance 배열이 없다는 점만 다름).
안전장치 2(지시사항): max_grid_cells(기본 30,000,000, Module D/E와 동일 값)를
넘는 배치는 저장하지 않고 버리되(에러 로그만 남기고 계속 진행) -- 이 상한에
실제로 걸리면 "GLIM 궤적이 튀어서 격자가 비정상적으로 커지려 했다"는 뜻이므로
그 사실을 통계로 남긴다(정확도 개선 로직이 아니라 순수 크래시 방지용).

**초기 1회 프레임 정합(필수, GPS 사후결합과는 다르다)**: GLIM의 map/odom
프레임은 "첫 스캔을 원점 부근으로 잡는" 자체 좌표계라(compare_gt_glim.py
docstring 참고), world(GT) 좌표계와 상수 SE3 오프셋이 있다. 이 오프셋을
전혀 안 맞추면 지도가 평가영역(BOX, world 절대좌표)과 아예 안 겹쳐 커버리지가
항상 0%가 된다 -- 이건 "GLIM 정확도"의 문제가 아니라 "로컬 좌표계를 세계
좌표계에 최초 1회 등록하는" 모든 SLAM 배포에 필요한 절차(실제 로봇도 이륙
지점의 GPS/기준점 하나로 이 등록을 한다)라서, 지시사항이 금지한 "GPS 사후
결합"(비행 내내 주기적으로 GPS로 재정합, glim_gps_correct.py 방식)과는
성격이 다르다고 판단했다. compare_gt_glim.py가 평가에 이미 쓰던 것과
정확히 같은 방법(Umeyama, 스케일 고정, 궤적 초반 align_window_s초만 사용,
그 뒤로는 절대 다시 맞추지 않음 -- GPS 사후결합처럼 비행 내내 반복 재정합하지
않는다)을 여기서도 독립적으로 재구현해 그대로 쓴다.

    python3 pure_glim_build_map.py <bag_dir> <traj_lidar.txt> <out.npz> \
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
MIN_RANGE = 2.5  # drone_elevation_mapper.py와 동일: 기체 자기반사 제거(센서 데이터 클리닝, GLIM/정확도 튜닝 아님)


def stamp_to_sec(s):
    return s.sec + s.nanosec * 1e-9


def load_gt_prefix(bag_dir, t_max_rel):
    """/tf map->drone/base_link 를 bag 시작~t_max_rel초까지만 읽는다(초반 정합용,
    22GB bag 전체를 안 훑기 위해 조기 종료)."""
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
    """src -> dst 로 맞추는 (R, t, s). compare_gt_glim.py와 동일 로직(재구현)."""
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


def align_traj_to_gt(bag_dir, t_imu, trans_imu, quat_imu, t_lidar, trans_lidar, quat_lidar,
                      align_window_s):
    """IMU 궤적 초반 align_window_s초를 GT에 맞춰 R,t(스케일 고정)를 구하고,
    같은 강체변환을 LiDAR 궤적 전체(회전+평행이동)에 적용해 반환한다."""
    gt_t, gt_x, gt_y, gt_z = load_gt_prefix(bag_dir, align_window_s + 5.0)
    mask = t_imu <= (t_imu[0] + align_window_s)
    q_t = t_imu[mask]
    gi_x = np.interp(q_t, gt_t, gt_x)
    gi_y = np.interp(q_t, gt_t, gt_y)
    gi_z = np.interp(q_t, gt_t, gt_z)
    src = trans_imu[mask]
    dst = np.stack([gi_x, gi_y, gi_z], axis=1)
    R, t, s = umeyama(src, dst, fix_scale=True)
    print(f'초기 프레임 정합: IMU 초반 {mask.sum()}개 점(t<={t_imu[0]+align_window_s:.1f}s), '
          f'R=\n{R}\nt={t}')

    new_trans_lidar = (R @ trans_lidar.T).T + t
    R_align_rot = Rotation.from_matrix(R)
    new_quat_lidar = (R_align_rot * Rotation.from_quat(quat_lidar)).as_quat()
    return new_trans_lidar, new_quat_lidar


def load_traj(path):
    data = np.loadtxt(path)
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


class GrowableMeanGrid:
    """sum_z/count 배열로 셀별 단순 평균을 누적. drone_elevation_mapper.py의
    _grow_to_fit와 동일한 패딩 로직(read-only로 그 파일을 참고해 독립 재구현,
    파일 자체는 손대지 않음)."""

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
        row_idx = np.floor((xs - self.origin_x) / self.res).astype(np.int64) if self.sum_z is not None \
            else np.floor((xs - 0.0) / self.res).astype(np.int64)
        col_idx = np.floor((ys - self.origin_y) / self.res).astype(np.int64) if self.sum_z is not None \
            else np.floor((ys - 0.0) / self.res).astype(np.int64)
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
    print(f'GLIM 궤적(LiDAR, 정합 전): {len(t_arr)}개 포즈, t=[{t_arr[0]:.2f},{t_arr[-1]:.2f}]')

    imu_traj_path = traj_path.replace('traj_lidar.txt', 'traj_imu.txt')
    t_imu, trans_imu, quat_imu = load_traj(imu_traj_path)
    trans_arr, quat_arr = align_traj_to_gt(
        bag_dir, t_imu, trans_imu, quat_imu, t_arr, trans_arr, quat_arr, align_window_s)

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
        print(f'*** 안전장치2 발동: {grid.n_dropped_batches}개 배치({grid.n_dropped_points}개 점) '
              f'격자 크기 상한 초과로 버려짐 -- GLIM 궤적이 튀었다는 증거로 기록 ***')
    else:
        print('안전장치2(격자 크기 상한) 발동 없음.')

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
