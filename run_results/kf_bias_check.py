#!/usr/bin/env python3
"""실험 F -- baseline 결과가 GT(순수 지형) 대비 일관된 방향의 편향(bias)이
있는지 통계적으로 확인한다.

    python3 kf_bias_check.py <elevation_map_mcap_dir>
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from grid_map_msgs.msg import GridMap

sys.path.insert(0, '/home/hyunwoo-chae/AG-CoNav-test_main/src/agconav_map_fusion/agconav_map_fusion')
sys.path.insert(0, '/home/hyunwoo-chae/AG-CoNav-test_main/run_results')
import grid_math  # noqa: E402
from gt_traversable import terrain_grid, building_mask  # noqa: E402


def load_elevation_map(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    msg = None
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        msg = deserialize_message(data, GridMap)
    if msg is None:
        raise RuntimeError(f'no GridMap message found in {bag_dir}')
    elevation, origin_x, origin_y = grid_math.extract_elevation(msg)
    return elevation, origin_x, origin_y, msg.info.resolution


def main():
    bag_dir = sys.argv[1]
    elev, origin_x, origin_y, res = load_elevation_map(bag_dir)
    n_rows, n_cols = elev.shape
    print(f'지도: {n_rows}x{n_cols} @ {res}m, origin=({origin_x:.2f},{origin_y:.2f})')

    xs, ys, gt_elev = terrain_grid(res)
    bmask = building_mask(xs, ys)  # shape (len(ys), len(xs))

    # gt_traversable 격자(xs=x축, ys=y축, meshgrid shape=(row=y,col=x))를
    # 우리 지도(row=+x,col=+y) 인덱스로 매핑.
    gx, gy = np.meshgrid(xs, ys)
    ri = np.round((gx - origin_x) / res - 0.5).astype(np.int64)
    ci = np.round((gy - origin_y) / res - 0.5).astype(np.int64)
    inb = (ri >= 0) & (ri < n_rows) & (ci >= 0) & (ci < n_cols)
    ri_c, ci_c = np.clip(ri, 0, n_rows - 1), np.clip(ci, 0, n_cols - 1)
    mapped = np.where(inb, elev[ri_c, ci_c], np.nan)

    valid = np.isfinite(mapped) & np.isfinite(gt_elev) & (~bmask)
    diff = mapped[valid] - gt_elev[valid]
    print(f'비교 가능한 셀(건물 제외, 둘 다 유효): {valid.sum()}개')
    print(f'diff(지도-GT지형) mean={diff.mean():.5f}m median={np.median(diff):.5f}m '
          f'std={diff.std():.5f}m')
    print(f'  p5={np.percentile(diff,5):.5f} p25={np.percentile(diff,25):.5f} '
          f'p75={np.percentile(diff,75):.5f} p95={np.percentile(diff,95):.5f}')
    # one-sample t-test 근사(표본이 매우 크므로 정규근사로 충분): mean/sem
    sem = diff.std(ddof=1) / np.sqrt(diff.size)
    t_stat = diff.mean() / sem if sem > 0 else float('nan')
    print(f'표준오차={sem:.6f}m, t통계량={t_stat:.2f} (|t|>>3이면 통계적으로 '
          f'유의한 편향)')


if __name__ == '__main__':
    main()
