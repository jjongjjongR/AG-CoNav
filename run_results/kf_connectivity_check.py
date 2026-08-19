#!/usr/bin/env python3
"""최종 종합 -- 상위 후보들의 저장된 elevation_map(elevation_map_saver mcap)으로
wheel/leg 최대연결덩어리%를 계산한다(지시된 코드 그대로 사용).

    python3 kf_connectivity_check.py <elevation_map_mcap_dir> [tag]
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from grid_map_msgs.msg import GridMap
from scipy import ndimage

sys.path.insert(0, '/home/hyunwoo-chae/AG-CoNav-test_main/src/agconav_map_fusion/agconav_map_fusion')
sys.path.insert(0, '/home/hyunwoo-chae/AG-CoNav-test_main/run_results')
import grid_math  # noqa: E402
from gt_traversable import build as build_gt  # noqa: E402

RES = 0.10
WHEEL_STEP, LEG_STEP = 0.08, 0.15
WHEEL_RADIUS, LEG_RADIUS = 0.55, 0.40


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


def step4(elev):
    pad = np.pad(elev, 1, mode="constant", constant_values=np.nan)
    up = np.abs(pad[:-2, 1:-1] - elev)
    down = np.abs(pad[2:, 1:-1] - elev)
    left = np.abs(pad[1:-1, :-2] - elev)
    right = np.abs(pad[1:-1, 2:] - elev)
    with np.errstate(invalid="ignore"):
        step = np.nanmax(np.stack([up, down, left, right]), axis=0)
    all_nan = np.all(np.isnan(np.stack([up, down, left, right])), axis=0)
    step[all_nan] = np.nan
    return step


def nearest_lookup(src, src_origin_x, src_origin_y, res, gx, gy):
    n_rows, n_cols = src.shape
    ri = np.round((gx - src_origin_x) / res - 0.5).astype(np.int64)
    ci = np.round((gy - src_origin_y) / res - 0.5).astype(np.int64)
    inb = (ri >= 0) & (ri < n_rows) & (ci >= 0) & (ci < n_cols)
    out = np.full(gx.shape, np.nan, dtype=np.float64)
    out_flat = out.reshape(-1)
    ri_c, ci_c = np.clip(ri, 0, n_rows - 1), np.clip(ci, 0, n_cols - 1)
    vals = src[ri_c, ci_c]
    out_flat[inb.reshape(-1)] = vals.reshape(-1)[inb.reshape(-1)]
    return out


def build_g(elev_eval, step_eval, thresh):
    g = np.full(elev_eval.shape, -1, dtype=np.int16)
    known = np.isfinite(elev_eval) & np.isfinite(step_eval)
    g[known & (step_eval < thresh)] = 0
    g[known & (step_eval >= thresh)] = 1
    return g


def largest_component_pct(g, inbox, radius_m):
    RADIUS = radius_m
    r = RADIUS / RES
    k = int(np.ceil(r))
    yy, xx = np.ogrid[-k:k + 1, -k:k + 1]
    disk = (xx * xx + yy * yy) <= r * r
    free = (g == 0) & inbox
    eroded = ndimage.binary_erosion(free, structure=disk)
    lab, n = ndimage.label(eroded)
    sizes = np.bincount(lab.ravel())
    sizes[0] = 0
    return 100.0 * sizes.max() / inbox.sum() if inbox.sum() else float("nan")


def fn_rate(g_eval, gt_mask):
    verdict = g_eval[gt_mask]
    n_gt = int(gt_mask.sum())
    n_blocked = int((verdict == 1).sum())
    n_unknown = int((verdict == -1).sum())
    return {
        "FN_percent": 100.0 * n_blocked / n_gt if n_gt else float("nan"),
        "unknown_percent": 100.0 * n_unknown / n_gt if n_gt else float("nan"),
    }


def main():
    bag_dir = sys.argv[1]
    tag = sys.argv[2] if len(sys.argv) > 2 else bag_dir

    elev, origin_x, origin_y, res = load_elevation_map(bag_dir)
    assert abs(res - RES) < 1e-6, f'resolution mismatch: {res}'
    step = step4(elev)

    gt = build_gt(RES)
    xs, ys = gt["xs"], gt["ys"]
    gx, gy = np.meshgrid(xs, ys)

    elev_eval = nearest_lookup(elev, origin_x, origin_y, RES, gx, gy)
    step_eval = nearest_lookup(step, origin_x, origin_y, RES, gx, gy)
    inbox = np.ones(gx.shape, dtype=bool)

    print(f'[{tag}]')
    for robot, thresh, radius, gt_key in (
        ("wheel", WHEEL_STEP, WHEEL_RADIUS, "wheel_traversable"),
        ("leg", LEG_STEP, LEG_RADIUS, "leg_traversable"),
    ):
        g_eval = build_g(elev_eval, step_eval, thresh)
        fn = fn_rate(g_eval, gt[gt_key])
        largest_pct = largest_component_pct(g_eval, inbox, radius)
        print(f'  {robot}: 재계산 FN%={fn["FN_percent"]:.2f}% (자체 재계산, capture_nav_fn.py의 '
              f'navmap 기반과는 정의 경로가 달라 참고용), 최대연결덩어리%={largest_pct:.2f}%')


if __name__ == "__main__":
    main()
