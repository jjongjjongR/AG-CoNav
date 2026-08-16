#!/usr/bin/env python3
"""GLIM+GPS 6단계 -- glim_gps_build_map.py가 만든 elevation map에서
4개 지표(커버리지, wheel FN%, wheel/leg 최대연결덩어리%)를 계산한다.
비행 소요시간은 별도(4단계 bag duration 실측값)로 합쳐 보고서에 기록.

최대 연결 덩어리% 알고리즘은 사용자가 지정한 코드를 정확히 그대로 사용
(run_results/PROGRESS.md 6-4절 원문).

    python3 glim_gps_metrics.py <map.npz> <out.md>
"""
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, '/home/hyunwoo-chae/AG-CoNav-test_main/run_results')
from gt_traversable import build as build_gt  # noqa: E402

WHEEL_STEP, LEG_STEP = 0.08, 0.15
RES = 0.10


def step4(elev):
    pad = np.pad(elev, 1, mode='edge')
    up = np.abs(pad[:-2, 1:-1] - elev)
    down = np.abs(pad[2:, 1:-1] - elev)
    left = np.abs(pad[1:-1, :-2] - elev)
    right = np.abs(pad[1:-1, 2:] - elev)
    return np.maximum(np.maximum(up, down), np.maximum(left, right))


def largest_connected_pct(g, inbox, radius_m):
    """사용자 지정 알고리즘 그대로."""
    r = radius_m / RES
    k = int(np.ceil(r))
    yy, xx = np.ogrid[-k:k + 1, -k:k + 1]
    disk = (xx * xx + yy * yy) <= r * r
    free = (g == 0) & inbox
    eroded = ndimage.binary_erosion(free, structure=disk)
    lab, n = ndimage.label(eroded)
    sizes = np.bincount(lab.ravel())
    sizes[0] = 0
    return 100.0 * sizes.max() / inbox.sum()


def main():
    map_path, out_path = sys.argv[1], sys.argv[2]
    d = np.load(map_path)
    elevation = d['elevation']
    count = d['count']
    n_rows, n_cols = int(d['n_rows']), int(d['n_cols'])

    valid = count > 0
    coverage_pct = 100.0 * valid.sum() / (n_rows * n_cols)
    print(f'커버리지: {coverage_pct:.2f}% ({int(valid.sum())}/{n_rows * n_cols})')

    gt = build_gt(RES)
    # gt_traversable.build()의 격자(xs,ys 기반)와 이 지도의 격자(BOX 기반 row/col)가
    # 동일한 BOX/RES로 만들어졌는지 shape로 확인(surface_model.BOX를 공유).
    assert gt['wheel_traversable'].shape == (n_rows, n_cols), (
        f"GT 격자{gt['wheel_traversable'].shape} != 지도 격자{(n_rows, n_cols)}")

    step = step4(elevation)  # NaN 셀은 step도 NaN -> 아래서 g<0으로 분류

    results = {}
    for robot, robot_step in (('wheel', WHEEL_STEP), ('leg', LEG_STEP)):
        g = np.where(~valid, -1, np.where(step < robot_step, 0, 1)).astype(np.int32)
        inbox = np.ones((n_rows, n_cols), dtype=bool)  # 평가영역 전체(GT 무관)
        largest_pct = largest_connected_pct(g, inbox, 0.55 if robot == 'wheel' else 0.40)
        results[robot] = {'g': g, 'largest_pct': largest_pct}
        print(f'{robot} 최대연결덩어리%: {largest_pct:.2f}%')

    gt_wheel = gt['wheel_traversable']
    occupied_wheel = results['wheel']['g'] > 0
    n_gt = int(gt_wheel.sum())
    n_fn = int((gt_wheel & occupied_wheel).sum())
    wheel_fn_pct = 100.0 * n_fn / n_gt if n_gt else float('nan')
    print(f'wheel FN%: {wheel_fn_pct:.2f}% ({n_fn}/{n_gt})')

    with open(out_path, 'w') as f:
        f.write('# GLIM+GPS 지표 계산 결과 (원시 데이터)\n\n')
        f.write(f'- coverage_percent: {coverage_pct:.4f}\n')
        f.write(f'- wheel_FN_percent: {wheel_fn_pct:.4f} (n_gt={n_gt}, n_fn={n_fn})\n')
        f.write(f"- wheel_largest_connected_percent: {results['wheel']['largest_pct']:.4f}\n")
        f.write(f"- leg_largest_connected_percent: {results['leg']['largest_pct']:.4f}\n")
    print(f'저장: {out_path}')


if __name__ == '__main__':
    main()
