#!/usr/bin/env python3
"""순수 GLIM 표준 재현 실험, 4단계 -- 4가지 지표(비행 소요시간은 별도로 bag에서
계산) 중 커버리지/wheel FN%/wheel·leg 최대연결덩어리%를 계산한다.

지시사항 그대로: 주행성 지도(g)는 이번 지도(pure_glim_build_map.py 산출물)의
4-이웃 최대 높이차(step)를 wheel 0.08m/leg 0.15m 기준으로 직접 구성한다
(Module F의 live traversability_verdictor 노드를 띄우지 않는다 -- 4-4가
"3번 지도 기준으로 g를 직접 구성"하라고 명시).

    python3 pure_glim_metrics.py <map.npz> <out.json>
"""
import json
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, "/home/hyunwoo-chae/AG-CoNav-test_main/run_results")
from gt_traversable import build as build_gt  # noqa: E402
from surface_model import BOX  # noqa: E402

RES = 0.10
WHEEL_STEP, LEG_STEP = 0.08, 0.15
WHEEL_RADIUS, LEG_RADIUS = 0.55, 0.40


def step4(elev):
    """4-이웃 최대 높이차. gt_traversable.py의 step4()와 동일 로직이지만
    이웃 중 하나라도 NaN이면 그 방향의 diff도 NaN으로 남긴다(경계 padding으로
    edge를 복제하는 gt_traversable.py와 달리, 우리 지도는 미관측 영역이 커서
    NaN 전파가 "미지"를 정확히 반영하는 데 더 적합)."""
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
    """gx/gy(월드 좌표, 임의 shape) 각 점에 대해 src 배열(row=+x,col=+y,
    src_origin 기준)에서 가장 가까운 셀 값을 반환. 범위 밖은 NaN."""
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
    """g: 0=free, 1=blocked, -1=unknown (nav_map/OccupancyGrid 관례와 동일)."""
    g = np.full(elev_eval.shape, -1, dtype=np.int16)
    known = np.isfinite(elev_eval) & np.isfinite(step_eval)
    g[known & (step_eval < thresh)] = 0
    g[known & (step_eval >= thresh)] = 1
    return g


def largest_component_pct(g, inbox, radius_m):
    r = radius_m / RES
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
    n_free = int((verdict == 0).sum())
    return {
        "n_gt_traversable": n_gt,
        "n_pipeline_blocked_FN": n_blocked,
        "n_pipeline_unknown": n_unknown,
        "n_pipeline_free_correct": n_free,
        "FN_percent": 100.0 * n_blocked / n_gt if n_gt else float("nan"),
        "unknown_percent": 100.0 * n_unknown / n_gt if n_gt else float("nan"),
    }


def main():
    map_path, out_path = sys.argv[1], sys.argv[2]
    d = np.load(map_path)
    elev = d["elevation"]
    origin_x, origin_y = float(d["origin_x"]), float(d["origin_y"])
    n_dropped_batches = int(d["n_dropped_batches"])
    n_dropped_points = int(d["n_dropped_points"])
    max_requested_cells = int(d["max_requested_cells"])
    max_grid_cells = int(d["max_grid_cells"])

    step = step4(elev)

    gt = build_gt(RES)
    xs, ys = gt["xs"], gt["ys"]
    gx, gy = np.meshgrid(xs, ys)  # shape (len(ys), len(xs)) = (row=y, col=x) per gt_traversable convention

    elev_eval = nearest_lookup(elev, origin_x, origin_y, RES, gx, gy)
    step_eval = nearest_lookup(step, origin_x, origin_y, RES, gx, gy)

    xa, xb, ya, yb = BOX
    n_eval_cells = gx.size
    n_valid_eval = int(np.isfinite(elev_eval).sum())
    coverage_percent = 100.0 * n_valid_eval / n_eval_cells

    inbox = np.ones(gx.shape, dtype=bool)  # 지시사항: inbox = 평가 영역 전체 셀

    result = {
        "map_path": map_path,
        "grid_cap": {
            "max_grid_cells": max_grid_cells,
            "max_requested_cells": max_requested_cells,
            "n_dropped_batches": n_dropped_batches,
            "n_dropped_points": n_dropped_points,
            "cap_triggered": n_dropped_batches > 0,
        },
        "coverage": {
            "n_eval_cells": int(n_eval_cells),
            "n_valid_cells": n_valid_eval,
            "coverage_percent": coverage_percent,
        },
    }

    for robot, thresh, radius, gt_key in (
        ("wheel", WHEEL_STEP, WHEEL_RADIUS, "wheel_traversable"),
        ("leg", LEG_STEP, LEG_RADIUS, "leg_traversable"),
    ):
        g_eval = build_g(elev_eval, step_eval, thresh)
        fn = fn_rate(g_eval, gt[gt_key])
        largest_pct = largest_component_pct(g_eval, inbox, radius)
        result[f"{robot}_FN"] = fn
        result[f"{robot}_largest_component_percent"] = largest_pct
        print(f'{robot}: FN%={fn["FN_percent"]:.2f}% (GT통과가능 {fn["n_gt_traversable"]}개 중 '
              f'막힘판정 {fn["n_pipeline_blocked_FN"]}개, 미측정 {fn["unknown_percent"]:.1f}%), '
              f'최대연결덩어리%={largest_pct:.2f}%')

    print(f'커버리지: {coverage_percent:.2f}% ({n_valid_eval}/{n_eval_cells})')
    if n_dropped_batches:
        print(f'*** 안전장치2 발동: {n_dropped_batches}개 배치 격자상한 초과로 버려짐 ***')

    with open(out_path, "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f'저장: {out_path}')


if __name__ == "__main__":
    main()
