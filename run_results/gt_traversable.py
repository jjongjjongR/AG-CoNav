#!/usr/bin/env python3
"""5단계 FN% 채점을 위한 ground-truth "실제로 통과 가능한 셀" 마스크 생성.

정의(사용자 지시, run_results/PROGRESS.md 5단계 참조):
    실제로 통과 가능한 셀 = 건물 풋프린트 밖 AND GT 지형(순수 terrain, 건물 제외)
    기준 인접 셀 4방향 중 최대 높이차가 로봇 기준(wheel 0.08m / leg 0.15m) 미만.

지형 높이는 eval_map.py와 동일하게 height_map.png(순수 지형)만 쓴다 -- 이건
surface_model.py의 "지형 vs 건물 중 높은 쪽"과는 다른, 순수 지형 참값이다
(건물 위는 애초에 로봇이 못 올라가므로 풋프린트로 아예 제외하지, 그 위 지형
자체의 단차를 평가하지 않는다).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "run_results"))
from surface_model import load_buildings, BOX, _point_in_poly  # noqa: E402

WHEEL_STEP, LEG_STEP = 0.08, 0.15
HM_PNG = str(_REPO_ROOT / "src/agconav_test_worlds/worlds/Seongdong_gu_100x100/mesh/height_map.png")
HM_SIZE_Z, HM_POS_Z = 5.249597, 1.202132


def terrain_grid(res=0.10):
    """height_map.png를 res 해상도 격자로 리샘플한 순수 지형 고도."""
    img = np.array(Image.open(HM_PNG)).astype(np.float64)
    ny, nx = img.shape
    xa, xb, ya, yb = BOX
    xs = np.arange(xa, xb, res) + res / 2.0
    ys = np.arange(ya, yb, res) + res / 2.0
    gx, gy = np.meshgrid(xs, ys)  # (row=y, col=x)

    cols = (gx - xa) / (xb - xa) * (nx - 1)
    rows = (yb - gy) / (yb - ya) * (ny - 1)
    c0 = np.clip(np.floor(cols).astype(int), 0, nx - 2)
    r0 = np.clip(np.floor(rows).astype(int), 0, ny - 2)
    fc, fr = cols - c0, rows - r0
    px = (img[r0, c0] * (1 - fc) * (1 - fr) + img[r0, c0 + 1] * fc * (1 - fr)
          + img[r0 + 1, c0] * (1 - fc) * fr + img[r0 + 1, c0 + 1] * fc * fr)
    elev = px / 65535.0 * HM_SIZE_Z + HM_POS_Z
    return xs, ys, elev


def building_mask(xs, ys):
    gx, gy = np.meshgrid(xs, ys)
    mask = np.zeros(gx.shape, dtype=bool)
    buildings = load_buildings()
    for b in buildings:
        x0, x1, y0, y1 = b["bbox"]
        box_mask = (gx >= x0 - 0.5) & (gx <= x1 + 0.5) & (gy >= y0 - 0.5) & (gy <= y1 + 0.5)
        if not box_mask.any():
            continue
        idxs = np.where(box_mask)
        inside = _point_in_poly(gx[idxs], gy[idxs], b["poly"])
        mask[idxs[0][inside], idxs[1][inside]] = True
    return mask


def step4(elev):
    """4-neighbour 최대 높이차 (테두리는 있는 이웃만)."""
    step = np.full_like(elev, np.nan)
    pad = np.pad(elev, 1, mode="edge")
    up = np.abs(pad[:-2, 1:-1] - elev)
    down = np.abs(pad[2:, 1:-1] - elev)
    left = np.abs(pad[1:-1, :-2] - elev)
    right = np.abs(pad[1:-1, 2:] - elev)
    step = np.maximum(np.maximum(up, down), np.maximum(left, right))
    return step


def build(res=0.10):
    xs, ys, elev = terrain_grid(res)
    bmask = building_mask(xs, ys)
    step = step4(elev)
    wheel_traversable = (~bmask) & (step < WHEEL_STEP)
    leg_traversable = (~bmask) & (step < LEG_STEP)
    return {
        "xs": xs, "ys": ys, "elev": elev, "building_mask": bmask, "step": step,
        "wheel_traversable": wheel_traversable, "leg_traversable": leg_traversable,
    }


if __name__ == "__main__":
    gt = build(0.10)
    n = gt["wheel_traversable"].size
    print(f"격자 {gt['wheel_traversable'].shape}, 셀 {n}")
    print(f"건물 풋프린트 셀: {gt['building_mask'].sum()} ({100*gt['building_mask'].sum()/n:.1f}%)")
    print(f"wheel 통과가능(GT): {gt['wheel_traversable'].sum()} ({100*gt['wheel_traversable'].sum()/n:.1f}%)")
    print(f"leg 통과가능(GT): {gt['leg_traversable'].sum()} ({100*gt['leg_traversable'].sum()/n:.1f}%)")
