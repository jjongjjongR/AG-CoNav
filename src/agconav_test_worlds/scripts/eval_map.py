#!/usr/bin/env python3
"""Score a drone-built elevation map against the test world's true terrain.

Every mapping method under test is scored with this one script so the numbers
are comparable. The reference is the world's own heightmap, read with the same
pixel/world mapping generate_test_world.py wrote it with, so "error" here means
error against the terrain Gazebo actually simulates.

Inputs it accepts:
  --cloud  a .npy of Nx3 map-frame points (methods that accumulate a cloud)
  --grid   a rosbag2 directory carrying grid_map_msgs/GridMap (module A output)

Metrics, and why each one is here:

  coverage        두 가지를 따로 센다. "측정 커버리지"는 점이 하나라도 들어온 칸의
                  비율 — 0.10 m 단위로 바닥이 실제로 얼마나 찍혔는지다. "지면 셀"은
                  그중 높이가 지형과 0.8 m 이내인 칸으로, 건물 위 칸이 빠진다.
                  초기 판에서는 후자만 "커버리지"로 표시해 실제 98.9%인 값이 59.3%로
                  보였다. 두 숫자는 목적이 다르므로 둘 다 낸다.
  height error    bias / sigma / p95 against the true terrain -- the headline
                  accuracy number
  reproducibility spread between independent passes over the same cell. This
                  separates "the map is wrong" from "my reference is wrong":
                  only a cloud with per-point scan ids can report it.
  step            max height difference to the 4-neighbours, which is what
                  module F thresholds. Reported against both robots' limits
                  (wheel 0.08 m, leg 0.15 m) since that is the decision the
                  whole pipeline exists to make.
"""

from __future__ import annotations

import argparse
import numpy as np
from PIL import Image

# Seongdong_gu_100x100, as written by generate_test_world.py.
BOX = (-38.6, 61.4, -166.1, -66.1)
SIZE_Z, POS_Z = 5.2496, 1.2021
WHEEL_STEP, LEG_STEP = 0.08, 0.15


def load_reference(path):
    img = np.array(Image.open(path)).astype(np.float64)
    n = img.shape[0]
    xa, _, _, yb = BOX[0], BOX[1], BOX[2], BOX[3]
    span = BOX[1] - BOX[0]

    def terrain(x, y):
        c = np.clip((x - xa) / span * (n - 1), 0, n - 2)
        r = np.clip((yb - y) / span * (n - 1), 0, n - 2)
        c0, r0 = np.floor(c).astype(int), np.floor(r).astype(int)
        fc, fr = c - c0, r - r0
        v = (img[r0, c0] * (1 - fc) * (1 - fr) + img[r0, c0 + 1] * fc * (1 - fr)
             + img[r0 + 1, c0] * (1 - fc) * fr + img[r0 + 1, c0 + 1] * fc * fr)
        return v / 65535.0 * SIZE_Z + POS_Z

    return terrain


def cells(pts, res):
    """Mean height per cell, plus the cell centre and the point count."""
    gx = np.floor((pts[:, 0] - BOX[0]) / res).astype(np.int64)
    gy = np.floor((pts[:, 1] - BOX[2]) / res).astype(np.int64)
    key = gx * 1_000_000 + gy
    order = np.argsort(key, kind='stable')
    ks = key[order]
    uniq, start, cnt = np.unique(ks, return_index=True, return_counts=True)
    z = np.add.reduceat(pts[order, 2], start) / cnt
    cx = BOX[0] + (uniq // 1_000_000 + 0.5) * res
    cy = BOX[2] + (uniq % 1_000_000 + 0.5) * res
    return uniq, cx, cy, z, cnt, order, start


def reproducibility(pts, scan_id, res):
    """Height spread between passes that are far apart in time."""
    _, _, _, _, cnt, order, start = cells(pts, res)
    zs, ss = pts[order, 2], scan_id[order]
    diffs = []
    for i in range(len(start)):
        c = cnt[i]
        if c < 4:
            continue
        z, s = zs[start[i]:start[i] + c], ss[start[i]:start[i] + c]
        if s.max() - s.min() < 20:          # same pass, tells us nothing
            continue
        lo = s < s.mean()
        if lo.sum() < 2 or (~lo).sum() < 2:
            continue
        diffs.append(z[lo].mean() - z[~lo].mean())
    return np.array(diffs)


def step_stats(uniq, z, res):
    look = {int(k): float(v) for k, v in zip(uniq, z)}
    steps = []
    for k, v in look.items():
        gx, gy = k // 1_000_000, k % 1_000_000
        best = 0.0
        for dk in (1_000_000, -1_000_000, 1, -1):
            w = look.get(int(k + dk))
            if w is not None:
                best = max(best, abs(v - w))
        if best > 0:
            steps.append(best)
    return np.array(steps)


def report(tag, pts, terrain, res, scan_id=None, gridded=False):
    inside = ((pts[:, 0] >= BOX[0]) & (pts[:, 0] <= BOX[1])
              & (pts[:, 1] >= BOX[2]) & (pts[:, 1] <= BOX[3]))
    pts = pts[inside]
    if scan_id is not None:
        scan_id = scan_id[inside]

    # Grid first, then decide which cells are terrain. A GridMap arrives already
    # averaged, so filtering its points beforehand is impossible; doing the same
    # to a raw cloud would flatter it by discarding building returns before they
    # can pollute a shared cell. Both are judged on the cell values they produce.
    #
    # `gridded` inputs are already cell values on their own grid. Re-binning them
    # here would charge them for an origin that happens not to line up with this
    # script's, which on sloped ground is a real height penalty for nothing.
    if gridded:
        uniq = (np.floor((pts[:, 0] - BOX[0]) / res).astype(np.int64) * 1_000_000
                + np.floor((pts[:, 1] - BOX[2]) / res).astype(np.int64))
        cx, cy, z = pts[:, 0], pts[:, 1], pts[:, 2]
        cnt = np.ones(len(pts))
    else:
        uniq, cx, cy, z, cnt, _, _ = cells(pts, res)
    n_measured = len(uniq)          # 점이 하나라도 들어온 칸
    is_ground = np.abs(z - terrain(cx, cy)) < 0.8
    uniq, cx, cy, z, cnt = (uniq[is_ground], cx[is_ground], cy[is_ground],
                            z[is_ground], cnt[is_ground])
    if not len(uniq):
        # A diverged estimate can land its whole map outside the box. That is a
        # result, not a crash: report it and score nothing.
        print('[%s] 박스 안에 지면으로 볼 수 있는 셀이 하나도 없음 — 지도 사용 불가' % tag)
        return dict(cov=0.0, bias=float('nan'), sigma=float('nan'),
                    p95=float('nan'), step=float('nan'),
                    wheel=float('nan'), leg=float('nan'))

    err = z - terrain(cx, cy)
    on_ground = np.abs(pts[:, 2] - terrain(pts[:, 0], pts[:, 1])) < 0.8
    g = pts[on_ground]
    area = (BOX[1] - BOX[0]) * (BOX[3] - BOX[2])
    steps = step_stats(uniq, z, res)

    print('[%s] 셀 %.2f m' % (tag, res))
    print('  점 %d (지면 %d)' % (len(pts), len(g)))
    print('  측정 커버리지 %.1f%% (점이 들어온 칸 %d / %d)   칸당 %.1f점'
          % (100 * n_measured * res * res / area, n_measured,
             int(round(area / (res * res))), cnt.mean()))
    print('  지면 셀 %.1f%% (%d) — 건물 위 칸 제외'
          % (100 * len(uniq) * res * res / area, len(uniq)))
    print('  높이 오차   편향 %+.4f m   표준편차 %.4f m   95%% %.4f m   최대 %.4f m'
          % (err.mean(), err.std(), np.percentile(np.abs(err), 95), np.abs(err).max()))
    if scan_id is not None:
        d = reproducibility(g, scan_id[on_ground], res)
        if len(d):
            print('  패스 간 재현성  표준편차 %.4f m   95%% %.4f m   (%d 셀)'
                  % (d.std(), np.percentile(np.abs(d), 95), len(d)))
    if len(steps):
        print('  단차        중앙값 %.4f m   95%% %.4f m   |  wheel(%.2f) 초과 %.1f%%   leg(%.2f) 초과 %.1f%%'
              % (np.median(steps), np.percentile(steps, 95), WHEEL_STEP,
                 100 * (steps > WHEEL_STEP).mean(), LEG_STEP,
                 100 * (steps > LEG_STEP).mean()))
    return dict(cov=100 * n_measured * res * res / area,
                ground=100 * len(uniq) * res * res / area, bias=err.mean(),
                sigma=err.std(), p95=np.percentile(np.abs(err), 95),
                step=np.median(steps) if len(steps) else float('nan'),
                wheel=100 * (steps > WHEEL_STEP).mean() if len(steps) else float('nan'),
                leg=100 * (steps > LEG_STEP).mean() if len(steps) else float('nan'))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cloud', required=True, help='Nx3 (or Nx4 with scan id) .npy')
    p.add_argument('--tag', default='?')
    p.add_argument('--res', type=float, default=0.10)
    p.add_argument('--gridded', action='store_true',
                   help='입력이 이미 격자화된 셀 값이면 지정 (재격자화하지 않음)')
    p.add_argument('--reference',
                   default='src/agconav_test_worlds/worlds/Seongdong_gu_100x100/mesh/height_map.png')
    args = p.parse_args()

    arr = np.load(args.cloud)
    scan_id = arr[:, 3].astype(np.int64) if arr.shape[1] > 3 else None
    report(args.tag, arr[:, :3], load_reference(args.reference), args.res, scan_id,
           gridded=args.gridded)


if __name__ == '__main__':
    main()
