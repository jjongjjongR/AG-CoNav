#!/usr/bin/env python3
"""그림 3 방법별 연결성 — 모듈 F 가 실제로 낸 nav_map 격자에서 잰다.

논문 2.3 의 척도 그대로다: 통과 판정 셀을 로봇 반지름 원판으로 침식한 뒤
최대 연결 성분이 평가 영역에서 차지하는 비율. 상수·정의는
src/agconav_traversability/scripts/eval_connectivity.py 와 같다.

입력은 capture_nav_fn_dump.py 가 저장한 <태그>_fn_wheel_nav_map.npz 다.
**고도 지도에서 판정을 재계산하지 않는다** — 재계산은 모듈 F 출력과 3~4 %p
어긋나는데, 논문 표 7 이 보여주듯 연결성은 침식 후 자유 공간이 0.04 %p 만
달라도 3배 튀는 지표라 그 오차로 값이 통째로 바뀐다.

**평가 영역은 다섯 방법이 공통으로 관측한 상자다.** 운용 채점기로 저장 점군을
재생하면 지도가 y = -120 에서 잘린다(원인 미확인 — max_range_m 120.0 이
센서가 아니라 map 원점 기준으로 걸리는 것으로 보이나 검증하지 못했다).
다섯이 같은 채점기라 잘린 모양도 같으므로 상자 하나를 다섯에 똑같이 적용하면
비교는 성립한다. 다만 이 상자는 논문 표 9 의 상자와 달라서 **절대값을 논문과
직접 비교하면 안 되고 방법 간 순위·배율로만 읽어야 한다.**

OccupancyGrid 값 규약: 0 = 자유, 100 = 점유, -1 = 미관측.
"""
import json
import os
import sys

import numpy as np
from scipy import ndimage

FIG3 = os.path.expanduser('~/projects/AG-CoNav-gicp/run_results/fig3')
RADIUS = {'wheel': 0.55, 'leg': 0.40}       # eval_connectivity.py 와 동일
ROWS = [('D_baseline', '정답 위치 기준선 (5 m)', 45.80),
        ('D_ours84',   '제안 방식 (84 m)',       27.84),
        ('D_fastlio',  'FAST-LIO2 (5 m)',       25.88),
        ('D_gicp',     '정답 위치 + GICP (5 m)', 12.08),
        ('D_vgicp',    '정답 위치 + VGICP (5 m)', 6.80)]


def disk(r_cells):
    k = int(np.ceil(r_cells))
    y, x = np.ogrid[-k:k + 1, -k:k + 1]
    return (x * x + y * y) <= r_cells ** 2


def load_grid(tag, robot):
    d = np.load('%s/%s_fn_%s_nav_map.npz' % (FIG3, tag, robot))
    return d['data'], d['xs'], d['ys'], float(d['resolution'])


def to_gt(arr, xs, ys, gx, gy, fill=-1):
    out = np.full((len(gy), len(gx)), fill, arr.dtype)
    res = float(xs[1] - xs[0])
    j = np.round((gx - xs[0]) / res).astype(int)
    i = np.round((gy - ys[0]) / res).astype(int)
    jm = (j >= 0) & (j < arr.shape[1])
    im = (i >= 0) & (i < arr.shape[0])
    if jm.any() and im.any():
        out[np.ix_(im, jm)] = arr[np.ix_(i[im], j[jm])]
    return out


def main():
    robot = sys.argv[1] if len(sys.argv) > 1 else 'wheel'
    gt = np.load(os.path.join(FIG3, 'gt_truth.npz'))
    gx, gy = gt['xs'], gt['ys']
    have = [t for t, _, _ in ROWS
            if os.path.exists('%s/%s_fn_%s_nav_map.npz' % (FIG3, t, robot))]

    grids = {}
    known_all = np.ones((len(gy), len(gx)), bool)
    for t in have:
        a, xs, ys, res = load_grid(t, robot)
        g = to_gt(a, xs, ys, gx, gy)
        grids[t] = (g, res)
        known_all &= (g >= 0)
    ri, ci = np.where(known_all)
    if ri.size == 0:
        print('공통 관측 영역이 없다.'); return
    box = (float(gx[ci.min()]), float(gx[ci.max()]),
           float(gy[ri.min()]), float(gy[ri.max()]))
    keep = np.zeros(known_all.shape, bool)
    keep[np.ix_((gy >= box[2]) & (gy <= box[3]),
                (gx >= box[0]) & (gx <= box[1]))] = True
    total = int(keep.sum())
    print('공통 평가 상자: x %.1f ~ %.1f m, y %.1f ~ %.1f m  (%.0f x %.0f m, %d 셀)'
          % (box[0], box[1], box[2], box[3],
             box[1] - box[0], box[3] - box[2], total))
    print('척도: 통과 판정 셀을 반지름 %.2f m 원판으로 침식 -> 최대 연결덩어리 / 상자'
          % RADIUS[robot])
    print()
    hdr = ('%-26s %9s %9s %9s %8s %11s' %
           ('', '자유%', '점유%', '침식후%', '덩어리', '최대덩어리%'))
    print(hdr); print('-' * len(hdr))

    gfree = (gt[robot] if robot in gt else gt['wheel']) & keep
    ger = ndimage.binary_erosion(gfree, structure=disk(RADIUS[robot] / 0.10))
    gl, gn = ndimage.label(ger)
    gs = np.bincount(gl.ravel()); gs[0] = 0
    print('%-26s %8.2f%% %8.2f%% %8.2f%% %8d %10.2f%%'
          % ('정답 지형', 100 * gfree.sum() / total, 100 * (keep & ~gfree).sum() / total,
             100 * ger.sum() / total, gn, 100 * gs.max() / total))

    out = {'box': box, 'cells': total, 'robot': robot,
           'gt': {'free_pct': 100 * gfree.sum() / total,
                  'largest_pct': 100 * gs.max() / total, 'components': int(gn)}}
    for t, name, paper in ROWS:
        if t not in grids:
            print('%-26s %9s' % (name, '없음')); continue
        g, res = grids[t]
        free = (g == 0) & keep
        occ = (g == 100) & keep
        er = ndimage.binary_erosion(free, structure=disk(RADIUS[robot] / res))
        lab, n = ndimage.label(er)
        if n:
            sz = np.bincount(lab.ravel()); sz[0] = 0
            largest = 100.0 * sz.max() / total
        else:
            largest = 0.0
        print('%-26s %8.2f%% %8.2f%% %8.2f%% %8d %10.2f%%   (논문 %.2f%%)'
              % (name, 100 * free.sum() / total, 100 * occ.sum() / total,
                 100 * er.sum() / total, n, largest, paper))
        out[t] = dict(name=name, free_pct=100 * free.sum() / total,
                      occupied_pct=100 * occ.sum() / total,
                      eroded_pct=100 * er.sum() / total,
                      components=int(n), largest_pct=largest, paper=paper)
    with open(os.path.join(FIG3, 'connectivity_%s.json' % robot), 'w') as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print('\n저장: %s/connectivity_%s.json' % (FIG3, robot))


if __name__ == '__main__':
    main()
