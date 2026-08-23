#!/usr/bin/env python3
"""그림: wheel / leg 주행성 지도와 각 지도가 산출한 경로.

입력은 2026-08-22 종단 실행이 남긴 maps/{wheel,leg}_nav_map.pgm 뿐이다.
경로는 그때 Nav2 가 실제로 달린 궤적이 아니라(그 실행은 궤적을 기록하지
않았다) **같은 시작·목표 좌표로 각 주행맵 위에서 다시 계획한 경로**다.
계획 규칙은 논문 평가 척도와 같다 — 자유 셀을 로봇 반지름 원판으로 침식한
뒤(wheel 0.55 m, leg 0.40 m) 8연결 최단 경로.
"""
import heapq, os, sys
import numpy as np
from scipy import ndimage, sparse
from scipy.sparse import csgraph
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.family'] = 'Noto Sans CJK JP'
matplotlib.rcParams['axes.unicode_minus'] = False
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(WS, 'results/final_maps')
RADIUS = {'wheel': 0.55, 'leg': 0.40}          # eval_connectivity.py 와 동일
# 2026-08-22 실행 로그의 bt_navigator "Begin navigating" 줄에서 그대로 가져왔다.
START = {'wheel': (-194.93, 72.89), 'leg': (-195.72, 76.39)}
GOAL  = {'wheel': (-67.40, -60.90), 'leg': (-65.00, -62.40)}
DS = 2                                          # 계획용 축소 (0.10 -> 0.20 m/cell)


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        assert maxv == 255
        return np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)


def read_yaml(path):
    d = {}
    for line in open(path):
        if ':' not in line:
            continue
        k, v = line.split(':', 1)
        d[k.strip()] = v.strip()
    org = [float(x) for x in d['origin'].strip('[]').split(',')]
    return float(d['resolution']), org[0], org[1]


def load(robot):
    """(free, occ, res, x0, y0) — 행 0 이 y 최소가 되도록 뒤집어 돌려준다."""
    img = read_pgm(os.path.join(WS, 'maps/%s_nav_map.pgm' % robot))
    res, x0, y0 = read_yaml(os.path.join(WS, 'maps/%s_nav_map.yaml' % robot))
    img = img[::-1]                             # map_server 는 위쪽이 y 최대
    # map_server trinary 규약: 254 자유 / 0 점유 / 205 미지
    return (img > 205), (img < 90), res, x0, y0


def disk(r_cells):
    k = int(np.ceil(r_cells))
    y, x = np.ogrid[-k:k + 1, -k:k + 1]
    return (x * x + y * y) <= r_cells ** 2


def plan(free, res, x0, y0, start, goal, radius):
    """침식된 자유 공간 위 8연결 최단 경로. 반환은 월드 좌표 (N,2)."""
    er = ndimage.binary_erosion(free, structure=disk(radius / res))
    # 계획용 축소 — 블록 안이 전부 통행 가능일 때만 통행 가능(보수적)
    h, w = er.shape
    hh, ww = h // DS * DS, w // DS * DS
    small = er[:hh, :ww].reshape(hh // DS, DS, ww // DS, DS).all(axis=(1, 3))
    sres = res * DS
    sh, sw = small.shape

    def rc(pt):
        c = int(round((pt[0] - x0) / sres))
        r = int(round((pt[1] - y0) / sres))
        return min(max(r, 0), sh - 1), min(max(c, 0), sw - 1)

    def nearest_free(r, c):
        if small[r, c]:
            return r, c
        d, idx = ndimage.distance_transform_edt(~small, return_indices=True)
        return int(idx[0, r, c]), int(idx[1, r, c])

    sr, sc = nearest_free(*rc(start))
    gr, gc = nearest_free(*rc(goal))

    idx = -np.ones(small.shape, np.int64)
    idx[small] = np.arange(int(small.sum()))
    n = int(small.sum())
    rows, cols, wts = [], [], []
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = small[max(0, -dr):sh - max(0, dr), max(0, -dc):sw - max(0, dc)]
        b = small[max(0, dr):sh - max(0, -dr), max(0, dc):sw - max(0, -dc)]
        both = a & b
        ia = idx[max(0, -dr):sh - max(0, dr), max(0, -dc):sw - max(0, dc)][both]
        ib = idx[max(0, dr):sh - max(0, -dr), max(0, dc):sw - max(0, -dc)][both]
        cost = sres * np.hypot(dr, dc)
        rows.append(ia); cols.append(ib); wts.append(np.full(ia.size, cost))
    g = sparse.coo_matrix((np.concatenate(wts),
                           (np.concatenate(rows), np.concatenate(cols))),
                          shape=(n, n)).tocsr()
    g = g + g.T
    dist, pred = csgraph.dijkstra(g, indices=idx[sr, sc], return_predecessors=True)
    t = idx[gr, gc]
    if t < 0 or not np.isfinite(dist[t]):
        return None, np.nan, er
    seq = []
    while t >= 0:
        seq.append(t); t = pred[t]
    seq = seq[::-1]
    rr, cc = np.nonzero(small)
    pts = np.column_stack([x0 + cc[seq] * sres, y0 + rr[seq] * sres])
    length = float(np.hypot(*np.diff(pts, axis=0).T).sum())
    return pts, length, er


def block_max(mask, f):
    h, w = mask.shape
    hh, ww = h // f * f, w // f * f
    return mask[:hh, :ww].reshape(hh // f, f, ww // f, f).any(axis=(1, 3))


def main():
    os.makedirs(OUT, exist_ok=True)
    data = {}
    for r in ('wheel', 'leg'):
        free, occ, res, x0, y0 = load(r)
        pts, length, er = plan(free, res, x0, y0, START[r], GOAL[r], RADIUS[r])
        data[r] = dict(free=free, occ=occ, res=res, x0=x0, y0=y0,
                       path=pts, length=length, er=er)
        print('%-5s free %.2f%%  occ %.2f%%  path %.1f m'
              % (r, 100 * free.mean(), 100 * occ.mean(), length))

    res = data['wheel']['res']; x0 = data['wheel']['x0']; y0 = data['wheel']['y0']
    F = 4                                        # 표시용 축소 (0.40 m/cell)
    h, w = data['wheel']['free'].shape
    extent = [x0, x0 + w * res, y0, y0 + h * res]

    fig, axgrid = plt.subplots(2, 2, figsize=(16.5, 14.2))
    axes = axgrid.ravel()
    cmap = ListedColormap(['#c8c8c8', '#2f8f4e', '#c0392b'])   # 미지/통과/막힘

    for ax, r in zip(axes[:2], ('wheel', 'leg')):
        d = data[r]
        img = np.zeros(block_max(d['free'], F).shape, np.uint8)
        img[block_max(d['free'], F)] = 1
        img[block_max(d['occ'], F)] = 2          # 얇은 벽이 사라지지 않도록 막힘 우선
        ax.imshow(img, origin='lower', extent=extent, cmap=cmap,
                  vmin=0, vmax=2, interpolation='nearest')
        if d['path'] is not None:
            ax.plot(d['path'][:, 0], d['path'][:, 1], '-', color='#1f4fd8', lw=2.6)
        ax.plot(*START[r], 'o', ms=11, mfc='#ffd400', mec='k', mew=1.4, zorder=5)
        ax.plot(*GOAL[r], '*', ms=20, mfc='#ff5ecb', mec='k', mew=1.2, zorder=5)
        pct = 100 * d['free'].mean()
        ax.set_title('(%s) %s 주행성 지도 — 통과 %.1f%%, 경로 %.1f m'
                     % ('a' if r == 'wheel' else 'b',
                        '4륜' if r == 'wheel' else '4족', pct, d['length']),
                     fontsize=13)

    # (c) 두 지도의 차이 — leg 는 통과인데 wheel 은 막힌 셀
    only_leg = data['leg']['free'] & data['wheel']['occ']
    only_wheel = data['wheel']['free'] & data['leg']['occ']
    ax = axes[2]
    base = block_max(data['wheel']['free'] | data['leg']['free'], F).astype(np.uint8)
    ax.imshow(base, origin='lower', extent=extent,
              cmap=ListedColormap(['#e9e9e9', '#f7f7f7']), vmin=0, vmax=1,
              interpolation='nearest')
    ol = block_max(only_leg, F); ow = block_max(only_wheel, F)
    ov = np.zeros(ol.shape + (4,))
    ov[ol] = (0.13, 0.47, 0.80, 1.0)
    ov[ow] = (0.85, 0.42, 0.05, 1.0)
    ax.imshow(ov, origin='lower', extent=extent, interpolation='nearest')
    for r, c in (('wheel', '#b8860b'), ('leg', '#0b6e9e')):
        if data[r]['path'] is not None:
            ax.plot(data[r]['path'][:, 0], data[r]['path'][:, 1], '-',
                    color=c, lw=2.4, label='%s 경로 %.1f m'
                    % ('4륜' if r == 'wheel' else '4족', data[r]['length']))
    ax.set_title('(c) 두 주행성 지도의 차이와 각자의 경로', fontsize=13)
    ax.legend(handles=[
        Patch(facecolor='#2178cc', label='4족만 통과 %d 셀' % int(only_leg.sum())),
        Patch(facecolor='#d96b0d', label='4륜만 통과 %d 셀' % int(only_wheel.sum())),
        Line2D([], [], color='#b8860b', lw=2.4,
               label='4륜 경로 %.1f m' % data['wheel']['length']),
        Line2D([], [], color='#0b6e9e', lw=2.4,
               label='4족 경로 %.1f m' % data['leg']['length'])],
        loc='lower right', fontsize=10, framealpha=0.92)

    axes[0].legend(handles=[Patch(facecolor='#2f8f4e', label='통과 가능'),
                            Patch(facecolor='#c0392b', label='막힘'),
                            Patch(facecolor='#c8c8c8', label='미관측'),
                            Line2D([], [], color='#1f4fd8', lw=2.6, label='계획 경로'),
                            Line2D([], [], marker='o', ls='', mfc='#ffd400',
                                   mec='k', ms=10, label='출발'),
                            Line2D([], [], marker='*', ls='', mfc='#ff5ecb',
                                   mec='k', ms=15, label='목표')],
                   loc='lower right', fontsize=10, framealpha=0.92)
    for ax in axes:
        ax.set_xlabel('World X [m]'); ax.set_ylabel('World Y [m]')
        ax.grid(alpha=0.25, lw=0.5)
    # (d) 두 경로가 갈라지는 구간 확대 — 이 그림의 요점이다.
    # 전체 맵에서는 7.5 m 차이가 눈에 안 띄므로 갈라지는 곳만 잘라 보여준다.
    pw, pl = data['wheel']['path'], data['leg']['path']
    d2 = np.hypot(pw[:, None, 0] - pl[None, :, 0],
                  pw[:, None, 1] - pl[None, :, 1]).min(axis=1)
    far = pw[d2 > 5.0]
    if far.size:
        mx, my = 22.0, 22.0
        box = [far[:, 0].min() - mx, far[:, 0].max() + mx,
               far[:, 1].min() - my, far[:, 1].max() + my]
    else:
        box = [-210, -50, -80, 90]
    ax = axes[3]
    wf = data['wheel']['free']; lf = data['leg']['free']
    c0 = int((box[0] - x0) / res); c1 = int((box[1] - x0) / res)
    r0 = int((box[2] - y0) / res); r1 = int((box[3] - y0) / res)
    # 계획기가 실제로 본 공간은 원본 자유 셀이 아니라 **로봇 반지름으로 침식한**
    # 공간이다(wheel 0.55 m / leg 0.40 m). 원본으로 그리면 둘 다 갈 수 있는
    # 것처럼 보여 경로가 갈라진 이유가 사라진다.
    we = data['wheel']['er']; le = data['leg']['er']
    sub = np.zeros((r1 - r0, c1 - c0), np.uint8)
    sub[we[r0:r1, c0:c1] & le[r0:r1, c0:c1]] = 1        # 둘 다 통행 가능
    sub[le[r0:r1, c0:c1] & ~we[r0:r1, c0:c1]] = 2       # 4족만 통행 가능
    sub[~le[r0:r1, c0:c1] & ~we[r0:r1, c0:c1]] = 3      # 둘 다 불가
    ax.imshow(sub, origin='lower', extent=box, interpolation='nearest',
              cmap=ListedColormap(['#dcdcdc', '#8fd6a4', '#2178cc', '#c0392b']),
              vmin=0, vmax=3)
    ax.plot(pw[:, 0], pw[:, 1], '-', color='#7a4b00', lw=3.0)
    ax.plot(pl[:, 0], pl[:, 1], color='#00304f', lw=3.0, ls='--')
    ax.set_xlim(box[0], box[1]); ax.set_ylim(box[2], box[3])
    ax.set_title('(d) 경로가 갈라지는 구간 확대 — 로봇 반지름 침식 후 통행 공간',
                 fontsize=13)
    ax.legend(handles=[
        Patch(facecolor='#8fd6a4', label='둘 다 통행 가능'),
        Patch(facecolor='#2178cc', label='4족만 통행 가능 (4륜 반지름에 막힘)'),
        Patch(facecolor='#c0392b', label='둘 다 불가'),
        Line2D([], [], color='#7a4b00', lw=3.0, label='4륜 경로'),
        Line2D([], [], color='#00304f', lw=3.0, ls='--', label='4족 경로')],
        loc='lower right', fontsize=10, framealpha=0.92)
    axes[2].add_patch(plt.Rectangle((box[0], box[2]), box[1] - box[0],
                                    box[3] - box[2], fill=False,
                                    ec='k', lw=1.8, ls=':'))

    fig.suptitle('단일 공중 스캔에서 갈라낸 플랫폼별 주행성 지도와 각자의 경로\n'
                 '5786 × 4855 셀 · 0.10 m/cell · 2026-08-22 종단 실행',
                 fontsize=15, y=0.985)
    fig.tight_layout(rect=[0, 0, 1, 0.955])
    p = os.path.join(OUT, 'navmap_wheel_vs_leg.png')
    fig.savefig(p, dpi=150)
    print('saved', p)


if __name__ == '__main__':
    main()
