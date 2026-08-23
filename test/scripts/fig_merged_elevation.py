#!/usr/bin/env python3
"""그림: 모듈 E 융합 지도 (2026-08-22 종단 실행).

기존 results/final_maps/merged_elevation.png 은 8/17 자 6525 x 4870 지도라
논문 표 12(5786 x 4855)와 어긋나고, 지상 이상치 필터 도입 이전이라 leg
오측이 만든 원형 아티팩트가 그대로 찍혀 있었다. 오늘 산출물로 다시 만든다.

두 칸으로 나눈 이유: 이 지도는 90% 가 고도 11 m 이하 지면이고 상위 10% 만
건물(40~80 m)이다. 한 색 범위로 그리면 지면이 통째로 단색이 되어 지상
로봇이 보탠 영역이 안 보인다.
"""
import os, sys
import numpy as np
from scipy import ndimage
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
matplotlib.rcParams['font.family'] = 'Noto Sans CJK JP'
matplotlib.rcParams['axes.unicode_minus'] = False

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(WS, 'src/agconav_traversability/scripts'))
from eval_verdict_accuracy import load_map          # noqa: E402

OUT = os.path.join(WS, 'results/final_maps')
F = 3                                               # 표시용 축소 (2810 만 셀)


def shrink(z):
    ny, nx = z.shape
    hh, ww = ny // F * F, nx // F * F
    with np.errstate(invalid='ignore'):
        return np.nanmean(z[:hh, :ww].reshape(hh // F, F, ww // F, F), axis=(1, 3))


def main():
    z, xs, ys, res = load_map(os.path.join(WS, 'maps/merged_elevation_map'))
    ok = np.isfinite(z)
    ny, nx = z.shape
    print('merged %d x %d, valid %.1f%%, z %.2f ~ %.2f'
          % (nx, ny, 100 * ok.mean(), np.nanmin(z), np.nanmax(z)))
    zs = shrink(z)
    extent = [xs[0], xs[-1], ys[0], ys[-1]]

    # 지상 로봇이 관측한 영역 — 병합 규칙(지상 우선)이 실제로 작동한 자리다.
    ground = {}
    for r, col in (('wheel', '#ff2d2d'), ('leg', '#ffd400')):
        gz, gxs, gys, _ = load_map(os.path.join(WS, 'maps/%s_elevation_map' % r))
        m = ndimage.binary_fill_holes(np.isfinite(gz))
        m = ndimage.binary_opening(m, np.ones((5, 5)))
        ground[r] = (m, [gxs[0], gxs[-1], gys[0], gys[-1]], col, int(np.isfinite(gz).sum()))
        print('%-5s %d x %d cells, valid %d' % (r, gz.shape[1], gz.shape[0], ground[r][3]))

    fig, axes = plt.subplots(1, 2, figsize=(21.0, 8.6))
    cmap = plt.get_cmap('terrain').copy()
    cmap.set_bad('#d0d0d0')
    masked = np.ma.masked_invalid(zs)

    for ax, (lo, hi, tag, sub) in zip(axes, [
            (float(np.nanmin(z)), float(np.nanpercentile(z, 99.9)), 'a',
             '전체 고도 범위 — 건물 포함'),
            (0.0, 12.0, 'b', '지면 대역(0~12 m)만 색 범위 — 지상 관측 기여가 보인다')]):
        im = ax.imshow(masked, origin='lower', extent=extent, cmap=cmap,
                       vmin=lo, vmax=hi, interpolation='nearest')
        cb = fig.colorbar(im, ax=ax, fraction=0.036, pad=0.02,
                          extend='max' if tag == 'b' else 'neither')
        cb.set_label('고도 [m]', fontsize=11)
        ax.set_title('(%s) %s' % (tag, sub), fontsize=13)
        ax.set_xlabel('World X [m]'); ax.set_ylabel('World Y [m]')
        ax.grid(alpha=0.22, lw=0.5)

    # (b) 에만 지상 관측 영역 경계를 얹는다.
    for r, (m, ex, col, cnt) in ground.items():
        ms = m[:m.shape[0] // F * F, :m.shape[1] // F * F].reshape(
            m.shape[0] // F, F, m.shape[1] // F, F).any(axis=(1, 3))
        gx = np.linspace(ex[0], ex[1], ms.shape[1])
        gy = np.linspace(ex[2], ex[3], ms.shape[0])
        axes[1].contour(gx, gy, ms.astype(float), levels=[0.5],
                        colors=[col], linewidths=2.0)
    axes[1].legend(handles=[
        Line2D([], [], color='#ff2d2d', lw=2.0,
               label='4륜 관측 영역 (%s 셀)' % format(ground['wheel'][3], ',')),
        Line2D([], [], color='#ffd400', lw=2.0,
               label='4족 관측 영역 (%s 셀)' % format(ground['leg'][3], ','))],
        loc='lower right', fontsize=10, framealpha=0.93)

    fig.suptitle('모듈 E 융합 지도 — 드론 + 4륜 + 4족 2.5D 지도 병합\n'
                 '%d × %d 셀 · %.2f m/cell · %.1f × %.1f m · 유효 셀 %.1f%% · '
                 '2026-08-22 종단 실행'
                 % (nx, ny, res, nx * res, ny * res, 100 * ok.mean()),
                 fontsize=15, y=0.985)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    p = os.path.join(OUT, 'merged_elevation.png')
    fig.savefig(p, dpi=150)
    print('saved', p)


if __name__ == '__main__':
    main()
