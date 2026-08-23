#!/usr/bin/env python3
"""그림: 저고도(5 m AGL) 스캔과 84 m 고정 고도 스캔의 비행 경로 비교.

논문 4.3 "고도를 고정하여 안전성을 확보했다" 의 근거 그림. 표 14 의 숫자가
전부 경로 형상 이야기인데 그림이 없었다.

**두 경로 모두 실제 경로다.**
  저고도  test/configs/path_100x100_5m_4m_5mps.yaml (5,251 웨이포인트).
          커밋 02926e2(실험 인프라 정리)에서 지워졌던 것을 git 이력에서
          되살렸다(4d73d5f). 파싱하면 논문 값이 그대로 나온다 —
          z 6.20~13.78 m(변동폭 7.58), 3D 2847.4 m, 수평 2700.0 m,
          수직 총량 453.8 m.
  84 m    평면 lawnmower 라 결정적으로 재구성된다. 같은 100x100 영역에
          간격 2.941 m -> 35 라인 x 2점 = 70 웨이포인트,
          35 x 100 + 34 x 2.941 = 3600.0 m 로 논문 값과 일치한다.
"""
import os
import re

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
matplotlib.rcParams['font.family'] = 'Noto Sans CJK JP'
matplotlib.rcParams['axes.unicode_minus'] = False

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(WS, 'results/final_maps')
LOW = os.path.join(WS, 'test/configs/path_100x100_5m_4m_5mps.yaml')

C_HI, C_LO = '#1b6ca8', '#d1495b'          # 84 m / 저고도
POS = re.compile(r'-\s*position:\s*\n\s*x:\s*(-?[\d.eE+]+)\s*\n'
                 r'\s*y:\s*(-?[\d.eE+]+)\s*\n\s*z:\s*(-?[\d.eE+]+)')


def load_low():
    txt = open(LOW, encoding='utf-8').read()
    return np.array([[float(a), float(b), float(c)]
                     for a, b, c in POS.findall(txt)])


def build_high(low, spacing=2.941, alt=84.0):
    """같은 영역을 덮는 84 m 평면 lawnmower. 라인당 양 끝 2점뿐이다."""
    x0, x1 = low[:, 0].min(), low[:, 0].max()
    y0, y1 = low[:, 1].min(), low[:, 1].max()
    n = int(round((y1 - y0) / spacing)) + 1
    pts = []
    for i in range(n):
        y = y0 + i * (y1 - y0) / (n - 1)
        pts += [[x0, y, alt], [x1, y, alt]] if i % 2 == 0 \
            else [[x1, y, alt], [x0, y, alt]]
    return np.array(pts)


def stats(p):
    d = np.diff(p, axis=0)
    return dict(n=len(p),
                l3d=float(np.linalg.norm(d, axis=1).sum()),
                lh=float(np.hypot(d[:, 0], d[:, 1]).sum()),
                lv=float(np.abs(d[:, 2]).sum()),
                zmin=float(p[:, 2].min()), zmax=float(p[:, 2].max()))


def main():
    low = load_low()
    high = build_high(low)
    sl, sh = stats(low), stats(high)
    print('저고도 %d wp  3D %.1f  수직 %.1f  z %.2f~%.2f'
          % (sl['n'], sl['l3d'], sl['lv'], sl['zmin'], sl['zmax']))
    print('84 m   %d wp  3D %.1f  수직 %.1f  z %.2f~%.2f'
          % (sh['n'], sh['l3d'], sh['lv'], sh['zmin'], sh['zmax']))

    fig = plt.figure(figsize=(19.0, 9.6))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.05, 1.35, 0.95],
                          height_ratios=[1, 1], hspace=0.30, wspace=0.26)

    # ── (a) 평면 경로 ────────────────────────────────────────────────
    ax = fig.add_subplot(gs[:, 0])
    ax.plot(low[:, 0], low[:, 1], '-', color=C_LO, lw=0.7, alpha=0.85)
    ax.plot(high[:, 0], high[:, 1], '-', color=C_HI, lw=1.0, alpha=0.9)
    ax.set_aspect('equal')
    ax.set_xlabel('World X [m]'); ax.set_ylabel('World Y [m]')
    ax.set_title('(a) 평면 경로 — 같은 100 × 100 m 영역', fontsize=13)
    ax.grid(alpha=0.25, lw=0.5)
    ax.legend(handles=[
        Line2D([], [], color=C_HI, lw=1.6,
               label='84 m 고정 · 간격 2.941 m · %d wp' % sh['n']),
        Line2D([], [], color=C_LO, lw=1.6,
               label='5 m AGL · 간격 4 m · %s wp' % format(sl['n'], ','))],
        loc='upper center', bbox_to_anchor=(0.5, -0.09), fontsize=10)

    # ── (b) 고도 단면 — 축을 끊는다 (84 m 와 6~14 m 를 한 눈금에 못 둔다) ──
    def along(p):
        return np.concatenate([[0], np.cumsum(np.hypot(*np.diff(p, axis=0)[:, :2].T))])

    ax_t = fig.add_subplot(gs[0, 1])
    ax_b = fig.add_subplot(gs[1, 1], sharex=ax_t)
    ax_t.plot(along(high), high[:, 2], '-', color=C_HI, lw=1.8)
    ax_b.plot(along(low), low[:, 2], '-', color=C_LO, lw=0.9)
    ax_t.set_ylim(80, 88); ax_b.set_ylim(4, 16)
    ax_t.spines['bottom'].set_visible(False)
    ax_b.spines['top'].set_visible(False)
    ax_t.tick_params(labelbottom=False, bottom=False)
    for ax_, y_ in ((ax_t, 0), (ax_b, 1)):
        ax_.plot([0, 1], [y_, y_], transform=ax_.transAxes, color='k',
                 lw=1.0, ls=(0, (4, 4)), clip_on=False)
    ax_t.set_title('(b) 비행 고도 단면 — 축을 끊어 그렸다', fontsize=13)
    ax_b.set_xlabel('수평 이동 거리 [m]')
    ax_t.set_ylabel('고도 [m]'); ax_b.set_ylabel('고도 [m]')
    for ax_ in (ax_t, ax_b):
        ax_.grid(alpha=0.25, lw=0.5)
    ax_t.annotate('84 m 고정 — 수직 이동 0 m',
                  xy=(0.50, 0.62), xycoords='axes fraction', color=C_HI,
                  fontsize=11, ha='center')
    ax_b.annotate('지형 추종 6.20 ~ 13.78 m\n수직 이동 총량 453.8 m · 상승률 상한 4.2 m/s',
                  xy=(0.50, 0.06), xycoords='axes fraction', color=C_LO,
                  fontsize=11, ha='center')

    # ── (c) 측정값 ──────────────────────────────────────────────────
    ax = fig.add_subplot(gs[:, 2]); ax.axis('off')
    rows = [
        ('유효 비행 시간',   '18분 40초',        '38분 00초'),
        ('실효 속도',        '3.21 m/s',         '1.25 m/s'),
        ('지령 대비',        '54%',              '25%'),
        ('3D 경로 길이',     '3,600.0 m',        '2,847.4 m'),
        ('수직 이동 총량',   '0 m',              '453.8 m'),
        ('고도 변동 폭',     '0 m',              '7.58 m'),
        ('waypoint',         '70개',             '5,251개'),
        ('안전 여유 보정',   '0회',              '11 / 21회'),
        ('완주',             '완주',             '미완주'),
        ('wheel FN',         '12.95%',           '16.52%'),
        ('연결성',           '27.84%',           '25.88%'),
    ]
    ax.set_title('(c) 측정값 — 논문 표 13 · 14', fontsize=13, pad=16)
    y = 0.955
    ax.text(0.42, y + 0.035, '84 m 고정', color=C_HI, fontsize=11.5,
            ha='center', fontweight='bold')
    ax.text(0.83, y + 0.035, '5 m 저고도', color=C_LO, fontsize=11.5,
            ha='center', fontweight='bold')
    for i, (k, a, b) in enumerate(rows):
        yy = y - i * 0.082
        if i % 2 == 0:
            ax.add_patch(plt.Rectangle((0.0, yy - 0.030), 1.0, 0.070,
                                       fc='#f2f5f8', ec='none',
                                       transform=ax.transAxes))
        ax.text(0.02, yy, k, fontsize=11, va='center')
        ax.text(0.42, yy, a, fontsize=11, va='center', ha='center', color=C_HI)
        ax.text(0.83, yy, b, fontsize=11, va='center', ha='center', color=C_LO)

    fig.suptitle('고정 고도 스캔과 저고도 스캔의 비행 경로 비교 '
                 '— 동일 100 × 100 m 영역',
                 fontsize=15.5, y=0.975)
    fig.subplots_adjust(top=0.90, bottom=0.09, left=0.05, right=0.985)
    p = os.path.join(OUT, 'flight_profile_84m_vs_5m.png')
    fig.savefig(p, dpi=150, bbox_inches='tight')
    print('saved', p)


if __name__ == '__main__':
    main()
