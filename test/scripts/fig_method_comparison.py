#!/usr/bin/env python3
"""그림 3: 방식별 생성 지도와 4륜 주행성 판정 (100 x 100 m 축소 월드).

**판정과 지표는 원본 채점기 eval_100x100.py 의 정의를 그대로 쓴다** —
재구현하지 않는다. 그 파일이 논문 표 9 를 낸 채점기다.
    지면 판정  |z - 정답지형| <= GROUND_TOL(0.8 m)  -> 건물 지붕은 채점 제외
    통과 판정  인접 8셀 최대 단차 <= 0.08 m (wheel)
    연결성     통과 셀을 반지름 0.55 m 원판으로 침식한 뒤 최대 연결성분 / 박스 셀

**84 m 열은 거리 게이트를 점군 생성 때 센서 기준으로 걸고 매퍼 게이트는 껐다.**
매퍼 게이트(max_range_m 120)는 실비행에서는 센서를 따라가지만, 저장 점군을
feed_cloud.py 로 재생하면 센서 TF 가 없어 map 원점 기준이 되어 반경 120 m 구를
잘라낸다. 그러면 평가 박스 안 지도 셀이 539,000 으로 다른 방법(100만)의 절반이
되고, 연결성 분모가 절반이라 값이 2배로 부풀려진다(13.47% vs 수정 후 25.04%).
저고도 네 방법은 gicp 브랜치 매퍼를 쓰는데 그 매퍼에는 이 게이트가 없어
원래부터 잘리지 않는다. 다섯 열의 inbox_cells 가 같은지 캡션에 찍는다.
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
matplotlib.rcParams['font.family'] = 'Noto Sans CJK JP'
matplotlib.rcParams['axes.unicode_minus'] = False

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MMS = os.path.expanduser('~/projects/AG-CoNav-mm/src/agconav_traversability/scripts')
sys.path.insert(0, MMS)
import eval_100x100 as E                            # noqa: E402
from eval_verdict_accuracy import load_map, max_step  # noqa: E402
sys.path.insert(0, os.path.expanduser('~/projects/AG-CoNav-mm/run_results'))
from gt_traversable import building_mask              # noqa: E402

FIG3 = os.path.expanduser('~/projects/AG-CoNav-gicp/run_results/fig3')
OUT = os.path.join(WS, 'results/final_maps')

# **네 방법 모두 같은 bag 재생 경로**로 잰다. GICP/VGICP 는 녹화 스캔끼리
# 정합하는 오프라인 연산이고 FAST-LIO2 도 bag 으로 궤적을 뽑으므로, 재생이
# 이 방법들의 유일한 평가 경로다. 제안 방식도 같은 경로에 맞춰 비교를 균일하게 한다.
#
# 재생에는 대가가 있다 — 저장된 점을 20만 개 단위로 흘려 센서 위치를 잃으므로
# 매퍼의 거리·입사각 가중이 무력화된다. 제안 방식을 실비행으로 재면 연결성이
# 25.04% -> 48.46% 로 오른다(논문 4.4 에 기재). 다섯 열 모두 같은 손실을 안고
# 있으므로 열 간 비교는 성립하지만, 절대값은 각 방법의 하한으로 읽어야 한다.
#
# FAST-LIO2 는 **전체 궤적 Umeyama 정합**(align_window_s=3000)으로 만든 것이다.
# 초반 20초만으로 정합하면 궤적이 거의 한 점이라 회전이 구속되지 않아 z 축이
# 5.9도 기울고, 100 m 끝에서 높이가 10.3 m 어긋나 채점 대상이 17.6% 로 붕괴한다
# (연결성 4.53%). 전체 궤적으로 맞추면 59.7% / 32.25% 가 된다.
COLS = [(None,           '정답 지형 (기준)'),
        ('F_ours84',     '제안 방식 — 84 m'),
        ('fastlio_full', 'FAST-LIO2 — 5 m'),
        ('vgicp',        '정답 위치 + VGICP — 5 m'),
        ('gicp',         '정답 위치 + GICP — 5 m')]


def map_uri(tag):
    # 실비행 열만 저장 경로가 다르다(experiment.launch.py 의 maps_output).
    if tag == 'live84':
        return os.path.join(FIG3, 'live_maps', 'drone_elevation_map')
    return os.path.join(FIG3, 'trav_%s' % tag, 'maps', 'drone_elevation_map')


def verdict(tag):
    """eval_100x100.evaluate 와 같은 계산으로 pred/ok/ref 를 되돌린다."""
    uri = map_uri(tag)
    z, xs, ys, res = load_map(uri)
    ref = E.reference_grid(E.HM_PNG, xs, ys, E.BOX, E.HM_SIZE_Z, E.HM_POS_Z)
    X, Y = np.meshgrid(xs, ys)
    inbox = (X >= E.BOX[0]) & (X <= E.BOX[1]) & (Y >= E.BOX[2]) & (Y <= E.BOX[3])
    meas_ok = np.isfinite(z) & inbox
    ground = meas_ok & (np.abs(z - ref) <= E.GROUND_TOL)
    m_step, m_have = max_step(np.where(meas_ok, z, 0.0), meas_ok)
    t_step, t_have = max_step(ref, np.ones(ref.shape, bool))
    ok = ground & m_have & t_have
    pred = (m_step <= E.WHEEL_STEP) & ok
    return dict(z=z, xs=xs, ys=ys, ref=ref, inbox=inbox, ok=ok, pred=pred,
                meas_ok=meas_ok)


def main():
    ev = {t: json.load(open('%s/%s_eval100.json' % (FIG3, t), encoding='utf-8'))
          for t, _ in COLS if t}
    v = {t: verdict(t) for t, _ in COLS if t}
    box = E.BOX

    fig, axes = plt.subplots(2, 5, figsize=(26.5, 11.8))
    terr = plt.get_cmap('terrain').copy(); terr.set_bad('#c9c9c9')
    verd = ListedColormap(['#c9c9c9', '#2f8f4e', '#c0392b'])

    ref0 = v[COLS[1][0]]['ref']
    lo, hi = float(np.nanpercentile(ref0, 0.5)), float(np.nanpercentile(ref0, 99.5))
    ext = [box[0], box[1], box[2], box[3]]

    for c, (tag, title) in enumerate(COLS):
        a1, a2 = axes[0, c], axes[1, c]
        if tag is None:
            d = v[COLS[1][0]]
            bld0 = building_mask(d['xs'], d['ys'])
            gext0 = [float(d['xs'][0]), float(d['xs'][-1]),
                     float(d['ys'][0]), float(d['ys'][-1])]
            a1.imshow(np.ma.masked_where(bld0, d['ref']), origin='lower',
                      extent=gext0, cmap=terr, vmin=lo, vmax=hi,
                      interpolation='nearest')
            t_step, t_have = max_step(d['ref'], np.ones(d['ref'].shape, bool))
            bld = building_mask(d['xs'], d['ys'])
            gt_pred = (t_step <= E.WHEEL_STEP) & t_have & ~bld
            m = np.zeros(d['ref'].shape, np.uint8)
            m[gt_pred] = 1
            m[~bld & ~gt_pred] = 2
            a2.imshow(m, origin='lower', extent=gext0,
                      cmap=verd, vmin=0, vmax=2, interpolation='nearest')
            a1.set_title('%s\n정답 heightmap · 최대 단차 %.4f m'
                         % (title, float(t_step[t_have].max())), fontsize=12)
            # !! 회색의 정의가 방법 열과 다르다 !! 방법 열은 GROUND_TOL(정답에서
            # ±0.8 m 초과)로 빠지지만 정답 지형에는 측정값이 없어 적용할 수 없다.
            # 대신 건물 풋프린트로 뺐다. 비율은 우연히 비슷하나(38.2% vs 38~40%)
            # 정의가 다르므로 캡션에 밝힌다.
            a2.set_title('정답상 통과 가능 %.2f%%\n회색 = 건물 풋프린트 %.1f%% '
                         '(방법 열의 회색은 GROUND_TOL 기준)'
                         % (100 * gt_pred.mean(), 100 * bld0.mean()), fontsize=10.5)
        else:
            d, e = v[tag], ev[tag]
            w = e['wheel']
            ex = [float(d['xs'][0]), float(d['xs'][-1]),
                  float(d['ys'][0]), float(d['ys'][-1])]
            a1.imshow(np.ma.masked_invalid(d['z']), origin='lower', extent=ex,
                      cmap=terr, vmin=lo, vmax=hi, interpolation='nearest')
            a1.set_title('%s\n커버리지 %.2f%% · 높이오차 RMS %.3f m'
                         % (title, e['coverage'], e['height_err_rms'] or float('nan')),
                         fontsize=12)
            m = np.zeros(d['z'].shape, np.uint8)
            m[d['pred']] = 1
            m[d['ok'] & ~d['pred']] = 2
            a2.imshow(m, origin='lower', extent=ex, cmap=verd, vmin=0, vmax=2,
                      interpolation='nearest')
            a2.set_title('연결성 %.2f%% · 덩어리 %d개\n'
                         'wheel FN %.2f%% (채점 대상 %.1f%% 에서만)'
                         % (w['largest_pct'], w['components'], w['fn_rate'],
                            100.0 * e['scored_cells'] / e['inbox_cells']),
                         fontsize=11)
        for ax in (a1, a2):
            ax.set_xlim(box[0], box[1]); ax.set_ylim(box[2], box[3])
            ax.grid(alpha=0.2, lw=0.4)
        a2.set_xlabel('World X [m]')
        a1.tick_params(labelbottom=False)
        a1.set_ylabel('2.5D 고도 지도 (모듈 A)' if c == 0 else '')
        a2.set_ylabel('4륜 주행성 판정 (모듈 F)' if c == 0 else '')

    sm = plt.cm.ScalarMappable(cmap=terr, norm=plt.Normalize(vmin=lo, vmax=hi))
    cax = fig.add_axes([0.33, 0.022, 0.34, 0.013])
    fig.colorbar(sm, cax=cax, orientation='horizontal').set_label('고도 [m]', fontsize=11)
    axes[1, 0].legend(handles=[Patch(facecolor='#2f8f4e', label='통과 가능'),
                               Patch(facecolor='#c0392b', label='막힘'),
                               Patch(facecolor='#c9c9c9', label='채점 제외 / 미관측')],
                      loc='lower left', fontsize=10, framealpha=0.93)

    cells = sorted({ev[t]['inbox_cells'] for t, _ in COLS if t})
    # 연결성 분모(평가 박스 안 지도 셀 수)가 열마다 크게 다르면 비교가 깨진다.
    # 실측 998,000~1,000,000 로 0.2% 차이라 무시할 수준이지만 값을 찍어 둔다.
    spread = 100.0 * (cells[-1] - cells[0]) / cells[-1]
    same = ('평가 박스 셀 수 %s~%s (편차 %.1f%%)'
            % (format(cells[0], ','), format(cells[-1], ','), spread)
            if len(cells) > 1 else '평가 박스 셀 수 %s 동일' % format(cells[0], ','))
    fig.suptitle('방식별 생성 지도와 4륜 주행성 판정 — 100 × 100 m 동일 지형 · '
                 '채점기 eval_100x100.py · 0.10 m/cell\n'
                 '네 방법 모두 동일한 bag 재생 경로로 측정(재생이 비교 대상 '
                 '방법들의 유일한 평가 경로). %s' % same,
                 fontsize=14, y=0.985)
    fig.subplots_adjust(top=0.87, bottom=0.085, left=0.035, right=0.985,
                        hspace=0.30, wspace=0.10)
    p = os.path.join(OUT, 'method_comparison.png')
    fig.savefig(p, dpi=130, bbox_inches='tight')
    print('saved', p)
    print(same)


if __name__ == '__main__':
    main()
