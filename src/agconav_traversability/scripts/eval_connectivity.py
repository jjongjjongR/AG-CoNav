#!/usr/bin/env python3
"""저장된 드론 2.5D 지도들에 대해 **FN 과 연결성을 나란히** 잰다.

    eval_connectivity.py <maps_test/TAG/drone_elevation_map> ... [--box x0 x1 y0 y1]

왜 필요한가: 지금까지 조합(속도 × 간격)의 순위를 FN 비율 하나로 매겼다.
그런데 종단 테스트에서 통과가능 78.9% 짜리 지도가 로봇 폭을 못 통과해 Nav2
계획이 19번 전부 실패했다(`10. 종단 테스트 — 전체 맵 주행 검증.md` §6).
FN 이 낮다고 잘 다닐 수 있는 것이 아니라면, **FN 순위와 연결성 순위가 같은지**
확인해야 한다. 다르면 지금까지의 최적 조합 선택이 근거를 잃는다.

연결성 정의: 통과가능 셀을 로봇 반지름만큼 **원판**으로 침식한 뒤 남는 최대
연결 성분의 비율. 사각형 침식은 대각선을 과하게 깎아 통과 면적을 실제보다
낮게 만든다(이전에 v8 조합을 37% vs 76% 로 잘못 평가한 적이 있다).

밴드 실험은 출발/목적지가 정해져 있지 않으므로 "특정 두 점이 이어지는가" 대신
**최대 연결 성분 비율**을 쓴다. 이게 클수록 임의의 두 지점이 이어질 확률이
높다.
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_verdict_accuracy import (LEG_STEP, WHEEL_STEP, load_map,  # noqa: E402
                                   max_step)
from eval_verdict_aligned import WDIR, params, reference_grid  # noqa: E402

GROUND_TOL = 0.8
# A300 외접 지름 1.094 m -> 반지름 0.547. Nav2 inflation_radius 도 0.55 라 같이 쓴다.
RADIUS = {'wheel': 0.55, 'leg': 0.40}


def disk(r_cells):
    k = int(np.ceil(r_cells))
    y, x = np.ogrid[-k:k + 1, -k:k + 1]
    return (x * x + y * y) <= r_cells ** 2


def one(uri, box, robot):
    z, xs, ys, res = load_map(uri)
    lx, ly, sz, pz = params(WDIR)
    ref = reference_grid(WDIR + '/mesh/height_map.png', xs, ys, lx, ly, sz, pz)

    X, Y = np.meshgrid(xs, ys)
    inbox = (X >= box[0]) & (X <= box[1]) & (Y >= box[2]) & (Y <= box[3])
    meas_ok = np.isfinite(z) & inbox
    ground = meas_ok & (np.abs(z - ref) <= GROUND_TOL)

    m_step, m_have = max_step(np.where(meas_ok, z, 0.0), meas_ok)
    t_step, t_have = max_step(ref, np.ones(ref.shape, bool))
    ok = ground & m_have & t_have
    th = WHEEL_STEP if robot == 'wheel' else LEG_STEP
    truth = (t_step <= th) & ok
    pred = (m_step <= th) & ok
    fn = int((truth & ~pred).sum())
    fn_rate = 100.0 * fn / max(1, int(truth.sum()))

    # 연결성: 평가 박스 안에서 "통과 가능으로 판정된" 셀만 자유로 본다.
    free = pred & inbox
    er = ndimage.binary_erosion(free, structure=disk(RADIUS[robot] / res))
    lab, n = ndimage.label(er)
    if n:
        sizes = np.bincount(lab.ravel())
        sizes[0] = 0
        largest = 100.0 * sizes.max() / max(1, int(inbox.sum()))
    else:
        largest = 0.0
    return dict(cells=int(ok.sum()),
                fn_rate=fn_rate,
                pred_pass=100.0 * pred[inbox].mean(),
                eroded_free=100.0 * er[inbox].mean(),
                components=int(n),
                largest_pct=largest)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('maps', nargs='+')
    p.add_argument('--robot', default='wheel', choices=('wheel', 'leg'))
    p.add_argument('--box', nargs=4, type=float,
                   default=[-288.94, 288.94, -40.0, 40.0])
    a = p.parse_args()

    print('%-14s %8s %10s %11s %9s %11s' % (
        '조합', 'FN%', '통과판정%', '침식후자유%', '덩어리', '최대덩어리%'))
    rows = []
    for m in a.maps:
        tag = os.path.basename(os.path.dirname(m.rstrip('/')))
        try:
            r = one(m, a.box, a.robot)
        except Exception as ex:                      # noqa: BLE001
            print('%-14s  실패: %s' % (tag, ex))
            continue
        rows.append((tag, r))
        print('%-14s %7.3f%% %9.2f%% %10.2f%% %9d %10.2f%%'
              % (tag, r['fn_rate'], r['pred_pass'], r['eroded_free'],
                 r['components'], r['largest_pct']))

    if len(rows) > 1:
        fn_rank = [t for t, _ in sorted(rows, key=lambda kv: kv[1]['fn_rate'])]
        cn_rank = [t for t, _ in sorted(rows, key=lambda kv: -kv[1]['largest_pct'])]
        print('\nFN  좋은 순: %s' % ' > '.join(fn_rank))
        print('연결 좋은 순: %s' % ' > '.join(cn_rank))
        print('순위 일치: %s' % ('예' if fn_rank == cn_rank else '아니오'))
    print(json.dumps([{'tag': t, **r} for t, r in rows], ensure_ascii=False))


if __name__ == '__main__':
    main()
