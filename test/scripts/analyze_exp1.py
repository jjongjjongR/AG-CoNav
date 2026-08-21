#!/usr/bin/env python3
"""실험 1 결과를 컨트롤러 x 지형 표로 집계한다.

    analyze_exp1.py [--dir test/results/exp1] [--tsv out.tsv]

반복 시행의 평균을 낸다. 판정은 시행마다 통과/실패/전복/기립X 로 갈리므로
**성공률**과 **평균 진출거리**를 같이 본다 — 한쪽만 보면 "3회 중 1회만 겨우
넘은 것"과 "3회 다 안정적으로 넘은 것"이 구분되지 않는다.
"""
import argparse
import collections
import glob
import json
import os
import statistics as st

ORDER = ['rl_robot_lab', 'rl_legged_gym', 'rl_himloco', 'guide']
LABEL = {'rl_robot_lab': 'RL robot_lab(운용)', 'rl_legged_gym': 'RL legged_gym',
         'rl_himloco': 'RL himloco', 'guide': 'unitree_guide(공식)'}


def key_of(tag):
    # slope15_rl_robot_lab_r2 -> ('slope', 15, 'rl_robot_lab')
    body, _, _rep = tag.rpartition('_r')
    kind = 'slope' if body.startswith('slope') else 'step'
    rest = body[len(kind):]
    val, _, name = rest.partition('_')
    return kind, int(val), name


def mean(xs):
    xs = [x for x in xs if x is not None]
    return st.mean(xs) if xs else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='test/results/exp1')
    ap.add_argument('--tsv', default='')
    a = ap.parse_args()

    runs = collections.defaultdict(list)
    for f in sorted(glob.glob(os.path.join(a.dir, '*.json'))):
        try:
            d = json.load(open(f))
        except Exception:
            continue
        runs[key_of(os.path.basename(f)[:-5])].append(d)

    rows = []
    for (kind, val, name), ds in sorted(runs.items()):
        ok = [d for d in ds if d.get('verdict') == '통과']
        rows.append(dict(
            kind=kind, value=val, ctrl=name, n=len(ds),
            pass_n=len(ok), pass_pct=100.0 * len(ok) / max(1, len(ds)),
            advance=mean([d.get('advance') for d in ds]),
            speed=mean([d.get('speed_actual') for d in ds]),
            tilt=mean([d.get('tilt_max') for d in ds]),
            climb=mean([d.get('climb') for d in ds]),
            z=mean([d.get('z_stand') for d in ds]),
            verdicts=collections.Counter(d.get('verdict') for d in ds)))

    for kind in ('slope', 'step'):
        sub = [r for r in rows if r['kind'] == kind]
        if not sub:
            continue
        unit = '도' if kind == 'slope' else 'mm'
        print('\n=== %s ===' % ('경사' if kind == 'slope' else '단차'))
        print('%-20s %6s %7s %8s %8s %7s %7s  %s' % (
            '컨트롤러', unit, '성공률', '진출m', '속도m/s', '기울기', '등반m', '판정'))
        for r in sorted(sub, key=lambda r: (r['value'], ORDER.index(r['ctrl'])
                                            if r['ctrl'] in ORDER else 9)):
            v = ' '.join('%s%d' % (k, n) for k, n in r['verdicts'].most_common())
            print('%-20s %6d %4d/%-2d %8.2f %8.2f %7.1f %7.2f  %s' % (
                LABEL.get(r['ctrl'], r['ctrl']), r['value'],
                r['pass_n'], r['n'], r['advance'], r['speed'],
                r['tilt'], r['climb'], v))

    # 컨트롤러별 등판 한계: 성공률 60% 이상을 넘긴 가장 큰 경사
    print('\n=== 등판 한계 (성공률 60% 이상인 가장 큰 경사) ===')
    for name in ORDER:
        best = None
        for r in sorted([r for r in rows if r['kind'] == 'slope'
                         and r['ctrl'] == name], key=lambda r: r['value']):
            if r['pass_pct'] >= 60.0:
                best = r['value']
        print('  %-20s %s' % (LABEL.get(name, name),
                              ('%d도' % best) if best is not None else '평지도 실패'))

    if a.tsv:
        with open(a.tsv, 'w') as f:
            f.write('kind\tvalue\tctrl\tn\tpass\tpass_pct\tadvance\tspeed\ttilt\tclimb\tz_stand\n')
            for r in rows:
                f.write('%s\t%d\t%s\t%d\t%d\t%.1f\t%.3f\t%.3f\t%.2f\t%.3f\t%.3f\n' % (
                    r['kind'], r['value'], r['ctrl'], r['n'], r['pass_n'],
                    r['pass_pct'], r['advance'], r['speed'], r['tilt'],
                    r['climb'], r['z']))
        print('\n%s 저장' % a.tsv)


if __name__ == '__main__':
    main()
