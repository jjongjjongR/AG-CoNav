#!/usr/bin/env python3
"""wheel(A300) 지형 주파 결과 집계. 문서 12 재확인용.

    analyze_wheel.py [--dir test/results/wheel]
"""
import argparse
import collections
import glob
import json
import os
import statistics as st


def mean(xs):
    xs = [x for x in xs if x is not None]
    return st.mean(xs) if xs else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='test/results/wheel')
    a = ap.parse_args()
    runs = collections.defaultdict(list)
    for f in sorted(glob.glob(os.path.join(a.dir, '*.json'))):
        try:
            d = json.load(open(f))
        except Exception:
            continue
        runs[(d.get('kind'), d.get('value'))].append(d)

    for kind, unit in (('slope', '도'), ('step', 'mm')):
        sub = {k: v for k, v in runs.items() if k[0] == kind}
        if not sub:
            continue
        print('\n=== wheel %s ===' % ('경사' if kind == 'slope' else '단차'))
        print('%8s %8s %9s %9s %8s %8s  %s'
              % (unit, '통과', '진출m', '속도m/s', '기울기', '이탈m', '판정'))
        for (_k, val), ds in sorted(sub.items(), key=lambda kv: kv[0][1]):
            ok = [d for d in ds if d.get('verdict') == '통과']
            # 측정실패는 분모에서 빼되 별도 표기한다.
            bad = [d for d in ds if d.get('verdict') in ('측정실패', '스폰실패')]
            good = [d for d in ds if d not in bad]
            v = ' '.join('%s%d' % (k, n) for k, n in
                         collections.Counter(d.get('verdict') for d in ds).most_common())
            print('%8g %4d/%-3d %9.2f %9.3f %8.1f %8.2f  %s' % (
                val, len(ok), len(good) if good else len(ds),
                mean([d.get('advance') for d in good]),
                mean([d.get('speed_actual') for d in good]),
                mean([d.get('tilt_max') for d in good]),
                mean([d.get('drift_y') for d in good]), v))

    print('\n=== 한계 (통과가 과반인 최대값) ===')
    for kind, unit in (('slope', '도'), ('step', 'mm')):
        best = None
        for (_k, val), ds in sorted([(k, v) for k, v in runs.items()
                                     if k[0] == kind], key=lambda kv: kv[0][1]):
            good = [d for d in ds if d.get('verdict') not in ('측정실패', '스폰실패')]
            if not good:
                continue
            ok = [d for d in good if d.get('verdict') == '통과']
            if len(ok) * 2 > len(good):
                best = val
        print('  %s: %s' % ('경사' if kind == 'slope' else '단차',
                            ('%g%s' % (best, unit)) if best is not None else '없음'))


if __name__ == '__main__':
    main()
