#!/usr/bin/env python3
"""A->F->B->C->D->E 통합 실행의 종단 검증.

로그에서 각 모듈의 성공 신호를 확인하고, 지상 지도의 이상치 제거 효과를
2026-08-21 이전 값과 비교한다. 실행 후 한 번 돌리면 된다.
"""
import re, subprocess, sys, os
LOG = sys.argv[1] if len(sys.argv) > 1 else 'test/logs/e2e_full.log'
txt = open(LOG, encoding='utf-8', errors='replace').read()

def n(pat): return len(re.findall(pat, txt))
def first(pat):
    m = re.search(pat, txt)
    return m.group(0) if m else None

print('=' * 62)
print(' 모듈별 통과 신호')
print('=' * 62)
checks = [
    ('A 드론 비행 완료',  r'waypoint 320 / 320 도달|경로 재생 완료|path_status'),
    ('A 고도지도 저장',   r'saved elevation map to "maps/drone_elevation_map"'),
    ('F wheel 주행맵',    r'traversability_verdictor_wheel.*주행 가능 맵 발행 완료'),
    ('F leg 주행맵',      r'traversability_verdictor_leg.*주행 가능 맵 발행 완료'),
    ('F->B/C 게이트 개방', r'A→F 완료: 후속 B→C→D→E 시작 허용'),
    ('B 위치추정',        r'yaw_consistency_checker.*Initial yaw offset'),
    ('C wheel 주행 시작', r'wheel\.bt_navigator.*Begin navigating'),
    ('C leg 주행 시작',   r'leg\.bt_navigator.*Begin navigating'),
    ('C wheel 목표 도달', r'wheel\.bt_navigator.*Goal succeeded'),
    ('C leg 목표 도달',   r'leg\.bt_navigator.*Goal succeeded'),
    ('D wheel 지도 저장', r'saved elevation map to "maps/wheel_elevation_map"'),
    ('D leg 지도 저장',   r'saved elevation map to "maps/leg_elevation_map"'),
    ('E 병합 완료',       r'merge complete \(once\)'),
    ('E 병합 저장',       r'saved merged map'),
]
ok = True
for name, pat in checks:
    hit = re.search(pat, txt) is not None
    ok &= hit
    print('  [%s] %s' % ('OK' if hit else '__', name))

print()
print(' 실패 신호 (전부 0 이어야 한다)')
for name, pat in (('Goal ABORTED', r'Goal ABORTED'), ('0 poses', r'0 poses'),
                  ('경로계획 실패', r'Failed to create a plan'),
                  ('예외', r'Traceback'), ('프로세스 사망(스포너 제외)',
                   r'process has died(?!.*spawner)')):
    c = n(pat)
    if name.startswith('프로세스'):
        c = len([m for m in re.findall(r'\[ERROR\] \[(\w+)-\d+\]: process has died', txt)
                 if m != 'spawner'])
    print('  %-22s %d %s' % (name, c, '' if c == 0 else '  <-- 확인 필요'))
    ok &= (c == 0)

print()
print(' 이상치 필터 효과 (2026-08-21 도입)')
print(' %-8s %10s %10s %10s' % ('', '최저 z', '-5m 아래', '비율'))
BEFORE = {'leg': (-46.4, 65062, 5.0986), 'wheel': (-9.9, 1392, 0.0847)}
try:
    sys.path.insert(0, 'src/agconav_traversability/scripts')
    import numpy as np
    from eval_verdict_accuracy import load_map
    for r in ('wheel', 'leg'):
        z, xs, ys, res = load_map('maps/%s_elevation_map' % r)
        v = z[np.isfinite(z)]
        lo, cnt, pct = v.min(), int((v < -5).sum()), 100.0 * (v < -5).mean()
        b = BEFORE[r]
        print('  %-8s %10.1f %10d %9.4f%%   (이전 %.1f / %d / %.4f%%)'
              % (r, lo, cnt, pct, b[0], b[1], b[2]))
except Exception as e:
    print('  지도 분석 건너뜀:', e)

print()
print('=' * 62)
print(' 종합 판정:', 'PASS' if ok else 'FAIL')
print('=' * 62)
sys.exit(0 if ok else 1)
