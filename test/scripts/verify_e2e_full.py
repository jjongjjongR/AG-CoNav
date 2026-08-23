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

# 실패 신호는 **주행 구간만** 센다. 모듈 E 저장을 확인한 뒤 launch 를 SIGINT
# 로 내리면, 아직 spin 중이던 rclpy 노드들이 ExternalShutdownException 과
# "rcl_shutdown() was called" 를 뱉으며 exit 1 로 죽는다. 산출물이 전부 저장된
# 뒤라 무해한데 전체 로그를 그대로 세면 이 종료 잡음이 판정을 FAIL 로 뒤집는다
# — 2026-08-22 실행에서 예외 11·사망 11 이 **전부** SIGINT 이후였고 주행 구간은
# 0 이었다. 종료 신호가 없는 로그는 분할점이 EOF 라 예전과 동작이 같다.
# 종료 구간 개수는 숨기지 않고 옆에 같이 찍는다.
_sig = re.search(r'signal_handler\(SIGINT/SIGTERM\)', txt)
run_txt, down_txt = (txt[:_sig.start()], txt[_sig.start():]) if _sig else (txt, '')

def count(pat, s):
    if pat is None:  # 스포너는 정상적으로 할 일을 마치고 빠지므로 제외
        return len([m for m in re.findall(r'\[ERROR\] \[(\w+)-\d+\]: process has died', s)
                    if m != 'spawner'])
    return len(re.findall(pat, s))

print()
print(' 실패 신호 — 주행 구간 (전부 0 이어야 한다)')
for name, pat in (('Goal ABORTED', r'Goal ABORTED'), ('0 poses', r'0 poses'),
                  ('경로계획 실패', r'Failed to create a plan'),
                  ('예외', r'Traceback'), ('프로세스 사망(스포너 제외)', None)):
    c, d = count(pat, run_txt), count(pat, down_txt)
    print('  %-22s %d %s%s' % (name, c, '' if c == 0 else '  <-- 확인 필요',
                               '' if d == 0 else '   [종료 구간 %d — 판정 제외]' % d))
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
