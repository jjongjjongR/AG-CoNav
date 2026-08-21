#!/usr/bin/env python3
"""실험 1 — **등판 한계** 일괄 실행 (단계식 경사로).

    exp1_ramp_sweep.py [--speeds 1.0 0.5 1.5] [--reps 3] [--resume]

단계식 경사로(5,10,15,20,25,30도)를 한 번 오르게 해서 멈춘 지점의 각도를
그 컨트롤러의 등판 한계로 본다. 각도별 월드를 따로 도는 것보다 6배 빠르고,
재는 값도 질문에 더 가깝다("몇 도까지 오르는가").

명령 속도도 변수로 넣는다. 등판에서는 빠른 명령이 유리하지 않을 수 있다 —
정책이 보폭을 키우다 접지를 잃는 쪽으로 갈 수 있어서다. 두 컨트롤러 모두
같은 m/s 명령을 받는다(축 단위는 런치가 맞춘다).
"""
import argparse
import json
import os
import subprocess
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUN = os.path.join(ROOT, 'test', 'scripts', 'run_op.sh')
WORLD = os.path.join(ROOT, 'test', 'worlds', 'ramp.sdf')
RESULTS = os.path.join(ROOT, 'test', 'results', 'ramp')

CONTROLLERS = [
    ('rl_robot_lab', 'rl', 'robot_lab'),      # 현재 운용 중
    ('rl_legged_gym', 'rl', 'legged_gym'),
    ('rl_himloco', 'rl', 'himloco'),
    ('guide', 'guide', 'none'),               # Unitree 공식
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speeds', type=float, nargs='+', default=[1.0])
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--resume', action='store_true')
    a = ap.parse_args()
    os.makedirs(RESULTS, exist_ok=True)

    todo = [(spd, name, ctl, pol, rep)
            for spd in a.speeds
            for name, ctl, pol in CONTROLLERS
            for rep in range(1, a.reps + 1)]
    print('총 %d 회' % len(todo), flush=True)
    t0, done = time.time(), 0
    for i, (spd, name, ctl, pol, rep) in enumerate(todo, 1):
        tag = 'ramp_%s_v%.1f_r%d' % (name, spd, rep)
        out = os.path.join(RESULTS, tag + '.json')
        if a.resume and os.path.exists(out) and os.path.getsize(out) > 0:
            continue
        el = (time.time() - t0) / 60
        eta = (el / done * (len(todo) - i)) if done else 0
        print('[%2d/%2d] %-30s (경과 %.0f분, 남은 %.0f분)'
              % (i, len(todo), tag, el, eta), flush=True)
        subprocess.run([RUN, WORLD, tag, 'ramp', '0', ctl, pol, str(spd), out],
                       cwd=ROOT, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        done += 1
        try:
            r = json.load(open(out))
            print('         -> %-6s 최대각 %2s도  등반 %5.2f m  x %5.2f  이탈 %4.2f m'
                  % (r.get('verdict'), r.get('max_angle'), r.get('climb') or 0,
                     r.get('x_max') or 0, r.get('drift_y') or 0), flush=True)
        except Exception as e:
            print('         -> 읽기 실패 %s' % e, flush=True)
    print('완료. %.1f 분' % ((time.time() - t0) / 60), flush=True)


if __name__ == '__main__':
    main()
