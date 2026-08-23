#!/usr/bin/env python3
"""실험 1 일괄 실행. 컨트롤러 x 지형 x 반복을 순서대로 돌린다.

    exp1_sweep.py [--phase slope|step|all] [--reps 3] [--resume]

한 번에 하나씩만 돌린다. 병렬로 돌리면 안 된다 — Gazebo 를 매번 새로 띄우고
cleanup 이 이름으로 프로세스를 죽이기 때문에 서로를 죽인다. 그리고 같은
기계에서 다른 작업이 함께 돌면 RTF 가 흔들려 속도 비교가 오염된다.

경사를 먼저 돈다. 이 실험의 판정 질문이 "Go2 가 15도, 가능하면 20도를
올라가는가" 라서다. 단차는 그 다음이다.
"""
import argparse
import json
import os
import subprocess
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUN = os.path.join(ROOT, 'test', 'scripts', 'run_op.sh')
WORLDS = os.path.join(ROOT, 'test', 'worlds')
RESULTS = os.path.join(ROOT, 'test', 'results', 'exp1')

# (이름, 컨트롤러, 정책). unitree_guide 가 Unitree 공식 컨트롤러다.
CONTROLLERS = [
    ('rl_robot_lab', 'rl', 'robot_lab'),      # 현재 운용 중
    ('rl_legged_gym', 'rl', 'legged_gym'),
    ('rl_himloco', 'rl', 'himloco'),
    ('guide', 'guide', 'none'),               # Unitree 공식
]

SLOPES = [0, 10, 15, 20, 25, 30]
STEPS = [100, 150, 200, 250]
SPEED = 1.0     # 두 컨트롤러에 같은 m/s 명령을 준다(축 단위는 런치가 맞춘다)


def cases(phase, reps):
    kinds = []
    if phase in ('slope', 'all'):
        kinds.append(('slope', SLOPES))
    if phase in ('step', 'all'):
        kinds.append(('step', STEPS))
    for kind, values in kinds:
        for v in values:
            for name, ctl, pol in CONTROLLERS:
                # 목표 구간(15도, 20도)은 판정이 갈리는 곳이라 더 여러 번 잰다.
                n = 5 if (kind == 'slope' and v in (15, 20)) else reps
                for rep in range(1, n + 1):
                    yield kind, v, name, ctl, pol, rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--phase', default='slope', choices=['slope', 'step', 'all'])
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--speed', type=float, default=SPEED)
    a = ap.parse_args()
    os.makedirs(RESULTS, exist_ok=True)

    todo = list(cases(a.phase, a.reps))
    print('총 %d 회 (%s)' % (len(todo), a.phase), flush=True)
    t0 = time.time()
    done = 0
    for i, (kind, v, name, ctl, pol, rep) in enumerate(todo, 1):
        world = os.path.join(WORLDS, '%s%d.sdf' % (kind, v))
        tag = '%s%d_%s_r%d' % (kind, v, name, rep)
        out = os.path.join(RESULTS, tag + '.json')
        if a.resume and os.path.exists(out) and os.path.getsize(out) > 0:
            continue
        el = (time.time() - t0) / 60
        eta = (el / done * (len(todo) - i)) if done else 0
        print('[%3d/%3d] %-34s (경과 %.0f분, 남은 %.0f분)'
              % (i, len(todo), tag, el, eta), flush=True)
        subprocess.run([RUN, world, tag, kind, str(v), ctl, pol,
                        str(a.speed), out], cwd=ROOT,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        done += 1
        try:
            r = json.load(open(out))
            print('          -> %-8s 진출 %5.2f m  속도 %5.2f m/s  기울기 %5.1f'
                  % (r.get('verdict'), r.get('advance') or 0,
                     r.get('speed_actual') or 0, r.get('tilt_max') or 0),
                  flush=True)
        except Exception as e:
            print('          -> 읽기 실패 %s' % e, flush=True)
    print('완료. %.1f 분' % ((time.time() - t0) / 60), flush=True)


if __name__ == '__main__':
    main()
