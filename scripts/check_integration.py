#!/usr/bin/env python3
"""모듈 A~F 연결 정합성 점검.

각 모듈 간 인터페이스 토픽에 **발행자와 구독자가 모두 붙어 있는지**를 본다.
한쪽만 있으면 이름이 어긋난 것이고, 그 지점에서 파이프라인이 끊긴다.

사용법:
    ./scripts/check_integration.py
    ./scripts/check_integration.py --quiet
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node

# (토픽, 발행 모듈, 구독 모듈, 필수여부)
# 필수=False 는 상류 조건이 갖춰져야 흐르는 것(1회성 트리거 등).
LINKS = [
    # 시뮬 -> 모듈
    ('/drone/points',                 '시뮬',   'A',      True),
    ('/wheel/points',                 '시뮬',   'C,D',    True),
    ('/leg/points',                   '시뮬',   'C,D',    True),
    # A -> F, E
    ('/drone/elevation_map',          'A',      'E,F',    False),
    ('/drone/elevation_map_status',   'A',      'F',      False),
    # F -> C
    ('/wheel/nav_map',                'F',      'C',      False),
    ('/leg/nav_map',                  'F',      'C',      False),
    # C -> D
    ('/wheel/navigation_status',      'C',      'D',      False),
    ('/leg/navigation_status',        'C',      'D',      False),
    # D -> E
    ('/wheel/elevation_map',          'D',      'E',      False),
    ('/leg/elevation_map',            'D',      'E',      False),
    ('/wheel/elevation_map_status',   'D',      'E',      False),
    ('/leg/elevation_map_status',     'D',      'E',      False),
    # E 내부/출력
    ('/merged/merge_trigger',         'E',      'E',      False),
    ('/merged/elevation_map',         'E',      'E',      False),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--discovery', type=float, default=20.0,
                    help='그래프 탐색 대기 시간(초)')
    args = ap.parse_args()

    rclpy.init()
    node = Node('integration_checker')
    # ROS 2 그래프 탐색은 노드가 spin해야 갱신되고, 노드가 100개를 넘거나
    # 머신 부하가 높으면 수십 초가 걸린다. 짧게 주면 실제로는 떠 있는데도
    # count_publishers()가 0을 돌려주는 오탐이 난다(실측: 6초는 부족, 15초면 안정).
    end = time.monotonic() + args.discovery
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    rows = []
    for topic, src, dst, required in LINKS:
        pubs = node.count_publishers(topic)
        subs = node.count_subscribers(topic)
        if pubs > 0 and subs > 0:
            state, ok = '연결됨', True
        elif pubs == 0 and subs == 0:
            state, ok = '양쪽 없음', not required
        elif pubs == 0:
            state, ok = '발행자 없음', False        # 이름 불일치 의심
        else:
            state, ok = '구독자 없음', not required
        rows.append((ok, topic, src, dst, pubs, subs, state, required))

    node.destroy_node()
    rclpy.shutdown()

    bad = [r for r in rows if not r[0]]
    if not args.quiet:
        print('=' * 92)
        print('모듈 간 연결 점검  (발행자/구독자가 양쪽 다 있어야 파이프라인이 이어짐)')
        print('=' * 92)
        print(f'{"":5s} {"토픽":34s} {"흐름":12s} {"pub":>4s} {"sub":>4s}  상태')
        for ok, topic, src, dst, pubs, subs, state, required in rows:
            mark = 'OK  ' if ok else 'FAIL'
            print(f'{mark:5s} {topic:34s} {src+" -> "+dst:12s} '
                  f'{pubs:4d} {subs:4d}  {state}')
        print('=' * 92)
    if bad:
        for _, topic, src, dst, pubs, subs, state, _r in bad:
            if args.quiet:
                print(f'FAIL | {topic} ({src} -> {dst}): {state}')
        print(f'끊긴 연결 {len(bad)} / 전체 {len(rows)}')
        return 1
    print(f'전체 {len(rows)}개 연결 정상')
    return 0


if __name__ == '__main__':
    sys.exit(main())
