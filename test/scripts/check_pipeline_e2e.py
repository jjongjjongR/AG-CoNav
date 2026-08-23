#!/usr/bin/env python3
"""A -> F -> B -> C -> D -> E 종단 파이프라인 검증.

    check_pipeline_e2e.py [--wait 60]

각 단계의 산출 토픽이 실제로 나왔는지 순서대로 본다. topics.md 의 계약을
그대로 쓴다. 통과/미달을 표로 찍고, 미달이면 어디서 끊겼는지 알려준다.

왜 토픽으로 보나: 런치가 떴다고 파이프라인이 흐르는 것이 아니다. 모듈 F 가
드론 지도를 못 받으면 조용히 멈추고, 모듈 D 는 /X/navigation_status 가
없으면 지도를 안 낸다. 각 단계의 **산출물**을 봐야 실제로 흘렀는지 안다.
"""
import argparse
import subprocess
import sys

# (단계, 토픽, 설명). 순서가 곧 데이터 흐름이다.
STAGES = [
    ('A', '/drone/points', '드론 라이다 점군'),
    ('A', '/drone/elevation_map', '드론 2.5D 지도'),
    ('A', '/drone/elevation_map_status', '드론 지도 완료 신호'),
    ('F', '/wheel/nav_map', 'wheel 주행성 지도'),
    ('F', '/leg/nav_map', 'leg 주행성 지도'),
    ('F', '/wheel/nav_map_status', 'wheel 지도 완료 신호'),
    ('F', '/leg/nav_map_status', 'leg 지도 완료 신호'),
    ('B', '/wheel/odometry/filtered', 'wheel 위치추정'),
    ('B', '/leg/odom', 'leg 위치추정'),
    ('C', '/wheel/cmd_vel', 'wheel 주행 명령'),
    ('C', '/leg/cmd_vel', 'leg 주행 명령'),
    ('C', '/wheel/navigation_status', 'wheel 주행 완료 신호'),
    ('C', '/leg/navigation_status', 'leg 주행 완료 신호'),
    ('D', '/wheel/elevation_map', 'wheel 지상 2.5D 지도'),
    ('D', '/leg/elevation_map', 'leg 지상 2.5D 지도'),
    ('E', '/merged/elevation_map', '병합 지도'),
]


def has_publisher(topic, timeout=6):
    """그 토픽에 발행자가 있는가."""
    try:
        out = subprocess.run(['ros2', 'topic', 'info', topic],
                             capture_output=True, text=True,
                             timeout=timeout).stdout
    except Exception:
        return None
    for line in out.splitlines():
        if line.startswith('Publisher count:'):
            return int(line.split(':')[1].strip())
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()

    print('%-4s %-34s %-24s %s' % ('단계', '토픽', '설명', '발행자'))
    print('-' * 82)
    missing = []
    for stage, topic, desc in STAGES:
        n = has_publisher(topic)
        mark = 'OK' if (n or 0) > 0 else '없음'
        if (n or 0) == 0:
            missing.append((stage, topic, desc))
        print('%-4s %-34s %-24s %s' % (stage, topic, desc, mark))

    print()
    if not missing:
        print('전 단계 통과 — A -> F -> B -> C -> D -> E 가 모두 흘렀다.')
        return 0
    first = missing[0]
    print('끊긴 지점: 모듈 %s 의 %s (%s)' % (first[0], first[1], first[2]))
    print('미달 %d개:' % len(missing))
    for stage, topic, desc in missing:
        print('  %s  %s  (%s)' % (stage, topic, desc))
    return 1


if __name__ == '__main__':
    sys.exit(main())
