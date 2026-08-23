#!/usr/bin/env python3
"""축소 월드 스캔 박스에 주어진 줄 간격으로 잔디깎기 경로를 만든다.

    make_strip_path.py <간격[m]> <출력.yaml> [x0 x1 y0 y1]

박스를 생략하면 축소 월드(Seongdong_gu_100x100). 본 맵 4줄 구간을 쓰려면
    make_strip_path.py 16.7 out.yaml -347.56 370.34 -331.07 -236.0

박스는 Seongdong_gu_100x100 의 범위(generate_test_world.py 가 쓴 값)이고,
고도는 팀 확정값 84.0 m 다. 줄 간격만 바꾸고 나머지는 그대로 둔다.
"""
import sys
import yaml

BOX = (-38.6, 61.4, -166.1, -66.1)      # x0, x1, y0, y1
ALT = 84.0
Q0 = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}                    # +x 방향
Q1 = {'x': 0.0, 'y': 0.0, 'z': 1.0, 'w': 6.123233995736766e-17}  # -x 방향


def main(spacing, out):
    x0, x1, y0, y1 = BOX
    ys, y = [], y0
    while y <= y1 + 1e-6:
        ys.append(round(y, 4))
        y += spacing
    # 마지막 줄이 박스 끝을 못 덮으면 하나 더 넣는다. 단 남은 틈이 간격의
    # 절반도 안 되면 넣지 않는다 — 0.1 m 간격의 중복 줄이 생긴다.
    if y1 - ys[-1] > spacing * 0.5:
        ys.append(y1)

    wps = []
    for i, yy in enumerate(ys):
        a, b = (x0, x1) if i % 2 == 0 else (x1, x0)
        q = Q0 if i % 2 == 0 else Q1
        wps.append({'position': {'x': a, 'y': yy, 'z': ALT}, 'orientation': dict(q)})
        wps.append({'position': {'x': b, 'y': yy, 'z': ALT}, 'orientation': dict(q)})

    with open(out, 'w') as f:
        f.write('# 줄 간격 %.2f m 로 생성 (스트립 간격 실험용).\n' % spacing)
        f.write('# 박스 x[%.1f, %.1f] y[%.1f, %.1f], 고도 %.1f m.\n' % (x0, x1, y0, y1, ALT))
        yaml.safe_dump({'frame_id': 'map', 'altitude': ALT,
                        'strip_spacing': float(spacing),
                        'num_waypoints': len(wps), 'waypoints': wps},
                       f, default_flow_style=False, sort_keys=False)
    length = len(ys) * (x1 - x0) + (len(ys) - 1) * spacing
    print('%s: 줄 %d개, waypoint %d개, 경로 %.0f m' % (out, len(ys), len(wps), length))


if __name__ == '__main__':
    if len(sys.argv) > 3:
        BOX = tuple(float(v) for v in sys.argv[3:7])
    main(float(sys.argv[1]), sys.argv[2])
