#!/usr/bin/env python3
"""주행성 지도가 실제로 **주행 가능한지** 연결성으로 따진다.

    analyze_nav_map.py <bag> [--topic /wheel/nav_map] [--start x y] [--goal x y]

왜 필요한가: 2단계 종단 테스트에서 Nav2 가 계획에 실패했다.

    GridBased plugin failed to plan from (-194.14, 71.67) to (-14.19, 9.67):
      "Failed to create plan with tolerance of: 0.500000"

FN 비율만 봐서는 이걸 예측할 수 없다. 통과가능 셀이 78.9% 나 되어도, 막힌
셀이 얼룩으로 흩어져 있으면 로봇 폭만 한 통로가 전부 끊긴다. 그래서
"몇 %가 통과가능한가"가 아니라 **"출발점에서 목적지까지 폭 W 의 길이 이어져
있는가"** 를 직접 잰다.

세 단계로 본다.
  1) 원본 격자에서 자유 공간(값 0)의 연결 성분
  2) 로봇 반지름만큼 침식한 뒤의 연결 성분 -- 실제로 로봇이 지나갈 수 있는 폭
  3) Nav2 inflation_radius(0.55 m)까지 반영한 뒤

침식은 사각형이 아니라 **원판**으로 한다. 사각형은 대각선 방향을 과하게 깎아
통과 가능 면적을 실제보다 낮게 만든다(이전 실험에서 v8 조합을 37% vs 76% 로
잘못 평가한 적이 있다).
"""
import argparse

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from scipy import ndimage

# A300(wheel): 폭 0.676 m (트랙 0.562 + 바퀴폭 0.1143), 외접 지름 1.094 m.
ROBOT = {'wheel': 1.094 / 2.0, 'leg': 0.40}
NAV2_INFLATION = 0.55


def load(bag, topic):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    last = None
    while r.has_next():
        t, data, _ = r.read_next()
        if t == topic:
            last = deserialize_message(data, OccupancyGrid)
    if last is None:
        raise SystemExit('%s 가 bag 에 없다' % topic)
    return last


def disk(radius_cells):
    r = int(np.ceil(radius_cells))
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return (x * x + y * y) <= radius_cells ** 2


def rc(msg, x, y):
    res = msg.info.resolution
    c = int((x - msg.info.origin.position.x) / res)
    r = int((y - msg.info.origin.position.y) / res)
    return r, c


def report(free, msg, start, goal, label):
    lab, n = ndimage.label(free)
    if n == 0:
        print('  %-22s 자유 공간이 남지 않았다' % label)
        return
    sizes = np.bincount(lab.ravel())
    sizes[0] = 0
    big = sizes.argmax()
    sr, sc = rc(msg, *start)
    gr, gc = rc(msg, *goal)
    ok = lambda r, c: 0 <= r < free.shape[0] and 0 <= c < free.shape[1]
    ls = lab[sr, sc] if ok(sr, sc) else 0
    lg = lab[gr, gc] if ok(gr, gc) else 0
    conn = (ls != 0 and ls == lg)
    print('  %-22s 자유 %5.1f%%  덩어리 %6d개  최대덩어리 %5.1f%%  '
          '출발%s 목적지%s  연결 %s'
          % (label, 100.0 * free.mean(), n, 100.0 * sizes[big] / free.size,
             '자유' if ls else '막힘', '자유' if lg else '막힘',
             'O' if conn else 'X'))
    return conn


def main():
    p = argparse.ArgumentParser()
    p.add_argument('bag')
    p.add_argument('--topic', default='/wheel/nav_map')
    p.add_argument('--robot', default='wheel')
    p.add_argument('--start', nargs=2, type=float, default=[-194.14, 71.67])
    p.add_argument('--goal', nargs=2, type=float, default=[-14.19, 9.67])
    a = p.parse_args()

    msg = load(a.bag, a.topic)
    res = msg.info.resolution
    g = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    print('%s: %d x %d 셀, %.2f m/셀, 원점 (%.2f, %.2f)'
          % (a.topic, msg.info.width, msg.info.height, res,
             msg.info.origin.position.x, msg.info.origin.position.y))
    print('  미지 %.2f%%  통과가능 %.2f%%  막힘 %.2f%%'
          % (100.0 * (g < 0).mean(), 100.0 * (g == 0).mean(), 100.0 * (g > 0).mean()))

    # 막힌 셀 덩어리 크기 -- 얼룩인지 진짜 장애물인지 구분한다.
    blk, nb = ndimage.label(g > 0)
    if nb:
        bs = np.bincount(blk.ravel())[1:]
        print('  막힌 덩어리 %d개, 중앙값 %.0f셀, 평균 %.1f셀, 최대 %d셀 '
              '(1~2셀짜리가 %.1f%%)'
              % (nb, np.median(bs), bs.mean(), bs.max(), 100.0 * (bs <= 2).mean()))

    free0 = (g == 0)
    print('\n연결성 (출발 %.2f,%.2f -> 목적지 %.2f,%.2f)'
          % (*a.start, *a.goal))
    report(free0, msg, a.start, a.goal, '원본')

    # 미지 셀을 통과 허용했을 때 (Nav2 allow_unknown:true 에 해당)
    report((g <= 0), msg, a.start, a.goal, '미지 통과 허용')

    for name, rad in (('로봇 반지름 %.2fm' % ROBOT[a.robot], ROBOT[a.robot]),
                      ('Nav2 팽창 %.2fm' % NAV2_INFLATION, NAV2_INFLATION)):
        er = ndimage.binary_erosion(free0, structure=disk(rad / res))
        report(er, msg, a.start, a.goal, name)


if __name__ == '__main__':
    main()
