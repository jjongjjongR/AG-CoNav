#!/usr/bin/env python3
"""bag 안의 주행성 지도를 Nav2 map_server 가 읽는 pgm + yaml 로 뽑는다.

    export_nav_map.py <bag> <출력 디렉터리> [--topics /wheel/nav_map /leg/nav_map]

왜 필요한가: 스캔 결과(전체 맵 v6x3 주행성 지도)를 저장소에 남겨야 하는데,
원본 bag 은 2.3 GB 라 커밋할 수 없다. 지도 자체는 5786 x 4855 이진 격자라
pgm 으로 내보내면 압축이 잘 되고, Nav2 의 map_server 로 그대로 다시 띄울 수 있다.

값 대응 (ROS map_server 규약):
    통과 가능(0)   -> 254 (흰색, free)
    막힘(100)      ->   0 (검정, occupied)
    미지(-1)       -> 205 (회색, unknown)
"""
import argparse
import os

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from PIL import Image


def main():
    p = argparse.ArgumentParser()
    p.add_argument('bag')
    p.add_argument('outdir')
    p.add_argument('--topics', nargs='+',
                   default=['/wheel/nav_map', '/leg/nav_map'])
    a = p.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    r = SequentialReader()
    r.open(StorageOptions(uri=a.bag, storage_id='mcap'), ConverterOptions('', ''))
    want = set(a.topics)
    found = {}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic in want:
            found[topic] = deserialize_message(data, OccupancyGrid)

    for topic, msg in found.items():
        name = topic.strip('/').replace('/', '_')
        g = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        img = np.full(g.shape, 205, dtype=np.uint8)   # 미지
        img[g == 0] = 254                             # 통과 가능
        img[g > 0] = 0                                # 막힘
        # pgm 은 좌상단이 원점이라 y 를 뒤집는다 (OccupancyGrid 는 좌하단 기준).
        Image.fromarray(np.flipud(img)).save(os.path.join(a.outdir, name + '.pgm'))
        with open(os.path.join(a.outdir, name + '.yaml'), 'w') as f:
            f.write('image: %s.pgm\n' % name)
            f.write('mode: trinary\n')
            f.write('resolution: %.6f\n' % msg.info.resolution)
            f.write('origin: [%.6f, %.6f, 0.0]\n'
                    % (msg.info.origin.position.x, msg.info.origin.position.y))
            f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        sz = os.path.getsize(os.path.join(a.outdir, name + '.pgm'))
        print('%s -> %s.pgm  %d x %d  %.1f MB  (통과 %.1f%% / 막힘 %.1f%% / 미지 %.1f%%)'
              % (topic, name, msg.info.width, msg.info.height, sz / 1e6,
                 100.0 * (g == 0).mean(), 100.0 * (g > 0).mean(), 100.0 * (g < 0).mean()))


if __name__ == '__main__':
    main()
