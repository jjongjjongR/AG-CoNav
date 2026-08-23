#!/usr/bin/env python3
"""저장된 지도 산출물을 축소해서 RViz로 보기 위한 **시각화 전용** 도구.

파이프라인은 건드리지 않는다.
읽기: maps/ 아래에 이미 저장된 파일만 읽는다.
쓰기: 전부 `/viz/...` 접두어 토픽으로만 내보낸다.
      계약 토픽(/drone/elevation_map, /X/nav_map 등)에는 아무것도 발행하지 않는다.

왜 축소해야 하나:
이 시나리오의 지도는 0.10 m/cell에 783 x 731 m라 약 5,010만 셀이다.
RViz는 이 크기를 감당하지 못한다 — 두 방식 모두 실측으로 확인했다.

  Map 표시(OccupancyGrid)  : 텍스처를 만들다 프로세스가 그대로 종료된다
      [rviz2]: Trying to create a map of size 7398 x 6741 using 1 swatches
  GridMap 표시             : 22.6 GB를 점유하고 렌더 스레드가 100%로 붙잡혀
                             화면이 아예 안 움직인다(멈춘 게 아니라 계산 중)

4배로 줄이면 313만 셀(0.40 m/cell)이 되어 무리 없이 보인다.

사용법:
    ./scripts/view_maps.py                 # 4배 축소, 있는 지도 전부
    ./scripts/view_maps.py --factor 8
    ./scripts/view_maps.py --only drone merged
"""
import argparse
import pathlib
import sys
import time
import warnings

import numpy as np
import rclpy
import yaml
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

MAPS = pathlib.Path('/home/lee/projects/AG-CoNav/maps')

LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# (이름, 종류, 경로, 발행할 시각화 토픽)
SOURCES = [
    ('drone',  'gridmap', 'drone_elevation_map',   '/viz/drone/elevation_map'),
    ('wheel',  'gridmap', 'wheel_elevation_map',   '/viz/wheel/elevation_map'),
    ('leg',    'gridmap', 'leg_elevation_map',     '/viz/leg/elevation_map'),
    ('merged', 'gridmap', 'merged_elevation_map',  '/viz/merged/elevation_map'),
    ('wheel_nav', 'occgrid', 'wheel_nav_map.yaml', '/viz/wheel/nav_map'),
    ('leg_nav',   'occgrid', 'leg_nav_map.yaml',   '/viz/leg/nav_map'),
]


def block_reduce_mean(a, n):
    """NaN을 무시하고 n x n 블록 평균. 블록이 전부 비었으면 NaN을 남긴다."""
    h, w = a.shape
    a = a[:h // n * n, :w // n * n]
    a = a.reshape(h // n, n, w // n, n)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # all-NaN 블록 경고
        return np.nanmean(a, axis=(1, 3))


def block_reduce_occupancy(a, n):
    """점유 우선으로 축소한다. 안전한 쪽으로 보수적이다.
    블록에 점유(100)가 하나라도 있으면 100, 없고 자유(0)가 있으면 0, 아니면 미지(-1).
    """
    h, w = a.shape
    a = a[:h // n * n, :w // n * n].reshape(h // n, n, w // n, n)
    occ = (a == 100).any(axis=(1, 3))
    free = (a == 0).any(axis=(1, 3))
    out = np.full(occ.shape, -1, dtype=np.int8)
    out[free] = 0
    out[occ] = 100
    return out


def read_gridmap(path):
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    while r.has_next():
        _topic, data, _t = r.read_next()
        msg = deserialize_message(data, GridMap)
        if msg.layers:
            return msg
    return None


def shrink_gridmap(gm, n):
    """GridMap을 n배 축소한다. 레이어 배열의 layout 구조는 그대로 유지한다."""
    out = GridMap()
    out.header = gm.header
    out.layers = list(gm.layers)
    out.basic_layers = list(gm.basic_layers)
    out.outer_start_index = 0
    out.inner_start_index = 0

    for lay in gm.data:
        d0, d1 = lay.layout.dim[0].size, lay.layout.dim[1].size
        a = np.array(lay.data, dtype=np.float32).reshape(d0, d1)
        s = block_reduce_mean(a, n)
        m = Float32MultiArray()
        m.layout.dim = [
            MultiArrayDimension(label=lay.layout.dim[0].label,
                                size=s.shape[0], stride=s.size),
            MultiArrayDimension(label=lay.layout.dim[1].label,
                                size=s.shape[1], stride=s.shape[1]),
        ]
        m.data = s.reshape(-1).tolist()
        out.data.append(m)
        rows, cols = s.shape

    out.info = gm.info
    out.info.resolution = gm.info.resolution * n
    # 셀 수가 잘린 만큼 실제 길이도 다시 계산한다.
    out.info.length_x = cols * out.info.resolution
    out.info.length_y = rows * out.info.resolution
    return out, rows * cols


def read_occgrid(yaml_path, n):
    meta = yaml.safe_load(open(yaml_path))
    base = yaml_path.parent
    with open(base / meta['image'], 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = (int(v) for v in line.split())
        int(f.readline())
        pgm = np.frombuffer(f.read(width * height),
                            dtype=np.uint8).reshape(height, width)
    p = pgm.astype(np.float32) / 255.0
    occ = p if meta.get('negate', 0) else (1.0 - p)
    full = np.full(pgm.shape, -1, dtype=np.int8)
    full[occ > float(meta['occupied_thresh'])] = 100
    full[occ < float(meta['free_thresh'])] = 0
    small = block_reduce_occupancy(np.flipud(full), n)

    g = OccupancyGrid()
    g.header.frame_id = 'map'
    g.info.resolution = float(meta['resolution']) * n
    g.info.height, g.info.width = small.shape
    ox, oy, _ = meta['origin']
    g.info.origin.position.x = float(ox)
    g.info.origin.position.y = float(oy)
    g.info.origin.orientation.w = 1.0
    g.data = small.reshape(-1).tolist()
    return g, small.size


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--factor', type=int, default=4, help='축소 배수 (기본 4)')
    ap.add_argument('--only', nargs='*', help='이 이름들만 (drone wheel leg merged ...)')
    args = ap.parse_args()

    rclpy.init()
    node = Node('map_viewer')
    pubs = []

    for name, kind, fname, topic in SOURCES:
        if args.only and name not in args.only:
            continue
        path = MAPS / fname
        if not path.exists():
            node.get_logger().warn(f'{name}: 없음 ({path})')
            continue
        t0 = time.monotonic()
        try:
            if kind == 'gridmap':
                gm = read_gridmap(path)
                if gm is None:
                    node.get_logger().warn(f'{name}: 레이어 없음')
                    continue
                msg, cells = shrink_gridmap(gm, args.factor)
                pub = node.create_publisher(GridMap, topic, LATCHED)
            else:
                msg, cells = read_occgrid(path, args.factor)
                pub = node.create_publisher(OccupancyGrid, topic, LATCHED)
        except Exception as exc:                                  # noqa: BLE001
            node.get_logger().error(f'{name}: 실패 {exc}')
            continue
        pub.publish(msg)
        pubs.append(pub)
        node.get_logger().info(
            f'{topic}  {cells/1e6:.2f}M 셀 '
            f'({args.factor}배 축소, {time.monotonic()-t0:.1f}s)')

    if not pubs:
        node.get_logger().error('발행할 지도가 없습니다.')
        return 1
    node.get_logger().info('래치 발행 유지 중 (Ctrl-C로 종료)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
