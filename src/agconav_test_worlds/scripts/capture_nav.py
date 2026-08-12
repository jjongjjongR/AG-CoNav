#!/usr/bin/env python3
"""Report what module A and module F produced, as numbers.

Subscribes with the same latched QoS the modules publish on, waits for the one
elevation map, the feature grid and both nav maps, prints the figures the
comparison table needs, and exits.

    python3 capture_nav.py <태그>
"""

import sys

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

WANT = {'/wheel/nav_map', '/leg/nav_map', 'elev', 'feat'}
LIMITS = {'slope': (20.0, 30.0), 'step': (0.08, 0.15)}


class Capture(Node):
    def __init__(self, tag, timeout):
        super().__init__('capture_nav')
        self.tag = tag
        self.seen = set()
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        for t in ('/wheel/nav_map', '/leg/nav_map'):
            self.create_subscription(OccupancyGrid, t,
                                     lambda m, t=t: self.on_grid(t, m), q)
        self.create_subscription(GridMap, '/drone/elevation_map', self.on_elev, q)
        self.create_subscription(GridMap, '/terrain/features', self.on_feat, q)
        self.create_timer(timeout, self.finish)

    def on_elev(self, m):
        if 'elev' in self.seen:
            return
        self.seen.add('elev')
        d = np.array(m.data[m.layers.index('elevation')].data, dtype=np.float64)
        ok = np.isfinite(d)
        print('[%s] elevation_map  %.0f x %.0f m @ %.3f m  셀 %d'
              % (self.tag, m.info.length_x, m.info.length_y, m.info.resolution, d.size))
        if ok.any():
            print('[%s]   유효 %d (%.1f%%)  고도 %.2f ~ %.2f m'
                  % (self.tag, ok.sum(), 100 * ok.mean(), d[ok].min(), d[ok].max()))
        self.maybe_done()

    def on_feat(self, m):
        if 'feat' in self.seen:
            return
        self.seen.add('feat')
        for name, (wheel, leg) in LIMITS.items():
            if name not in m.layers:
                continue
            d = np.array(m.data[m.layers.index(name)].data, dtype=np.float64)
            v = d[np.isfinite(d)]
            if not v.size:
                continue
            print('[%s] %-5s 중앙값 %.4f  95%% %.4f  최대 %.4f  |  wheel(%.2f) 초과 %.1f%%'
                  '   leg(%.2f) 초과 %.1f%%'
                  % (self.tag, name, np.median(v), np.percentile(v, 95), v.max(),
                     wheel, 100 * (v > wheel).mean(), leg, 100 * (v > leg).mean()))
        self.maybe_done()

    def on_grid(self, topic, m):
        if topic in self.seen:
            return
        self.seen.add(topic)
        d = np.array(m.data, dtype=np.int16)
        unknown, free, blocked = (d < 0).sum(), (d == 0).sum(), (d >= 100).sum()
        known = free + blocked
        print('[%s] %-14s %d x %d @ %.2f m  |  미지 %5.1f%%  주행가능 %5.1f%%  막힘 %5.1f%%'
              '   (알려진 셀 중 주행가능 %s)'
              % (self.tag, topic, m.info.width, m.info.height, m.info.resolution,
                 100 * unknown / d.size, 100 * free / d.size, 100 * blocked / d.size,
                 ('%.1f%%' % (100 * free / known)) if known else '알려진 셀 없음'))
        self.maybe_done()

    def maybe_done(self):
        if WANT <= self.seen:
            self.finish()

    def finish(self):
        missing = WANT - self.seen
        if missing:
            print('[%s] 수신 실패: %s' % (self.tag, ', '.join(sorted(missing))))
        rclpy.shutdown()


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else '?'
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
    rclpy.init()
    try:
        rclpy.spin(Capture(tag, timeout))
    except Exception:
        pass


if __name__ == '__main__':
    main()
