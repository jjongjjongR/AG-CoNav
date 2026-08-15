#!/usr/bin/env python3
"""Publish a map-frame cloud as /drone/points and trigger module A.

Lets any accumulated cloud -- GT-registered, GLIM's, anything -- be pushed
through the unmodified module A -> module F chain, so traversability is computed
by the real nodes rather than reimplemented here.

Points go out already in `map`, so module A's TF lookup is map->map and resolves
to identity with no broadcaster running. When the cloud is exhausted a latched
/drone/path_status=True fires, which is what makes module A publish its map.

    python3 feed_cloud.py /tmp/cloud.npy
"""

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header

CHUNK = 200_000


class Feeder(Node):
    def __init__(self, pts):
        super().__init__('feed_cloud')
        # copy=False: 입력이 이미 float32 mmap이면 그대로 두고, 슬라이스 시점에만
        # 페이지인되게 한다 (astype 기본 copy=True는 여기서 전체를 즉시 RAM에 복사해버려
        # mmap_mode='r'로 연 의미가 없어짐).
        self.pts = pts.astype(np.float32, copy=False)
        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=5)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(PointCloud2, '/drone/points', sensor)
        self.status = self.create_publisher(Bool, '/drone/path_status', latched)
        self.n = 0
        self.get_logger().info('점 %d개 발행 시작' % len(self.pts))
        self.timer = self.create_timer(0.5, self.tick)

    def msg_for(self, chunk):
        m = PointCloud2()
        m.header = Header()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.height, m.width = 1, len(chunk)
        m.fields = [PointField(name=n, offset=4 * i,
                               datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(('x', 'y', 'z'))]
        m.is_bigendian = False
        m.point_step = 12
        m.row_step = 12 * len(chunk)
        m.data = chunk.tobytes()
        m.is_dense = True
        return m

    def tick(self):
        if self.n * CHUNK < len(self.pts):
            self.pub.publish(self.msg_for(self.pts[self.n * CHUNK:(self.n + 1) * CHUNK]))
            self.n += 1
            if self.n % 5 == 0:
                self.get_logger().info('  %d / %d 점'
                                       % (min(self.n * CHUNK, len(self.pts)), len(self.pts)))
        else:
            self.get_logger().info('전송 완료 — path_status=True')
            self.status.publish(Bool(data=True))
            self.timer.cancel()
            self.create_timer(8.0, lambda: rclpy.shutdown())


def main():
    # mmap_mode='r': 이 VM은 5.8GB RAM뿐이라, 큰 비행(예: 5m AGL 4m/5mps, 점
    # ~수억 개)의 cloud를 np.load로 통째로 올리면 OOM 위험이 있다(방법B cloud
    # 생성 스크립트에서 실제로 OOM-kill 관측, run_results/PROGRESS.md 참조).
    # 메모리매핑으로 열면 self.pts[a:b] 슬라이스만 그때그때 페이지인된다.
    arr = np.load(sys.argv[1], mmap_mode='r')[:, :3]
    print('%s: %d점  X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]'
          % (sys.argv[1], len(arr), arr[:, 0].min(), arr[:, 0].max(),
             arr[:, 1].min(), arr[:, 1].max(), arr[:, 2].min(), arr[:, 2].max()))
    rclpy.init()
    rclpy.spin(Feeder(arr))


if __name__ == '__main__':
    main()
