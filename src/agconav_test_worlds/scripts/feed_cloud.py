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
        self.pts = pts.astype(np.float32)
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
    arr = np.load(sys.argv[1])[:, :3]
    print('%s: %d점  X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]'
          % (sys.argv[1], len(arr), arr[:, 0].min(), arr[:, 0].max(),
             arr[:, 1].min(), arr[:, 1].max(), arr[:, 2].min(), arr[:, 2].max()))
    rclpy.init()
    rclpy.spin(Feeder(arr))


if __name__ == '__main__':
    main()
