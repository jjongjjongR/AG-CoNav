#!/usr/bin/env python3
"""GLIM+GPS 실험 전용: /drone/points(원본, per-point 't' 필드 없음)를 구독해
add_point_times.py와 동일한 로직으로 't' 필드를 추가한 뒤 /drone/points_t로
재발행한다.

디스크에 t필드 추가된 새 bag을 통째로 만들면(원본 22GB -> 신규 ~25GB+) 이
VM(78GB, 여유 23GB)에서 두 bag이 동시에 존재할 공간이 부족하다(실측:
디스크 88%까지 차오르는 것 확인 후 중단). 대신 `ros2 bag play`로 원본을
재생하면서 이 노드가 실시간으로 t필드를 얹어 재발행하고, GLIM은
glim_rosnode(라이브 구독)로 그 토픽을 바로 소비한다 -- 디스크에 중간
산출물을 전혀 안 만든다.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

SCAN_PERIOD = 0.1  # 10 Hz


def add_times(m: PointCloud2) -> PointCloud2:
    raw = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, m.point_step)
    col = (np.arange(m.width, dtype=np.float32) / m.width * SCAN_PERIOD)
    tcol = np.repeat(col[None, :], m.height, axis=0).copy().view(np.uint8)
    out = np.concatenate([raw, tcol.reshape(m.height, m.width, 4)], axis=2)

    new = PointCloud2()
    new.header = m.header
    new.height, new.width = m.height, m.width
    new.fields = list(m.fields) + [PointField(
        name='t', offset=m.point_step, datatype=PointField.FLOAT32, count=1)]
    new.is_bigendian = m.is_bigendian
    new.point_step = m.point_step + 4
    new.row_step = new.point_step * m.width
    new.data = out.tobytes()
    new.is_dense = m.is_dense
    return new


class Republisher(Node):
    def __init__(self):
        super().__init__('points_t_republisher')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(PointCloud2, '/drone/points_t', qos)
        self.sub = self.create_subscription(PointCloud2, '/drone/points', self._cb, qos)
        self.n = 0
        self.n_dropped = 0
        self._last_stamp = None  # (sec, nanosec) 튜플 -- 단조증가 가드
        self.get_logger().info("points_t_republisher 시작: /drone/points -> /drone/points_t")

    def _cb(self, msg):
        # 원인 미상의 재정렬(GLIM이 "point/imu timestamp rewind" 경고를 내며
        # 스캔을 skip하는 현상 실측 -- run_results/PROGRESS.md 참고)을 근본
        # 원인 규명 없이 안전하게 막는다: header.stamp가 직전에 재발행한
        # 것보다 뒤로 가면(같거나 작으면) 이 메시지는 버린다. GLIM에는
        # 항상 단조증가하는 시각만 전달되도록 보장.
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self._last_stamp is not None and stamp <= self._last_stamp:
            self.n_dropped += 1
            if self.n_dropped % 100 == 1:
                self.get_logger().warn(
                    f'비단조 stamp 감지, 버림(누적 {self.n_dropped}건): '
                    f'{stamp} <= {self._last_stamp}')
            return
        self._last_stamp = stamp
        self.pub.publish(add_times(msg))
        self.n += 1
        if self.n % 500 == 0:
            self.get_logger().info(f'{self.n}개 재발행 (버림 {self.n_dropped}건)')


def main():
    rclpy.init()
    node = Republisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info(f'종료 -- 총 {node.n}개 재발행')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
