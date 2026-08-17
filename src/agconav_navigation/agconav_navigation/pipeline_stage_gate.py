#!/usr/bin/env python3
"""Exit successfully after both Module F navigation maps are ready."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class PipelineStageGate(Node):
    def __init__(self):
        super().__init__('pipeline_stage_gate')
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._ready = {'wheel': False, 'leg': False}
        self._done = False
        for robot in self._ready:
            self.create_subscription(
                Bool, f'/{robot}/nav_map_status',
                lambda msg, name=robot: self._on_status(name, msg), qos)
        self.get_logger().info(
            'A→F 완료 대기: wheel/leg 주행 가능 지도 상태를 기다립니다')

    @property
    def done(self):
        return self._done

    def _on_status(self, robot, message):
        if not message.data:
            self.get_logger().error(f'Module F {robot} 지도 생성 실패 상태 수신')
            return
        if not self._ready[robot]:
            self._ready[robot] = True
            self.get_logger().info(f'Module F {robot} 지도 준비 완료')
        if all(self._ready.values()):
            self._done = True
            self.get_logger().info('A→F 완료: 후속 B→C→D→E 시작 허용')


def main(args=None):
    rclpy.init(args=args)
    node = PipelineStageGate()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        success = node.done
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
