"""주행성 판정 (모듈 F).

/terrain/features(slope, step)를 로봇별 통과 기준과 비교해 Nav2용 2D 주행 가능
맵(OccupancyGrid)을 만들고, 발행 직후 완료 상태(Bool)를 발행한다.
wheel·leg가 같은 노드를 파라미터(yaml)만 달리해서 각각 실행한다.
"""
import math

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from agconav_traversability.grid_map_util import layer_to_array, to_occupancy_grid

# 지도 토픽 QoS — README §3.4: reliable + transient_local(래치).
MAP_QOS = QoSProfile(depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

FREE = 0
OCCUPIED = 100


class TraversabilityVerdictor(Node):

    def __init__(self):
        super().__init__('traversability_verdictor')
        self.declare_parameter('features_topic', '/terrain/features')
        self.declare_parameter('output_map_topic', '/wheel/nav_map')
        self.declare_parameter('status_topic', '/wheel/nav_map_status')
        self.declare_parameter('max_slope_deg', 20.0)
        self.declare_parameter('max_step_m', 0.08)
        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('frame_id', 'map')

        self._max_slope_rad = math.radians(
            self.get_parameter('max_slope_deg').value)
        self._max_step = self.get_parameter('max_step_m').value
        self._unknown = self.get_parameter('unknown_value').value
        self._frame_id = self.get_parameter('frame_id').value
        self._done = False

        self._map_publisher = self.create_publisher(
            OccupancyGrid, self.get_parameter('output_map_topic').value,
            MAP_QOS)
        self._status_publisher = self.create_publisher(
            Bool, self.get_parameter('status_topic').value, MAP_QOS)
        self.create_subscription(
            GridMap, self.get_parameter('features_topic').value,
            self._on_features, MAP_QOS)

    def _on_features(self, message):
        if self._done:
            return
        slope = layer_to_array(message, 'slope')
        step = layer_to_array(message, 'step')

        # NaN과의 비교는 항상 False라, 미관측 셀은 unknown 값 그대로 남는다.
        known = np.isfinite(slope) & np.isfinite(step)
        passable = (slope <= self._max_slope_rad) & (step <= self._max_step)
        values = np.full(slope.shape, self._unknown, dtype=np.int8)
        values[known] = np.where(passable[known], FREE, OCCUPIED)

        header = message.header
        header.frame_id = self._frame_id
        self._map_publisher.publish(
            to_occupancy_grid(values, message.info, header))
        self._status_publisher.publish(Bool(data=True))

        self._done = True
        self.get_logger().info(
            f'주행 가능 맵 발행 완료 (통과 {int(np.count_nonzero(values == FREE))} / '
            f'불가 {int(np.count_nonzero(values == OCCUPIED))} 셀).')


def main():
    rclpy.init()
    node = TraversabilityVerdictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
