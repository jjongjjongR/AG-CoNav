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
        self.declare_parameter('preview_map_topic', '/wheel/nav_map_preview')
        self.declare_parameter('preview_factor', 8)
        self.declare_parameter('status_topic', '/wheel/nav_map_status')
        self.declare_parameter('max_slope_deg', 20.0)
        self.declare_parameter('max_step_m', 0.08)
        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('frame_id', 'map')
        # When ground robots are present during the drone survey, their own
        # bodies are measured as terrain obstacles.  Clear only the known
        # spawn footprint (robot radius + Nav2 inflation margin) so a robot
        # is not trapped inside an obstacle made from itself.
        self.declare_parameter('spawn_clear_x', 0.0)
        self.declare_parameter('spawn_clear_y', 0.0)
        self.declare_parameter('spawn_clear_radius_m', 0.0)

        self._max_slope_rad = math.radians(
            self.get_parameter('max_slope_deg').value)
        self._max_step = self.get_parameter('max_step_m').value
        self._unknown = self.get_parameter('unknown_value').value
        self._frame_id = self.get_parameter('frame_id').value
        self._done = False

        self._map_publisher = self.create_publisher(
            OccupancyGrid, self.get_parameter('output_map_topic').value,
            MAP_QOS)
        self._preview_publisher = self.create_publisher(
            OccupancyGrid, self.get_parameter('preview_map_topic').value,
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

        clear_radius = float(
            self.get_parameter('spawn_clear_radius_m').value)
        if clear_radius > 0.0:
            clear_x = float(self.get_parameter('spawn_clear_x').value)
            clear_y = float(self.get_parameter('spawn_clear_y').value)
            resolution = float(message.info.resolution)
            max_x = message.info.pose.position.x + message.info.length_x / 2.0
            max_y = message.info.pose.position.y + message.info.length_y / 2.0
            center_i = int(round((max_x - clear_x) / resolution))
            center_j = int(round((max_y - clear_y) / resolution))
            radius_cells = int(math.ceil(clear_radius / resolution))
            i0, i1 = max(0, center_i - radius_cells), min(
                values.shape[0], center_i + radius_cells + 1)
            j0, j1 = max(0, center_j - radius_cells), min(
                values.shape[1], center_j + radius_cells + 1)
            ii, jj = np.ogrid[i0:i1, j0:j1]
            disk = ((ii - center_i) ** 2 + (jj - center_j) ** 2
                    <= (clear_radius / resolution) ** 2)
            cleared = int(np.count_nonzero(
                values[i0:i1, j0:j1][disk] != FREE))
            values[i0:i1, j0:j1][disk] = FREE
            self.get_logger().info(
                f'스폰 위치 자기 형상 제거: ({clear_x:.3f}, {clear_y:.3f}), '
                f'반경 {clear_radius:.2f}m, {cleared}셀 정리')

        header = message.header
        header.frame_id = self._frame_id
        self._map_publisher.publish(to_occupancy_grid(values, message.info, header))

        # 원본은 약 5천만 셀이라 RViz Map/GridMap 플러그인이 텍스처·정점
        # 메모리를 크게 잡고 SIGKILL로 끝날 수 있다. Nav2에는 원본을 그대로
        # 보내고, 시각화 전용 토픽만 보수적으로 블록 축소한다.
        factor = max(1, int(self.get_parameter('preview_factor').value))
        sx, sy = values.shape
        px, py = (sx + factor - 1) // factor, (sy + factor - 1) // factor
        padded = np.full((px * factor, py * factor), self._unknown,
                         dtype=np.int8)
        padded[:sx, :sy] = values
        blocks = padded.reshape(px, factor, py, factor)
        occupied = np.any(blocks == OCCUPIED, axis=(1, 3))
        free = np.any(blocks == FREE, axis=(1, 3))
        preview = np.full((px, py), self._unknown, dtype=np.int8)
        preview[free] = FREE
        preview[occupied] = OCCUPIED
        preview_info = type(message.info)()
        preview_info.resolution = message.info.resolution * factor
        preview_info.length_x = message.info.length_x
        preview_info.length_y = message.info.length_y
        preview_info.pose = message.info.pose
        self._preview_publisher.publish(
            to_occupancy_grid(preview, preview_info, header))
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
