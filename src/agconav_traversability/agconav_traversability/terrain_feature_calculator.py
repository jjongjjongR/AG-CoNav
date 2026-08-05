"""지형 특성 계산 (모듈 F).

드론 지도 생성 완료 신호(/drone/elevation_map_status)를 받으면, 마지막으로 받은
드론 2.5D 지도(/drone/elevation_map)로 셀별 경사·단차를 1회 계산해
/terrain/features(layer: slope, step)로 발행한다. 반복 갱신은 하지 않는다.
"""
import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from agconav_traversability.grid_map_util import array_to_layer, layer_to_array

# 지도 토픽 QoS — README §3.4: reliable + transient_local(래치).
MAP_QOS = QoSProfile(depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _shift(array, di, dj):
    """array를 (di, dj)만큼 민다. 지도 밖으로 나간 자리는 NaN(미관측)."""
    size_x, size_y = array.shape
    shifted = np.full_like(array, np.nan)
    shifted[max(0, -di):size_x - max(0, di),
            max(0, -dj):size_y - max(0, dj)] = \
        array[max(0, di):size_x - max(0, -di),
              max(0, dj):size_y - max(0, -dj)]
    return shifted


def compute_features(elevation, resolution, window):
    """이웃 셀과 높이를 비교해 (경사[rad], 단차[m])를 계산한다.

    - 단차: 인접 8셀과의 최대 높이차. 연석·턱 같은 높이 불연속을 잡는다.
    - 경사: window 반경 중심차분으로 구한 지형 기울기의 arctan.

    둘을 다른 방법으로 재야 "높이 0.10 m 턱"(단차 크고 경사 없음)과
    "완만한 비탈"(경사 크고 단차 작음)이 구분된다. 한 셀 차이로 경사를 재면
    턱이 곧 45°가 되어 두 기준이 같은 값이 되고, wheel·leg 판정이 갈리지 않는다.

    이웃이 지도 밖이거나 미관측(NaN)이면 그 방향은 빠진다. 단차는 남은 이웃으로
    계산하고(np.fmax), 경사는 양쪽이 있어야 하므로 NaN이 되어 미관측으로 넘어간다.
    """
    step = np.full_like(elevation, np.nan)
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            if di == 0 and dj == 0:
                continue
            step = np.fmax(step, np.abs(_shift(elevation, di, dj) - elevation))

    span = window * resolution
    gradient_x = (_shift(elevation, window, 0)
                  - _shift(elevation, -window, 0)) / (2.0 * span)
    gradient_y = (_shift(elevation, 0, window)
                  - _shift(elevation, 0, -window)) / (2.0 * span)
    slope = np.arctan(np.hypot(gradient_x, gradient_y))
    return slope, step


class TerrainFeatureCalculator(Node):

    def __init__(self):
        super().__init__('terrain_feature_calculator')
        self.declare_parameter('input_topic', '/drone/elevation_map')
        self.declare_parameter('trigger_topic', '/drone/elevation_map_status')
        self.declare_parameter('elevation_layer', 'elevation')
        self.declare_parameter('output_topic', '/terrain/features')
        self.declare_parameter('slope_window', 1)

        self._layer = self.get_parameter('elevation_layer').value
        self._window = self.get_parameter('slope_window').value
        self._last_map = None
        self._done = False

        self._publisher = self.create_publisher(
            GridMap, self.get_parameter('output_topic').value, MAP_QOS)
        self.create_subscription(
            GridMap, self.get_parameter('input_topic').value,
            self._on_map, MAP_QOS)
        self.create_subscription(
            Bool, self.get_parameter('trigger_topic').value,
            self._on_trigger, MAP_QOS)

    def _on_map(self, message):
        self._last_map = message

    def _on_trigger(self, message):
        if not message.data or self._done:
            return
        if self._last_map is None:
            self.get_logger().warn('완료 신호를 받았지만 드론 지도가 아직 없다.')
            return
        if self._layer not in self._last_map.layers:
            self.get_logger().error(f'드론 지도에 {self._layer} 레이어가 없다.')
            return

        elevation = layer_to_array(self._last_map, self._layer)
        slope, step = compute_features(
            elevation, self._last_map.info.resolution, self._window)

        features = GridMap()
        features.header = self._last_map.header
        features.info = self._last_map.info
        features.layers = ['slope', 'step']
        features.data = [array_to_layer(slope), array_to_layer(step)]
        self._publisher.publish(features)

        self._done = True
        self.get_logger().info(
            f'지형 특성 계산 완료 ({elevation.shape[0]}x{elevation.shape[1]} 셀).')


def main():
    rclpy.init()
    node = TerrainFeatureCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
