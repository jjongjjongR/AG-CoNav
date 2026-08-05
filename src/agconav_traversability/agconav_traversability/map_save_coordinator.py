"""지도 저장 제어 (모듈 F).

wheel·leg 주행 가능 맵 생성 완료 상태가 모두 True가 되면, map_saver_server에
SaveMap 서비스를 wheel → leg 순서로 1회씩 호출한다. 중복 저장은 하지 않고
실패하면 retry_count까지 재시도한다.
"""
import os

import rclpy
from nav2_msgs.srv import SaveMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

# 상태 토픽 QoS — 판정 노드가 래치로 1회 발행한다.
STATUS_QOS = QoSProfile(depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)

# map_saver.yaml의 기본 임계값과 같은 값 (README §3.3의 자유/점유 규약).
FREE_THRESH = 0.25
OCCUPIED_THRESH = 0.65


class MapSaveCoordinator(Node):

    def __init__(self):
        super().__init__('map_save_coordinator')
        self.declare_parameter('wheel_status_topic', '/wheel/nav_map_status')
        self.declare_parameter('leg_status_topic', '/leg/nav_map_status')
        self.declare_parameter('save_service', '/map_saver/save_map')
        self.declare_parameter('wheel_map_topic', '/wheel/nav_map')
        self.declare_parameter('leg_map_topic', '/leg/nav_map')
        self.declare_parameter('wheel_map_url', 'maps/wheel_nav_map')
        self.declare_parameter('leg_map_url', 'maps/leg_nav_map')
        self.declare_parameter('image_format', 'pgm')
        self.declare_parameter('map_mode', 'trinary')
        self.declare_parameter('retry_count', 3)

        self._image_format = self.get_parameter('image_format').value
        self._map_mode = self.get_parameter('map_mode').value
        self._retry_count = self.get_parameter('retry_count').value
        # 저장 순서: wheel 먼저, 응답 확인 후 leg.
        self._jobs = [
            ('wheel', self.get_parameter('wheel_map_topic').value,
             self.get_parameter('wheel_map_url').value),
            ('leg', self.get_parameter('leg_map_topic').value,
             self.get_parameter('leg_map_url').value),
        ]
        self._ready = {'wheel': False, 'leg': False}
        self._started = False

        self._client = self.create_client(
            SaveMap, self.get_parameter('save_service').value)
        self.create_subscription(
            Bool, self.get_parameter('wheel_status_topic').value,
            lambda msg: self._on_status('wheel', msg), STATUS_QOS)
        self.create_subscription(
            Bool, self.get_parameter('leg_status_topic').value,
            lambda msg: self._on_status('leg', msg), STATUS_QOS)

    def _on_status(self, robot, message):
        self._ready[robot] = message.data
        if self._started or not all(self._ready.values()):
            return
        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('map_saver 저장 서비스가 올라오지 않았다.')
            return
        self._started = True
        self._save(0, 1)

    def _save(self, index, attempt):
        if index >= len(self._jobs):
            self.get_logger().info('wheel·leg 주행 가능 맵 저장 완료.')
            return
        robot, map_topic, map_url = self._jobs[index]
        # map_saver는 상위 디렉터리를 만들지 않는다. maps/는 .gitignore 대상이라
        # 새로 clone한 작업공간에는 없으므로, 저장 전에 직접 만들어 준다.
        directory = os.path.dirname(map_url)
        if directory:
            os.makedirs(directory, exist_ok=True)
        request = SaveMap.Request()
        request.map_topic = map_topic
        request.map_url = map_url
        request.image_format = self._image_format
        request.map_mode = self._map_mode
        request.free_thresh = FREE_THRESH
        request.occupied_thresh = OCCUPIED_THRESH
        self.get_logger().info(f'{robot} 지도 저장 요청 ({attempt}회차): {map_url}')
        future = self._client.call_async(request)
        future.add_done_callback(
            lambda done: self._on_response(done, index, attempt))

    def _on_response(self, future, index, attempt):
        robot = self._jobs[index][0]
        response = future.result()
        if response is not None and response.result:
            self._save(index + 1, 1)
        elif attempt < self._retry_count:
            self.get_logger().warn(f'{robot} 지도 저장 실패, 재시도한다.')
            self._save(index, attempt + 1)
        else:
            self.get_logger().error(
                f'{robot} 지도 저장을 {attempt}회 시도했지만 실패했다.')


def main():
    rclpy.init()
    node = MapSaveCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
