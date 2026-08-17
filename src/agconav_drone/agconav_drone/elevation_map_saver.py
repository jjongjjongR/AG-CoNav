"""Saves the drone's elevation map to an mcap rosbag2 as soon as it arrives.

Structure/저장 로직은 채현우님의
agconav_ground_mapping/ground_elevation_map_saver.py를 그대로 재사용한다:
elevation_map을 받는 순간 자체가 "지도 완성됨"을 의미하므로(오늘 만든
drone_elevation_mapper가 /drone/path_status를 받은 뒤 딱 한 번만
elevation_map을 발행하니까) 별도로 완료 신호를 또 구독하지 않고 콜백에서
바로 저장한다. ground_elevation_map_saver도 원래는 saver가 완료 토픽을
독자적으로 구독했다가 mapper의 발행보다 먼저 저장을 시도하는 race가
있었고, "발행 쪽이 완료 신호를 갖고 있다가 한 번만 쏘고 그 발행 자체를
저장 트리거로 삼는" 방식으로 바꿔 해결했다고 되어 있어 그대로 따른다.
rosbag2_py.SequentialWriter로 mcap에 기록하고 del writer로 즉시
flush/close하는 것도 동일.

드론이라 달라진 부분:
- 토픽: 드론 1대뿐이라 wheel/leg처럼 launch 네임스페이스로 나누지 않고
  /drone/elevation_map, /drone/elevation_map_status로 고정.
- 저장 실패 시 동작이 ground보다 명시적이다: ground_elevation_map_saver는
  성공(True)만 발행하고 실패 시에는 로그만 남기는데, 여기서는 실패도
  elevation_map_status에 False로 발행하고, drone_pose_controller가 이미
  쓰고 있는 공용 /status(String) 토픽에도 실패 사유를 남긴다 - 드론
  스택 전체가 그 토픽을 "무슨 일이 실패했는지"의 공용 채널로 쓰고 있어서
  (drone_pose_controller.py 참고) 맞춰준다.
"""

import os

from grid_map_msgs.msg import GridMap
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class DroneElevationMapSaver(Node):
    """Buffers the latest elevation_map and writes it to mcap on arrival."""

    def __init__(self):
        super().__init__('elevation_map_saver')

        self.declare_parameter('input_topic', '/drone/elevation_map')
        self.declare_parameter('output_directory', 'maps')
        self.declare_parameter('map_name', 'drone_elevation_map')
        self.declare_parameter('output_format', 'mcap')
        self.declare_parameter('enable_manual_save_service', False)
        self.declare_parameter('status_topic', '/drone/elevation_map_status')

        self._input_topic = self.get_parameter('input_topic').value
        self._output_directory = self.get_parameter('output_directory').value
        self._map_name = self.get_parameter('map_name').value
        self._output_format = self.get_parameter('output_format').value
        self._status_topic = self.get_parameter('status_topic').value

        self._latest_elevation_map = None

        # ground_elevation_map_saver와 동일: elevation_map은 reliable /
        # transient_local / keep_last / depth 1 (한 번만 오는 최종 지도).
        elevation_map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._elevation_map_sub = self.create_subscription(
            GridMap, self._input_topic, self._elevation_map_callback, elevation_map_qos)
        self._status_pub = self.create_publisher(Bool, self._status_topic, status_qos)
        # drone_pose_controller.py가 실패 사유를 남기는 것과 같은 공용
        # /status(String) 채널 - QoS는 drone_pose_controller와 동일하게
        # 기본값(volatile) 그대로 둔다.
        self._text_status_pub = self.create_publisher(String, '/status', 10)

        self._save_service = None
        if self.get_parameter('enable_manual_save_service').value:
            self._save_service = self.create_service(
                Trigger, 'save_elevation_map', self._manual_save_callback)

        self.get_logger().info(
            f'input_topic="{self._input_topic}", '
            f'output="{os.path.join(self._output_directory, self._map_name)}" '
            f'({self._output_format}), '
            f'status_topic="{self._status_topic}"')

    def _elevation_map_callback(self, msg):
        # drone_elevation_mapper는 /drone/path_status가 True가 된 뒤 딱 한
        # 번만 elevation_map을 발행하므로, 여기서 받는 순간 자체가 "경로
        # 완료 + 최종 지도 준비 완료"를 의미한다 - 별도 완료 토픽 구독이나
        # 서비스 호출 없이 바로 저장한다.
        self._latest_elevation_map = msg
        self._save_elevation_map()

    def _manual_save_callback(self, request, response):
        response.success, response.message = self._save_elevation_map()
        return response

    def _save_elevation_map(self):
        if self._latest_elevation_map is None:
            message = 'no elevation_map received yet, nothing to save.'
            self.get_logger().warn(message)
            return False, message

        bag_path = os.path.join(self._output_directory, self._map_name)
        try:
            self._archive_existing_bag(bag_path)
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id=self._output_format),
                rosbag2_py.ConverterOptions('', ''),
            )
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=0,
                name=self._input_topic,
                type='grid_map_msgs/msg/GridMap',
                serialization_format='cdr',
            ))
            stamp = self._latest_elevation_map.header.stamp
            timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            writer.write(
                self._input_topic,
                serialize_message(self._latest_elevation_map),
                timestamp_ns,
            )
            del writer  # flush/close the bag now, rather than at GC time
        except Exception as ex:
            message = f'failed to save elevation map to "{bag_path}": {ex}'
            self.get_logger().error(message)
            self._status_pub.publish(Bool(data=False))
            self._text_status_pub.publish(String(data=f'elevation map 저장 실패: {message}'))
            return False, message

        message = f'saved elevation map to "{bag_path}"'
        self.get_logger().info(message)
        self._status_pub.publish(Bool(data=True))
        return True, message

    def _archive_existing_bag(self, bag_path):
        if not os.path.exists(bag_path):
            return
        backup_path = f'{bag_path}.previous'
        suffix = 2
        while os.path.exists(backup_path):
            backup_path = f'{bag_path}.previous.{suffix}'
            suffix += 1
        os.rename(bag_path, backup_path)
        self.get_logger().warn(
            f'existing bag preserved as "{backup_path}" before saving this run')


def main(args=None):
    rclpy.init(args=args)
    node = DroneElevationMapSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
