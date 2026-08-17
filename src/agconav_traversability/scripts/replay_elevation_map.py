#!/usr/bin/env python3
"""Replay one saved drone GridMap and its completion status with latched QoS.

This is the module-F counterpart of ``replay_nav_map.py``.  A saved
elevation-map bag contains the final GridMap but not necessarily the separate
completion signal that causes terrain_feature_calculator to run.  Publishing
both here also preserves the reliable/transient-local contract used by the
live module-A nodes.
"""

import argparse

from grid_map_msgs.msg import GridMap
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from std_msgs.msg import Bool


LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def read_last(bag, topic):
    """Return the last GridMap stored on topic in bag."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'),
                ConverterOptions('', ''))
    result = None
    while reader.has_next():
        current_topic, data, _ = reader.read_next()
        if current_topic == topic:
            result = deserialize_message(data, GridMap)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--map-topic', default='/drone/elevation_map')
    parser.add_argument('--status-topic', default='/drone/elevation_map_status')
    args = parser.parse_args()

    grid_map = read_last(args.bag, args.map_topic)
    if grid_map is None:
        raise SystemExit(f'{args.bag} 에 {args.map_topic} 메시지가 없다.')

    rclpy.init()
    node = Node('elevation_map_replayer')
    map_pub = node.create_publisher(GridMap, args.map_topic, LATCHED)
    status_pub = node.create_publisher(Bool, args.status_topic, LATCHED)
    map_pub.publish(grid_map)
    status_sent = False

    def publish_status():
        nonlocal status_sent
        # !! 한 번만 보내면 안 된다 !!
        # 지도가 112 MB(2,808만 칸)라 RELIABLE 전송이 몇 초 걸리는데, 완료
        # 신호는 Bool 한 개라 먼저 도착한다. 그러면 모듈 F 가
        #   "완료 신호를 받았지만 드론 지도가 아직 없다"
        # 로 흘려버리고, 그 뒤로 아무 일도 일어나지 않는다. 실제로 이것 때문에
        # 파이프라인이 F 에서 멈추고 지상 로봇이 스폰조차 되지 않았다.
        # 토픽이 다르면 도착 순서가 보장되지 않으므로, 신호를 계속 보내
        # 지도가 도착한 뒤의 신호가 반드시 한 번은 걸리게 한다.
        status_pub.publish(Bool(data=True))
        if not status_sent:
            status_sent = True
            node.get_logger().info('완료 신호 발행 시작 (5초 간격 반복)')

    node.create_timer(5.0, publish_status)
    node.get_logger().info(
        '%s 발행: %.1f x %.1f m, 해상도 %.3f m, 레이어 %s; %s=True'
        % (args.map_topic, grid_map.info.length_x, grid_map.info.length_y,
           grid_map.info.resolution, ','.join(grid_map.layers),
           args.status_topic))

    # A transient-local sample exists only while its publisher is alive.
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
