#!/usr/bin/env python3
"""1단계 bag 에 담긴 주행성 지도를 latched 로 계속 내보낸다.

    replay_nav_map.py <bag 디렉터리> [--topics /wheel/nav_map /leg/nav_map]

왜 필요한가: 종단 테스트를 두 시뮬레이션으로 나눴다. 1단계(드론 단독)가 모듈
A->F 로 주행성 지도를 만들고, 2단계(로봇 주행)는 그 지도 위에서 Nav2 를 돌린다.
2단계에서 모듈 F 를 다시 돌리려면 드론을 또 46.7 km 날려야 하므로, 1단계가
남긴 지도를 그대로 공급한다.

`ros2 bag play` 를 쓰지 않는 이유는 QoS 다. Nav2 의 static_layer 는
`map_subscribe_transient_local: true` 로 붙기 때문에 발행자가
TRANSIENT_LOCAL 이어야 늦게 뜬 코스트맵도 지도를 받는다. bag play 는 기본
VOLATILE 이라 코스트맵이 먼저 안 떠 있으면 지도를 영영 못 받는다 -- 실제로
이 프로젝트에서 static_layer 가 지도를 못 받아 종단 검증이 막힌 적이 있다
(nav2_common.yaml 의 map_topic 주석 참고).

각 토픽의 **마지막** 메시지를 쓴다. 모듈 F 는 비행 완료 시 한 번만 내보내므로
보통 1개지만, 여러 개면 가장 나중 것이 완성본이다.
"""
import argparse

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def read_last(bag, topics):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'),
                ConverterOptions('', ''))
    want = set(topics)
    found = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in want:
            found[topic] = deserialize_message(data, OccupancyGrid)
    return found


def main():
    p = argparse.ArgumentParser()
    p.add_argument('bag')
    p.add_argument('--topics', nargs='+',
                   default=['/wheel/nav_map', '/leg/nav_map'])
    a = p.parse_args()

    maps = read_last(a.bag, a.topics)
    missing = [t for t in a.topics if t not in maps]
    if missing:
        raise SystemExit('bag 에 %s 가 없다. 1단계 모듈 F 가 지도를 내보냈는지 '
                         '확인하라.' % ', '.join(missing))

    rclpy.init()
    node = Node('nav_map_replayer')
    pubs = {}
    for topic, msg in maps.items():
        pubs[topic] = node.create_publisher(OccupancyGrid, topic, LATCHED)
        pubs[topic].publish(msg)
        node.get_logger().info(
            '%s 발행: %d x %d 셀, 해상도 %.3f m, 원점 (%.2f, %.2f), '
            '미지 %.1f%% / 통과가능 %.1f%% / 막힘 %.1f%%'
            % (topic, msg.info.width, msg.info.height, msg.info.resolution,
               msg.info.origin.position.x, msg.info.origin.position.y,
               100.0 * sum(1 for v in msg.data if v < 0) / max(1, len(msg.data)),
               100.0 * sum(1 for v in msg.data if v == 0) / max(1, len(msg.data)),
               100.0 * sum(1 for v in msg.data if v > 0) / max(1, len(msg.data))))

    # TRANSIENT_LOCAL 이라 늦게 붙는 구독자도 마지막 메시지를 받는다.
    # 노드는 계속 살려 둔다 -- 죽으면 latched 샘플도 함께 사라진다.
    rclpy.spin(node)


if __name__ == '__main__':
    main()
