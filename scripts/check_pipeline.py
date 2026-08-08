#!/usr/bin/env python3
"""파이프라인 실동작 점검 — "연결돼 있다"가 아니라 "실제로 흐른다"를 본다.

check_consistency.py는 센서·TF·위치추정을, check_integration.py는 토픽 양단에
발행자/구독자가 붙었는지를 본다. 둘 다 통과해도 Nav2 lifecycle이 configure에
실패해 통째로 비활성이거나, 노드가 콜백마다 예외로 죽어 아무것도 발행하지 않는
상태를 잡지 못한다. 이 스크립트는 그 두 가지를 직접 확인한다.

  1. Nav2 lifecycle 노드가 wheel·leg 양쪽에서 전부 active 인가
     (하나라도 configure에 실패하면 lifecycle manager가 bringup 전체를 중단한다)
  2. 모듈 C의 지면 제거 결과 /X/points_filtered가 실제로 들어오는가
  3. 모듈 F/E 등 1회성 산출물의 발행 주체가 살아 있는가 (노드 존재 확인)

사용법:
    ./scripts/check_pipeline.py
    ./scripts/check_pipeline.py --quiet
종료코드: 0 = 전부 통과, 1 = 실패 있음
"""
import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

ROBOTS = ('wheel', 'leg')

# navigation_launch.py의 lifecycle_nodes 하드코딩 리스트와 같아야 한다.
# 이 중 하나라도 configure에 실패하면 lifecycle manager가
# "Failed to bring up all requested nodes. Aborting bringup." 으로 전체를 중단한다.
NAV2_LIFECYCLE_NODES = (
    'controller_server',
    'smoother_server',
    'planner_server',
    'route_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
    'waypoint_follower',
    'docking_server',
)

# 실제 메시지가 흘러야 하는 토픽 (모듈 C 출력)
FLOWING = [f'/{r}/points_filtered' for r in ROBOTS]

# 살아 있어야 하는 노드 (1회성 산출물이라 토픽만으로는 확인이 안 되는 모듈)
REQUIRED_NODES = [
    '/drone_elevation_mapper',              # 모듈 A
    '/terrain_feature_calculator',          # 모듈 F
    '/traversability_verdictor_wheel',
    '/traversability_verdictor_leg',
    '/map_merge_collector',                 # 모듈 E
    '/elevation_map_merger',
    '/wheel/ground_elevation_mapper',       # 모듈 D
    '/leg/ground_elevation_mapper',
]


class PipelineChecker(Node):
    def __init__(self):
        super().__init__('pipeline_checker')
        self.counts = {t: 0 for t in FLOWING}
        for topic in FLOWING:
            self.create_subscription(
                PointCloud2, topic,
                lambda _m, t=topic: self.counts.__setitem__(t, self.counts[t] + 1),
                qos_profile_sensor_data)

    def collect(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def lifecycle_state(self, node_name, timeout=5.0):
        """lifecycle 노드의 현재 상태 라벨을 돌려준다. 실패하면 사유 문자열."""
        cli = self.create_client(GetState, f'{node_name}/get_state')
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return 'no service'
            future = cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                return 'no response'
            return future.result().current_state.label
        finally:
            self.destroy_client(cli)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--flow-seconds', type=float, default=12.0)
    args = ap.parse_args()

    rclpy.init()
    node = PipelineChecker()

    # 그래프 탐색 + 메시지 수집
    node.collect(args.flow_seconds)

    results = []

    def add(ok, group, msg):
        results.append((ok, group, msg))

    # 1. Nav2 lifecycle
    for robot in ROBOTS:
        for name in NAV2_LIFECYCLE_NODES:
            full = f'/{robot}/{name}'
            label = node.lifecycle_state(full)
            add(label == 'active', 'Nav2 lifecycle', f'{full:44s} {label}')

    # 2. 실제 메시지 흐름
    for topic in FLOWING:
        n = node.counts[topic]
        add(n > 0, '메시지 흐름',
            f'{topic:26s} {n}건 수신' + ('' if n else '  (모듈 C 지면분할 미동작)'))

    # 3. 필수 노드 생존
    alive = set(node.get_node_names_and_namespaces())
    alive_full = {(ns.rstrip('/') + '/' + n) for n, ns in alive}
    for full in REQUIRED_NODES:
        add(full in alive_full, '노드 생존', full)

    node.destroy_node()
    rclpy.shutdown()

    failed = [r for r in results if not r[0]]
    if not args.quiet:
        print('=' * 96)
        print('파이프라인 실동작 점검')
        print('=' * 96)
        last = None
        for ok, group, msg in results:
            if group != last:
                print(f'\n[{group}]')
                last = group
            print(f'  {"OK  " if ok else "FAIL"} | {msg}')
        print('\n' + '=' * 96)
    if failed:
        if args.quiet:
            for _, group, msg in failed:
                print(f'FAIL | [{group}] {msg}')
        print(f'실패 {len(failed)} / 전체 {len(results)}')
        return 1
    print(f'전체 {len(results)}항목 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
