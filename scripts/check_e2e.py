#!/usr/bin/env python3
"""모듈 A→F→C→D→E 종단 파이프라인 검증.

시뮬이 떠 있는 상태에서 붙어, 1회성 산출물이 순서대로 나오는지 끝까지 지켜본다.

    모듈 A  드론 경로비행 -> /drone/elevation_map (+status)
      -> 모듈 F  /wheel/nav_map · /leg/nav_map (+status)
      -> 모듈 C  (여기서 목표를 보낸다) -> /X/navigation_status = True
      -> 모듈 D  /X/elevation_map (+status)
      -> 모듈 E  /merged/elevation_map (+merge_status)

모듈 C의 navigation_status는 Nav2 navigate_to_pose 액션 상태에서 나오므로,
누군가 목표를 보내지 않으면 파이프라인이 그 지점에서 멈춘다. 이 스크립트가
nav_map을 받은 뒤 로봇별로 현재 위치 근처의 짧은 목표를 직접 보낸다.

사용법:
    ./scripts/check_e2e.py --timeout 3600
종료코드: 0 = 끝까지 통과, 1 = 실패/시간초과
"""
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool
import tf2_ros

ROBOTS = ('wheel', 'leg')

LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class E2EChecker(Node):
    def __init__(self):
        super().__init__('e2e_checker')
        self.seen = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, None, spin_thread=True)

        def watch(msg_type, topic):
            self.create_subscription(
                msg_type, topic,
                lambda m, t=topic: self._mark(t, m), LATCHED)

        watch(GridMap, '/drone/elevation_map')
        watch(Bool, '/drone/elevation_map_status')
        for r in ROBOTS:
            watch(OccupancyGrid, f'/{r}/nav_map')
            watch(Bool, f'/{r}/nav_map_status')
            watch(Bool, f'/{r}/navigation_status')
            watch(GridMap, f'/{r}/elevation_map')
            watch(Bool, f'/{r}/elevation_map_status')
        watch(GridMap, '/merged/elevation_map')
        watch(Bool, '/merged/merge_status')

        self._nav_clients = {
            r: ActionClient(self, NavigateToPose, f'/{r}/navigate_to_pose')
            for r in ROBOTS
        }
        self._attempt = {r: 0 for r in ROBOTS}
        self._sent_at = {r: None for r in ROBOTS}

    def _mark(self, topic, msg):
        # Bool 계열은 True인 첫 메시지만 "도달"로 본다.
        # navigation_status는 이동 중 False를 계속 내보내기 때문이다.
        if isinstance(msg, Bool) and not msg.data:
            return
        self.seen.setdefault(topic, time.monotonic())

    def has(self, topic):
        return topic in self.seen

    def robot_pose(self, robot):
        tr = self.tf_buffer.lookup_transform(
            'map', f'{robot}/base_link', rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=5.0))
        return tr.transform.translation, tr.transform.rotation

    def send_goal(self, robot, forward):
        """현재 위치에서 로봇 진행 방향으로 forward m 앞을 목표로 보낸다.

        forward=0이면 제자리 목표라 계획 없이 바로 성공한다 — 실제 주행이
        막혔을 때도 모듈 C→D→E 배선을 끝까지 확인하기 위한 마지막 수단이다.
        """
        cli = self._nav_clients[robot]
        if not cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'{robot}: navigate_to_pose 액션 서버 없음')
            return False
        trans, rot = self.robot_pose(robot)
        yaw = math.atan2(2.0 * (rot.w * rot.z + rot.x * rot.y),
                         1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = trans.x + forward * math.cos(yaw)
        goal.pose.pose.position.y = trans.y + forward * math.sin(yaw)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = rot
        cli.send_goal_async(goal)
        self._sent_at[robot] = time.monotonic()
        self.get_logger().info(
            f'{robot}: 목표 전송 #{self._attempt[robot]} '
            f'({goal.pose.pose.position.x:.2f}, {goal.pose.pose.position.y:.2f}) '
            f'전방 {forward:.1f} m')
        return True

    def drive_module_c(self, robot, forward, retry_after=90.0):
        """nav_map이 나온 뒤 목표를 보내고, 실패하면 거리를 줄여 재시도한다."""
        if self.has(f'/{robot}/navigation_status'):
            return
        if not self.has(f'/{robot}/nav_map_status'):
            return
        sent = self._sent_at[robot]
        if sent is not None and time.monotonic() - sent < retry_after:
            return
        # 2 m -> 1 m -> 0 m(제자리) 순으로 낮춰가며 재시도
        distances = [forward, forward / 2.0, 0.0]
        idx = min(self._attempt[robot], len(distances) - 1)
        try:
            if self.send_goal(robot, distances[idx]):
                self._attempt[robot] += 1
        except Exception as exc:                              # noqa: BLE001
            self.get_logger().warn(f'{robot}: 목표 전송 실패 {exc}')
            self._sent_at[robot] = time.monotonic()


STAGES = [
    ('모듈 A  드론 2.5D 지도',      ['/drone/elevation_map',
                                     '/drone/elevation_map_status']),
    ('모듈 F  주행 가능 맵',        [f'/{r}/nav_map' for r in ROBOTS]
                                    + [f'/{r}/nav_map_status' for r in ROBOTS]),
    ('모듈 C  이동 완료',           [f'/{r}/navigation_status' for r in ROBOTS]),
    ('모듈 D  지상 누적 지도',      [f'/{r}/elevation_map' for r in ROBOTS]
                                    + [f'/{r}/elevation_map_status' for r in ROBOTS]),
    ('모듈 E  병합 지도',           ['/merged/elevation_map',
                                     '/merged/merge_status']),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=3600.0,
                    help='전체 제한 시간(초). 드론 경로비행만 약 34분이다.')
    ap.add_argument('--goal-forward', type=float, default=2.0)
    ap.add_argument('--skip-drone', action='store_true',
                    help='모듈 A(드론 경로비행 74분)를 건너뛰고 모듈 F부터 본다. '
                         '저장해 둔 nav_map을 대신 발행할 때 쓴다.')
    args = ap.parse_args()

    stages = STAGES[1:] if args.skip_drone else STAGES

    rclpy.init()
    node = E2EChecker()
    start = time.monotonic()
    deadline = start + args.timeout
    reported = set()

    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

        for name, topics in stages:
            if name in reported:
                continue
            if all(node.has(t) for t in topics):
                reported.add(name)
                print(f'[{time.monotonic()-start:7.1f}s] 통과  {name}', flush=True)

        # nav_map이 나왔으면 모듈 C를 굴리기 위해 목표를 보낸다.
        for r in ROBOTS:
            node.drive_module_c(r, args.goal_forward)

        if len(reported) == len(stages):
            break

    print()
    print('=' * 72)
    ok = True
    for name, topics in stages:
        done = all(node.has(t) for t in topics)
        ok &= done
        print(f'  {"OK  " if done else "FAIL"} | {name}')
        for t in topics:
            mark = f'{node.seen[t]-start:7.1f}s' if node.has(t) else '   미도달'
            print(f'         {mark}  {t}')
    print('=' * 72)

    node.destroy_node()
    rclpy.shutdown()
    print('종단 파이프라인 전체 통과' if ok else '종단 파이프라인 미완료')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
