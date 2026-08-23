#!/usr/bin/env python3
"""주행 명령 배선 점검 — /X/cmd_vel을 넣으면 로봇이 실제로 움직이는가.

모듈 C(Nav2)가 내보내는 것과 **같은 토픽·같은 메시지 타입**으로 직접 속도를
넣고, odometry가 실제로 변하는지 본다. 토픽 이름이나 타입이 어긋나면 발행자와
구독자가 아예 연결되지 않아 조용히 아무 일도 일어나지 않는데, 그 상태를
그래프 점검(pub/sub 개수)만으로는 잡지 못한다.

  wheel : /wheel/cmd_vel (TwistStamped)
          -> twist_mux -> /wheel/platform/cmd_vel -> diff_drive_controller
          ros2_controllers Jazzy의 diff_drive_controller는 TwistStamped 전용이다.
  leg   : /leg/cmd_vel (Twist) -> CHAMP quadruped_controller (cmd_vel/smooth 리맵)

사용법:
    ./scripts/check_drive.py
종료코드: 0 = 두 로봇 다 움직임, 1 = 하나라도 안 움직임
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

# 로봇: (cmd 토픽, 메시지 타입, odom 토픽, 선속도 m/s)
ROBOTS = {
    'wheel': ('/wheel/cmd_vel', TwistStamped, '/wheel/platform/odom', 0.4),
    'leg':   ('/leg/cmd_vel',   Twist,        '/leg/odom',            0.3),
}


class DriveChecker(Node):
    def __init__(self):
        super().__init__('drive_checker')
        self.pos = {}
        self.pubs = {}
        for robot, (cmd_topic, cmd_type, odom_topic, _v) in ROBOTS.items():
            self.pubs[robot] = self.create_publisher(cmd_type, cmd_topic, 10)
            self.create_subscription(
                Odometry, odom_topic,
                lambda m, r=robot: self.pos.__setitem__(
                    r, (m.pose.pose.position.x, m.pose.pose.position.y)),
                10)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for robot, (_t, cmd_type, _o, v) in ROBOTS.items():
                if cmd_type is TwistStamped:
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = f'{robot}/base_link'
                    msg.twist.linear.x = v
                else:
                    msg = Twist()
                    msg.linear.x = v
                self.pubs[robot].publish(msg)
            self.spin_for(0.1)
        # 정지 명령
        for robot, (_t, cmd_type, _o, _v) in ROBOTS.items():
            self.pubs[robot].publish(cmd_type())
        self.spin_for(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--drive-seconds', type=float, default=8.0)
    ap.add_argument('--min-distance', type=float, default=0.15,
                    help='이 거리(m) 이상 움직여야 통과')
    args = ap.parse_args()

    rclpy.init()
    node = DriveChecker()
    node.spin_for(8.0)          # odom 첫 수신 대기 + 그래프 탐색

    start = dict(node.pos)
    missing = [r for r in ROBOTS if r not in start]
    node.drive(args.drive_seconds)
    end = dict(node.pos)

    results = []
    for robot, (cmd_topic, cmd_type, odom_topic, _v) in ROBOTS.items():
        if robot in missing or robot not in end:
            results.append((False, f'{robot:5s} odometry({odom_topic}) 수신 없음'))
            continue
        d = math.dist(start[robot], end[robot])
        results.append((
            d >= args.min_distance,
            f'{robot:5s} {cmd_topic} ({cmd_type.__name__}) -> {d:.3f} m 이동 '
            f'(기준 {args.min_distance} m)'))

    node.destroy_node()
    rclpy.shutdown()

    failed = [r for r in results if not r[0]]
    if not args.quiet:
        print('=' * 80)
        print('주행 명령 배선 점검')
        print('=' * 80)
        for ok, msg in results:
            print(f'  {"OK  " if ok else "FAIL"} | {msg}')
        print('=' * 80)
    if failed:
        if args.quiet:
            for _, msg in failed:
                print(f'FAIL | {msg}')
        print(f'실패 {len(failed)} / 전체 {len(results)}')
        return 1
    print(f'전체 {len(results)}항목 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
