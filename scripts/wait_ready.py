#!/usr/bin/env python3
"""시뮬 스택이 "다 떴는지"를 기다린다. 고정 sleep 대신 쓰는 준비 완료 게이트.

고정 시간으로 기다리면 안 되는 이유:
이 월드는 실시간계수(RTF)가 0.3~0.4다. 벽시계 120초를 기다려도 시뮬 시간으로는
40여 초밖에 지나지 않고, EKF의 첫 GPS fix·컨트롤러 활성화·Nav2 lifecycle 전이는
전부 **시뮬 시간** 기준으로 일어난다. 그래서 같은 sleep을 줘도 그날 머신 부하에
따라 어떤 실행은 준비가 끝나 있고 어떤 실행은 아직이라, 점검이 "아직 안 뜬 것"을
"고장난 것"으로 잡아내는 오탐이 난다.

여기서는 **실제로 준비됐다는 신호**를 기다린다. 제한 시간 안에 못 갖추면 1로
끝나므로, 진짜 고장을 놓치지도 않는다.

  1. /clock 이 흐른다 (시뮬이 돌고 있다)
  2. 로봇별 EKF 출력 /X/odometry/filtered 가 나온다 (모듈 B 준비 완료)
  3. map -> X/base_link TF가 조회된다 (전체 TF 체인 연결)
  4. 로봇별 Nav2 bt_navigator 가 active
  5. 모듈 C의 지면 분할 결과 /X/points_filtered 가 실제로 나온다

사용법:
    ./scripts/wait_ready.py --timeout 420
종료코드: 0 = 준비됨, 1 = 시간 초과
"""
import argparse
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import tf2_ros

ROBOTS = ('wheel', 'leg')


class ReadinessGate(Node):
    def __init__(self):
        super().__init__('readiness_gate')
        self.odom_seen = set()
        self.cloud_seen = set()
        for robot in ROBOTS:
            self.create_subscription(
                Odometry, f'/{robot}/odometry/filtered',
                lambda _m, r=robot: self.odom_seen.add(r), 10)
            # 모듈 C는 Nav2와 함께 뜨므로 가장 늦게 준비된다. 이것까지 기다려야
            # 점검이 "아직 안 뜬 것"을 "고장난 것"으로 잡지 않는다.
            self.create_subscription(
                PointCloud2, f'/{robot}/points_filtered',
                lambda _m, r=robot: self.cloud_seen.add(r),
                qos_profile_sensor_data)
        self._state_clients = {
            r: self.create_client(GetState, f'/{r}/bt_navigator/get_state')
            for r in ROBOTS
        }
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, None, spin_thread=True)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def tf_ok(self, robot):
        try:
            self.tf_buffer.lookup_transform(
                'map', f'{robot}/base_link', rclpy.time.Time())
            return True
        except Exception:                                     # noqa: BLE001
            return False

    def nav2_active(self, robot):
        # 클라이언트는 __init__에서 한 번만 만든다.
        # 호출할 때마다 create/destroy 하면 아직 처리 중인 future가 남은 채로
        # 파괴돼 rclpy가 InvalidHandle을 던지고 이 스크립트가 죽는다
        #   InvalidHandle: cannot use Destroyable because destruction was requested
        # 그러면 게이트가 실패로 끝나 점검이 기동 도중에 들어간다(실측 1회).
        cli = self._state_clients[robot]
        if not cli.service_is_ready():
            return False
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False
        return fut.result().current_state.label == 'active'

    def missing(self):
        out = []
        for robot in ROBOTS:
            if robot not in self.odom_seen:
                out.append(f'/{robot}/odometry/filtered')
            if not self.tf_ok(robot):
                out.append(f'map->{robot}/base_link')
            if not self.nav2_active(robot):
                out.append(f'{robot}/bt_navigator active')
            if robot not in self.cloud_seen:
                out.append(f'/{robot}/points_filtered')
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=420.0)
    ap.add_argument('--settle', type=float, default=20.0,
                    help='준비 확인 후 추가로 안정화시킬 시간(초)')
    args = ap.parse_args()

    rclpy.init()
    node = ReadinessGate()
    start = time.monotonic()
    deadline = start + args.timeout
    last_report = ''

    while time.monotonic() < deadline:
        node.spin(5.0)
        missing = node.missing()
        if not missing:
            print(f'[wait_ready] {time.monotonic()-start:.0f}s 만에 준비 완료', flush=True)
            node.spin(args.settle)
            node.destroy_node()
            rclpy.shutdown()
            return 0
        report = ', '.join(missing)
        if report != last_report:
            print(f'[wait_ready] {time.monotonic()-start:6.0f}s 대기중: {report}',
                  flush=True)
            last_report = report

    print(f'[wait_ready] 시간 초과 {args.timeout:.0f}s — 남은 항목: '
          f'{", ".join(node.missing())}', file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
