#!/usr/bin/env python3
"""프레임·토픽 정합성 자동 점검.

실행 중인 시뮬레이션에 붙어서, 로봇 3종의 센서 토픽·frame_id·TF 연결이
topics.md의 계약과 맞는지 한 번에 확인한다. 매번 손으로 ros2 topic/tf를
두드리지 않기 위한 스크립트.

사용법:
    ./scripts/check_consistency.py            # 전체 점검
    ./scripts/check_consistency.py --quiet    # 실패만 출력

종료코드: 0 = 전부 통과, 1 = 실패 있음 (CI/launch 테스트에서 사용 가능)
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped  # noqa: F401  (tf2 등록용)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from nav_msgs.msg import Odometry
import tf2_ros

# ── 계약 (topics.md §1, §2) ────────────────────────────────────────────
SENSORS = {
    # 토픽: (타입, 기대 frame_id, 기대 점군 크기 또는 None)
    '/drone/points': (PointCloud2, 'drone/os1_lidar', 1024 * 32),
    '/drone/imu':    (Imu,         'drone/base_link', None),
    '/drone/gps':    (NavSatFix,   'drone/gps_link',  None),
    '/leg/points':   (PointCloud2, 'leg/os1_lidar',   1024 * 32),
    '/leg/imu':      (Imu,         'leg/base_link',   None),
    '/leg/gps':      (NavSatFix,   'leg/gps_link',    None),
    '/wheel/points': (PointCloud2, 'wheel/lidar3d_0_sensor_link', 1024 * 32),
    '/wheel/imu':    (Imu,         'wheel/imu_0_link',            None),
    '/wheel/gps':    (NavSatFix,   'wheel/gps_0_link',            None),
}

# clearpath가 시스템 xacro에서 gz_frame_id를 ${name}_sensor_link로 고정해
# 접두어를 못 붙인다. 소비자(모듈 D)는 target_source_frame 파라미터로 덮어쓴다.
KNOWN_FRAME_EXCEPTIONS = {
    '/wheel/points': 'lidar3d_0_sensor_link',
    '/wheel/imu': 'imu_0_link',
    '/wheel/gps': 'gps_0_link',
}

# map까지 변환 가능해야 하는 프레임.
# drone/os1_lidar는 모듈 A(drone_sim_test.launch.py)가 발행한다.
# agconav_sim.launch.py만 단독으로 띄우면 이 항목은 실패한다 —
# agconav_all.launch.py로 모듈 A까지 함께 띄운 상태를 전제로 한다.
TF_TARGETS = ['drone/base_link', 'drone/os1_lidar', 'drone/gps_link',
              'wheel/base_link', 'leg/base_link']

# 모듈 B 출력 (GPS 배선이 살아있는지)
LOCALIZATION = ['/wheel/gps/odom', '/leg/gps/odom',
                '/wheel/platform/odom', '/leg/odom',
                '/wheel/odometry/filtered', '/leg/odometry/filtered']


class Checker(Node):
    def __init__(self):
        super().__init__('consistency_checker')
        self.msgs = {}
        for topic, (msg_type, _, _) in SENSORS.items():
            self.create_subscription(
                msg_type, topic,
                lambda m, t=topic: self.msgs.setdefault(t, m),
                qos_profile_sensor_data)
        for topic in LOCALIZATION:
            self.create_subscription(
                Odometry, topic, lambda m, t=topic: self.msgs.setdefault(t, m), 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def collect(self, seconds, min_tf_seconds=4.0):
        """토픽 메시지를 모은다.

        토픽이 다 들어와도 최소 min_tf_seconds 동안은 계속 spin한다.
        TF는 여러 발행자(EKF의 map->X/odom, relay의 X/odom->X/base_link)가
        각자 주기로 내보내므로, 버퍼가 덜 찬 상태로 조회하면 실제로는 이어져
        있는데도 ConnectivityException이 난다(간헐 오탐).
        """
        start = time.monotonic()
        end = start + seconds
        want = len(SENSORS) + len(LOCALIZATION)
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.msgs) >= want and time.monotonic() - start >= min_tf_seconds:
                break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quiet', action='store_true', help='실패만 출력')
    ap.add_argument('--timeout', type=float, default=25.0)
    args = ap.parse_args()

    rclpy.init()
    node = Checker()
    node.collect(args.timeout)

    results = []   # (통과여부, 분류, 메시지)

    def add(ok, group, msg):
        results.append((ok, group, msg))

    # 1. 센서 토픽 수신 + frame_id + 점군 크기
    for topic, (_, want_frame, want_points) in SENSORS.items():
        m = node.msgs.get(topic)
        if m is None:
            add(False, '센서', f'{topic:16s} 수신 없음')
            continue
        got = m.header.frame_id
        if got == want_frame:
            add(True, '센서', f'{topic:16s} frame_id={got}')
        elif KNOWN_FRAME_EXCEPTIONS.get(topic) == got:
            add(True, '센서', f'{topic:16s} frame_id={got} '
                              f'(clearpath 제약, 소비자가 override — 알려진 예외)')
        else:
            add(False, '센서', f'{topic:16s} frame_id={got} != 계약 {want_frame}')
        if want_points is not None:
            n = m.width * m.height
            add(n == want_points, '센서',
                f'{topic:16s} 점군 {n}점 (기대 {want_points} = OS1-32)')

    # 2. TF: map까지 이어지는가
    tf_dump = None
    for frame in TF_TARGETS:
        try:
            node.tf_buffer.lookup_transform(
                'map', frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0))
            add(True, 'TF', f'map -> {frame}')
        except Exception as exc:                                  # noqa: BLE001
            add(False, 'TF', f'map -> {frame} 실패: {type(exc).__name__}: {exc}')
            # 실패는 대부분 간헐적이라 사후에 원인을 못 찾는다. 그 순간의
            # 프레임 트리(각 프레임의 부모·발행 주체·최신 시각)를 남겨둔다.
            if tf_dump is None:
                try:
                    tf_dump = node.tf_buffer.all_frames_as_yaml()
                except Exception:                                 # noqa: BLE001
                    tf_dump = '(프레임 트리 덤프 실패)'

    # 3. 위치추정 출력 (GPS 배선)
    for topic in LOCALIZATION:
        m = node.msgs.get(topic)
        add(m is not None, '위치추정',
            f'{topic:20s} ' + ('수신' if m else '수신 없음 (GPS 배선/EKF 확인)'))

    node.destroy_node()
    rclpy.shutdown()

    # ── 출력 ────────────────────────────────────────────────────────
    failed = [r for r in results if not r[0]]
    width = 96
    if not args.quiet:
        print('=' * width)
        print('프레임·토픽 정합성 점검')
        print('=' * width)
        last = None
        for ok, group, msg in results:
            if group != last:
                print(f'\n[{group}]')
                last = group
            print(f'  {"OK  " if ok else "FAIL"} | {msg}')
        print('\n' + '=' * width)
    if tf_dump:
        print('\n[TF 실패 시점의 프레임 트리]')
        print(tf_dump)
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
