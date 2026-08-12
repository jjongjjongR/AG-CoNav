#!/usr/bin/env python3
"""모듈 F 판정을 관문으로 삼아 Nav2 목표를 보낸다 (C -> D -> E 사슬 착화).

두 가지 일을 한다.

1) 관문 — 모듈 F의 /X/nav_map을 받아 "이 판정으로 주행을 시켜도 되는지" 검사한다.
   기준에 미달하면 목표를 보내지 않고 실패로 끝낸다. 실험을 여기서 멈추기
   위해서다. F가 쓸모없는 지도를 냈는데도 Nav2가 어떻게든 길을 내면, 파이프라인
   전체가 "통과"로 보이면서 실제로는 아무것도 검증하지 못한다.

2) 목표 전송 — 통과하면 실제로 '통과 가능'으로 판정된 칸 중에서 목표를 골라
   navigate_to_pose로 보낸다. 모듈 C의 navigation_complete_node가 그 액션 상태를
   보고 /X/navigation_status를 내고, 그게 있어야 모듈 D가 지도를 발행하고
   모듈 E가 병합한다. 지금까지 이 한 칸이 비어 있어서 D와 E가 조용했다.

/X/nav_map은 grid_map이 아니라 nav_msgs/OccupancyGrid다
(traversability_verdictor.py: FREE=0, OCCUPIED=100, 미관측=-1).
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

FREE, OCCUPIED, UNKNOWN = 0, 100, -1

# 모듈 F가 쓰는 QoS와 같아야 받는다 (MAP_QOS).
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


class GateAndGoals(Node):
    def __init__(self, robots, args):
        super().__init__('send_nav_goals')
        self.robots = robots
        self.a = args
        self.maps = {}
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self, spin_thread=True)
        for r in robots:
            self.create_subscription(
                OccupancyGrid, f'/{r}/nav_map',
                lambda m, r=r: self.maps.setdefault(r, m), MAP_QOS)

    # ── 수집 ──────────────────────────────────────────────────────────
    def wait_for_maps(self, timeout_s):
        end = self.get_clock().now() + Duration(seconds=timeout_s)
        while len(self.maps) < len(self.robots) and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
        return len(self.maps) == len(self.robots)

    def robot_xy(self, robot, timeout_s=15.0):
        end = self.get_clock().now() + Duration(seconds=timeout_s)
        while self.get_clock().now() < end:
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', f'{robot}/base_link', rclpy.time.Time())
                return t.transform.translation.x, t.transform.translation.y
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.3)
        return None

    # ── 관문 ──────────────────────────────────────────────────────────
    def inspect(self, robot):
        """지도를 수치로 풀어놓는다. 판정은 gate()가 한다."""
        m = self.maps[robot]
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        n_free = int((g == FREE).sum())
        n_occ = int((g == OCCUPIED).sum())
        n_unknown = int((g == UNKNOWN).sum())
        known = n_free + n_occ
        return {
            'grid': g, 'info': m.info,
            'cells': g.size, 'free': n_free, 'occupied': n_occ,
            'unknown': n_unknown, 'known': known,
            'known_ratio': known / g.size if g.size else 0.0,
            'free_of_known': n_free / known if known else 0.0,
            'free_area_m2': n_free * m.info.resolution ** 2,
        }

    def free_cells_xy(self, robot, st):
        """'통과 가능' 칸의 world 좌표. OccupancyGrid는 행=y, 열=x, 원점은 최소 모서리."""
        rows, cols = np.nonzero(st['grid'] == FREE)
        res = st['info'].resolution
        ox = st['info'].origin.position.x
        oy = st['info'].origin.position.y
        X = ox + (cols + 0.5) * res
        Y = oy + (rows + 0.5) * res
        return X, Y

    def gate(self, robot, st, rx, ry):
        """통과 기준. 하나라도 어기면 실험을 멈춘다."""
        fails = []
        if st['known_ratio'] < self.a.min_known_ratio:
            fails.append(
                f"관측된 칸이 {st['known_ratio']*100:.1f}% 뿐 "
                f"(기준 {self.a.min_known_ratio*100:.0f}% 이상) "
                f"— 드론이 지도를 제대로 못 만들었다")
        if st['free_area_m2'] < self.a.min_free_area_m2:
            fails.append(
                f"통과 가능 면적이 {st['free_area_m2']:.1f} m² 뿐 "
                f"(기준 {self.a.min_free_area_m2:.0f} m² 이상) "
                f"— 주행할 땅이 없다")
        X, Y = self.free_cells_xy(robot, st)
        if len(X) == 0:
            fails.append("통과 가능 칸이 하나도 없다")
            return fails, None
        d = np.hypot(X - rx, Y - ry)
        band = (d >= self.a.min_distance) & (d <= self.a.max_distance)
        if not band.any():
            fails.append(
                f"로봇 {self.a.min_distance:.0f}~{self.a.max_distance:.0f} m 안에 "
                f"통과 가능 칸이 없다 (가장 가까운 것이 {d.min():.1f} m) "
                f"— 로봇이 판정된 영역 밖에 서 있다")
            return fails, None
        k = int(np.argmin(np.abs(d[band] - self.a.distance)))
        goal = (float(X[band][k]), float(Y[band][k]), float(d[band][k]))
        return fails, goal

    # ── 전송 ──────────────────────────────────────────────────────────
    def send(self, robot, gx, gy, rx, ry):
        client = ActionClient(self, NavigateToPose, f'/{robot}/navigate_to_pose')
        if not client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(f'{robot}: navigate_to_pose 서버가 없다')
            return None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        yaw = math.atan2(gy - ry, gx - rx)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{robot}: 목표가 거부됐다')
            return None
        return handle


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robots', default='wheel,leg')
    ap.add_argument('--distance', type=float, default=8.0,
                    help='목표까지 원하는 거리 (m)')
    ap.add_argument('--min-distance', type=float, default=3.0)
    ap.add_argument('--max-distance', type=float, default=25.0)
    ap.add_argument('--map-timeout', type=float, default=180.0)
    ap.add_argument('--nav-timeout', type=float, default=300.0)
    # 관문 기준
    ap.add_argument('--min-known-ratio', type=float, default=0.05,
                    help='격자 중 관측된(통과+불가) 칸의 최소 비율')
    ap.add_argument('--min-free-area-m2', type=float, default=50.0,
                    help='통과 가능 판정 면적의 최소값')
    args = ap.parse_args()

    robots = [r for r in args.robots.split(',') if r]
    rclpy.init()
    node = GateAndGoals(robots, args)

    print(f'[관문] 모듈 F의 주행 가능 맵 대기 (최대 {args.map_timeout:.0f}초)...')
    if not node.wait_for_maps(args.map_timeout):
        missing = [r for r in robots if r not in node.maps]
        print(f'\n실험 중단: 모듈 F가 {missing} 의 nav_map을 내지 않았다.')
        print('  모듈 A가 지도를 발행했는지, 모듈 F가 특성 계산을 마쳤는지 확인할 것.')
        node.destroy_node(); rclpy.shutdown(); return 2

    # 1단계: 관문
    print('\n[관문] 모듈 F 판정 검사')
    blocked = False
    plan = {}
    for r in robots:
        st = node.inspect(r)
        xy = node.robot_xy(r)
        print(f'\n  [{r}] 격자 {st["info"].width}x{st["info"].height} '
              f'@ {st["info"].resolution:.2f} m')
        print(f'       통과 {st["free"]:,} / 불가 {st["occupied"]:,} / '
              f'미관측 {st["unknown"]:,}')
        print(f'       관측 비율 {st["known_ratio"]*100:.1f}%, '
              f'관측 칸 중 통과 {st["free_of_known"]*100:.1f}%, '
              f'통과 면적 {st["free_area_m2"]:.1f} m²')
        if xy is None:
            print(f'       기준 미달: {r}/base_link TF가 없다 (모듈 B 실패)')
            blocked = True
            continue
        rx, ry = xy
        print(f'       로봇 위치 ({rx:.2f}, {ry:.2f})')
        fails, goal = node.gate(r, st, rx, ry)
        for f in fails:
            print(f'       기준 미달: {f}')
        if fails:
            blocked = True
        else:
            gx, gy, dist = goal
            print(f'       통과 → 목표 ({gx:.2f}, {gy:.2f}), 거리 {dist:.2f} m')
            plan[r] = (gx, gy, rx, ry)

    if blocked:
        print('\n실험 중단: 모듈 F 판정이 기준에 미달했다. '
              'Nav2 목표를 보내지 않는다.')
        print('  주행성이 성립하지 않는 지도로 이동시키면 뒤따르는 D·E 결과가 '
              '의미를 잃으므로 여기서 멈춘다.')
        node.destroy_node(); rclpy.shutdown(); return 1

    # 2단계: 전송
    print('\n[전송] Nav2 목표 발행')
    handles = {}
    for r, (gx, gy, rx, ry) in plan.items():
        h = node.send(r, gx, gy, rx, ry)
        if h is None:
            print(f'  {r}: 전송 실패')
        else:
            print(f'  {r}: 목표 수락 ({gx:.2f}, {gy:.2f})')
            handles[r] = h
    if not handles:
        print('\n실험 중단: 목표를 하나도 못 보냈다.')
        node.destroy_node(); rclpy.shutdown(); return 1

    print(f'\n[대기] 이동 완료 (최대 {args.nav_timeout:.0f}초)...')
    results = {}
    futs = {r: h.get_result_async() for r, h in handles.items()}
    end = node.get_clock().now() + Duration(seconds=args.nav_timeout)
    while len(results) < len(futs) and node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.5)
        for r, f in futs.items():
            if r not in results and f.done():
                st = f.result().status
                ok = st == GoalStatus.STATUS_SUCCEEDED
                results[r] = ok
                print(f'  {r}: {"성공" if ok else f"실패(status={st})"}')
    for r in futs:
        if r not in results:
            print(f'  {r}: 시간 초과')
            results[r] = False

    node.destroy_node()
    rclpy.shutdown()
    if not any(results.values()):
        print('\n실험 중단: 어느 로봇도 목표에 도달하지 못했다. '
              'allow_unknown:false 이므로 경로는 모듈 F가 통과로 판정한 칸으로만 난다 '
              '— 계획 실패는 판정 영역이 끊겨 있다는 뜻이다.')
        return 1
    print('\n이동 완료 → /X/navigation_status → 모듈 D 지도 발행 → 모듈 E 병합')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
