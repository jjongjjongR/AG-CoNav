#!/usr/bin/env python3
"""wheel(A300) 지형 주파 시험 1회. +x 로 밀어 경사/단차를 넘게 한다.

    wheel_trial.py --tag slope10 --kind slope --value 10 --speed 0.8 \
                   --world Seongdong_gu --pose-index 3

문서 12(2026-08-16)의 재확인이다. 그때 쓴 스크립트는 리팩터링 때 삭제됐고
원자료(experiments/wheel_terrain/*.json)만 남아 있어 다시 만들었다.

계측 규약 — leg 쪽에서 하나씩 틀려 본 것들을 그대로 따른다.

1. **포즈는 월드 정답값을 PoseArray 로 받는다.** `/wheel/odometry/filtered`
   (모듈 B EKF)는 이 구성(GPS 없음, Nav2 없음)에서 절대 기준을 못 잡아
   **로봇이 43 m 를 갔는데도 계속 0 을 냈다**(문서 12 §1). 월드 포즈를 쓴다.
   `gz topic -e` 텍스트를 파이썬으로 파싱하는 방법은 쓰지 말 것 — 링크가
   많으면 파서가 못 따라가 읽는 값이 몇 초씩 과거가 된다(leg 에서 당했다).
   PoseArray 는 이름이 없으므로 **첨자를 셸에서 미리 찾아 넘긴다.**
2. **시간·속도는 /clock(시뮬 시간) 기준.**
3. **전복은 월드 자세의 roll/pitch 로 본다.** 경사에서 pitch 가 경사각만큼
   나오는 것은 정상이므로, 판정은 지형 대비 초과분으로 한다.
"""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseArray, TwistStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

OBST_START = 3.0
OBST_RUN = 6.0
OBST_HALF_W = 10.0
FLIP_DEG = 45.0         # 지형 기울기를 뺀 초과 기울기가 이만큼이면 전복
STALL_SEC = 12.0
STALL_EPS = 0.15


def terrain_z(kind, value, x, y=0.0):
    if abs(y) > OBST_HALF_W:
        return 0.0
    if kind == 'slope':
        t = math.tan(math.radians(value))
        if x <= OBST_START:
            return 0.0
        if x >= OBST_START + OBST_RUN:
            return OBST_RUN * t
        return (x - OBST_START) * t
    h = value / 1000.0
    return h if OBST_START <= x <= OBST_START + OBST_RUN else 0.0


def goal_x(kind):
    # 문서 12 와 같은 기준: 경사는 정상부 9.5 m, 단차는 너머 7.0 m.
    return 9.5 if kind == 'slope' else 7.0


class WheelTrial(Node):
    def __init__(self, world, idx):
        super().__init__('wheel_trial')
        self.idx = idx
        self.pose = None
        self.rpy = (0.0, 0.0)
        self.sim = None
        # !! 토픽과 타입 둘 다 leg 와 다르다 !!
        #  타입: TwistStamped (leg 의 /leg/cmd_vel 은 평범한 Twist).
        #  토픽: **/wheel/cmd_vel** — twist_mux 의 입구다.
        #
        # 출구인 /wheel/platform/cmd_vel 에 직접 쓰면 안 된다. 거기는
        # twist_mux 가 계속 0 을 쓰고 있어서 내 명령과 번갈아 덮어쓰고,
        # 결과적으로 로봇이 꿈쩍도 안 한다(실측 진출 0.00 m). 발행자 수를
        # 보면 그 토픽에 10개가 물려 있다. 입구로 내야 우선순위 조정을 거쳐
        # 한 줄기로 나간다.
        self.cmd = self.create_publisher(TwistStamped, '/wheel/cmd_vel', 10)
        self.create_subscription(
            PoseArray, '/world/%s/pose/info' % world, self._poses, 10)
        self.create_subscription(Clock, '/clock', self._clk, 10)

    def _poses(self, m):
        if self.idx >= len(m.poses):
            return
        q = m.poses[self.idx]
        p = q.position
        self.pose = (p.x, p.y, p.z)
        o = q.orientation
        sinr = 2.0 * (o.w * o.x + o.y * o.z)
        cosr = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
        sinp = max(-1.0, min(1.0, 2.0 * (o.w * o.y - o.z * o.x)))
        self.rpy = (math.degrees(math.atan2(sinr, cosr)),
                    math.degrees(math.asin(sinp)))

    def _clk(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def spin(self, sim_seconds, drive=None, wall_limit=240.0):
        """시뮬 시간 기준으로 돈다. 벽시계 상한도 함께 둔다.

        !! 벽시계 상한이 없으면 걸린다 !!
        경사에서 바퀴가 헛돌면 접촉 계산이 폭증해 RTF 가 바닥으로 떨어진다.
        그러면 시뮬 45초가 벽시계 600초를 넘어 상위 timeout 에 잘리고,
        결과 파일이 비어 "측정실패" 로만 남는다(실측: 15도 3회 전부).
        상한에 걸리면 그때까지의 진출거리로 판정한다 — 못 넘은 것은 분명하다.
        """
        t0 = self.sim
        w0 = time.time()
        while rclpy.ok():
            if time.time() - w0 > wall_limit:
                return
            if drive is not None:
                tw = TwistStamped()
                tw.header.stamp = self.get_clock().now().to_msg()
                tw.twist.linear.x = float(drive)
                self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.sim is None or t0 is None:
                t0 = self.sim
                continue
            if self.sim - t0 >= sim_seconds:
                return
            yield


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', required=True)
    ap.add_argument('--kind', required=True, choices=['slope', 'step'])
    ap.add_argument('--value', type=float, required=True)
    ap.add_argument('--speed', type=float, default=0.8)
    ap.add_argument('--world', default='Seongdong_gu')
    ap.add_argument('--pose-index', type=int, required=True)
    ap.add_argument('--settle-sec', type=float, default=12.0)
    ap.add_argument('--drive-sec', type=float, default=45.0)
    ap.add_argument('--wait-timeout', type=float, default=300.0)
    a = ap.parse_args()

    rclpy.init()
    node = WheelTrial(a.world, a.pose_index)
    out = {'tag': a.tag, 'kind': a.kind, 'value': a.value, 'speed': a.speed}

    def emit(**kw):
        out.update(kw)
        print(json.dumps(out, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()

    # !! 구동 경로가 열릴 때까지 기다린다 !!
    # 포즈만 보고 출발하면 clearpath 의 속도 컨트롤러가 아직 안 붙어서
    # 명령이 갈 곳이 없다. 그쪽 스폰이 락을 20초씩 여러 번 기다리느라 늦게
    # 뜬다. 실측: 같은 평지 3회가 통과 9.51 / 정지 8.97 / 미동 0.00 으로
    # 갈렸다. cmd_vel 구독자 수가 곧 "받을 사람이 있는가" 다.
    t0 = time.time()
    ready = False
    while rclpy.ok() and time.time() - t0 < a.wait_timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (node.pose is not None and node.sim is not None
                and node.cmd.get_subscription_count() >= 1):
            ready = True
            break
    if node.pose is None:
        emit(verdict='포즈없음')
        return
    if not ready:
        emit(verdict='구동경로없음')
        return

    for _ in node.spin(a.settle_sec):
        pass
    x0, y0, z0 = node.pose
    out.update(x_start=round(x0, 3), z_start=round(z0, 3))

    target = x0 + goal_x(a.kind)
    t_start = node.sim
    x_max, tilt_max = x0, 0.0
    last_gain, best_x = t_start, x0
    verdict = '실패'
    for _ in node.spin(a.drive_sec, drive=a.speed):
        if node.pose is None:
            continue
        x, y, z = node.pose
        x_max = max(x_max, x)
        # 지형이 만드는 기울기를 뺀 초과분으로 전복을 본다.
        base = math.degrees(math.atan(
            math.tan(math.radians(a.value)))) if a.kind == 'slope' else 0.0
        excess = max(abs(node.rpy[0]), abs(abs(node.rpy[1]) - base))
        tilt_max = max(tilt_max, max(abs(node.rpy[0]), abs(node.rpy[1])))
        if excess > FLIP_DEG:
            verdict = '전복'
            break
        if x > best_x + STALL_EPS:
            best_x, last_gain = x, node.sim
        elif node.sim - last_gain > STALL_SEC:
            verdict = '정지'
            break
        if x >= target:
            verdict = '통과'
            break

    x, y, z = node.pose
    elapsed = (node.sim - t_start) if node.sim and t_start else 0.0
    advance = x_max - x0
    emit(verdict=verdict,
         advance=round(advance, 3), need=goal_x(a.kind),
         climb=round(z - z0, 3), drift_y=round(abs(y - y0), 3),
         sim_elapsed=round(elapsed, 2),
         speed_actual=round(advance / elapsed, 3) if elapsed > 0.5 else 0.0,
         tilt_max=round(tilt_max, 2),
         rpy_end=[round(node.rpy[0], 2), round(node.rpy[1], 2)])


if __name__ == '__main__':
    main()
