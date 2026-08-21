#!/usr/bin/env python3
"""실험 1 시험 1회 — 운용 스택과 같은 방식으로 잰다.

운용에서 검증된 go2_rl_spawn.launch.py 가 로봇을 띄우고 FSM 을 몰아 준다.
여기서는 /leg/cmd_vel 로 Twist 만 내고 결과를 읽는다.

측정
  위치  : /leg/odom (gz OdometryPublisher, 절대 좌표. 2D 라 z 는 안 쓴다)
  자세  : /leg/imu (실제 IMU. odom 의 orientation 은 링크 프레임 오프셋이
          섞여 서 있어도 roll 90/180 이 나온다 — 자세 판정에 쓰면 안 된다)
  시간  : /clock (RTF 가 1 밑이면 벽시계로는 속도를 잘못 잰다)

판정
  통과 : 장애 너머까지 진출(경사 x>=10.0 / 단차 x>=9.5), 넘어짐 없음
  넘어짐: |기울기| > 50° 가 0.5초 이상 지속
  기립X : 제한 시간 안에 controller_ready 가 오지 않음
"""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

FALL_DEG = 50.0
FALL_HOLD = 0.5


def rp(q):
    sinr, cosr = 2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    return math.degrees(math.atan2(sinr, cosr)), math.degrees(math.asin(sinp))


class Trial(Node):
    def __init__(self, ns):
        super().__init__('leg_trial2')
        self.xy = None
        self.tilt = None
        self.sim = None
        self.ready = False
        best = QoSProfile(depth=10)
        best.reliability = ReliabilityPolicy.BEST_EFFORT
        lat = QoSProfile(depth=1)
        lat.durability = DurabilityPolicy.TRANSIENT_LOCAL
        lat.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Odometry, f'/{ns}/odom', self._odom, 10)
        self.create_subscription(Imu, f'/{ns}/imu', self._imu, best)
        self.create_subscription(Clock, '/clock', self._clk, 10)
        self.create_subscription(Bool, f'/{ns}/controller_ready',
                                 self._rdy, lat)
        self.cmd = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

    def _odom(self, m):
        p = m.pose.pose.position
        self.xy = (p.x, p.y)

    def _imu(self, m):
        self.tilt = rp(m.orientation)

    def _clk(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def _rdy(self, m):
        self.ready = self.ready or bool(m.data)

    def send(self, vx):
        t = Twist()
        t.linear.x = float(vx)
        self.cmd.publish(t)

    def spin(self, sec):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', required=True)
    ap.add_argument('--kind', required=True, choices=['slope', 'step'])
    ap.add_argument('--value', type=float, required=True)
    ap.add_argument('--controller', default='rl')
    ap.add_argument('--policy', default='robot_lab')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--drive-sec', type=float, default=60.0)
    ap.add_argument('--ns', default='leg')
    a = ap.parse_args()

    rclpy.init()
    n = Trial(a.ns)
    out = {'tag': a.tag, 'kind': a.kind, 'value': a.value,
           'controller': a.controller, 'policy': a.policy, 'speed': a.speed}

    # 기립 완료(controller_ready)까지 대기. 릴레이가 FSM 을 몰아 준다.
    t0 = time.time()
    while rclpy.ok() and not n.ready and time.time() - t0 < 240:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.ready:
        out['verdict'] = '기립X'
        print(json.dumps(out, ensure_ascii=False)); return

    # 운용 릴레이는 첫 주행 명령이 와야 RL 로 들어간다. 살짝 밀어 깨운다.
    t0 = time.time()
    while rclpy.ok() and (n.xy is None or n.sim is None) and time.time() - t0 < 60:
        n.send(0.05)
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.xy is None or n.sim is None:
        out['verdict'] = '토픽없음'
        print(json.dumps(out, ensure_ascii=False)); return
    n.spin(8.0)                      # RL 진입 dwell

    x0, y0 = n.xy
    sim0 = n.sim
    wall0 = time.time()
    max_x, worst, over_since, fell = x0, 0.0, None, False
    while rclpy.ok() and (n.sim - sim0) < a.drive_sec \
            and time.time() - wall0 < a.drive_sec * 15:
        frac = min(1.0, (n.sim - sim0) / 3.0)     # 급가속은 정책을 무너뜨린다
        n.send(a.speed * frac)
        rclpy.spin_once(n, timeout_sec=0.02)
        if n.xy:
            max_x = max(max_x, n.xy[0])
        if n.tilt and (n.sim - sim0) > 1.0:
            t = math.hypot(*n.tilt)
            worst = max(worst, t)
            if t > FALL_DEG:
                over_since = over_since or n.sim
                if n.sim - over_since > FALL_HOLD:
                    fell = True
                    break
            else:
                over_since = None
    n.send(0.0); n.spin(1.0)

    sim = max(n.sim - sim0, 1e-6)
    adv = max_x - x0
    target = 10.0 if a.kind == 'slope' else 9.5
    out.update(verdict='넘어짐' if fell else ('통과' if max_x >= target else '실패'),
               sim_sec=round(sim, 1),
               rtf=round(sim / max(time.time() - wall0, 1e-6), 2),
               x_start=round(x0, 3), x_max=round(max_x, 3),
               advance=round(adv, 3), speed_actual=round(adv / sim, 3),
               tilt_max=round(worst, 1))
    print(json.dumps(out, ensure_ascii=False))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
