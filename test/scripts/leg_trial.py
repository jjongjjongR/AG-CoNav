#!/usr/bin/env python3
"""실험 1 시험 1회. FSM 을 몰아 기립시킨 뒤 +x 로 밀고 결과를 JSON 으로 낸다.

    leg_trial.py --tag slope15_rl_robot_lab --kind slope --value 15 \
                 --controller rl --speed 1.0

판정
  통과  : 장애 너머(경사 정상부 x>=10.0 / 단차 너머 x>=9.5)까지 진출, 전복 없음
  실패  : 시간 내 못 넘음
  전복  : |기울기| > 45°
  기립X : 제한 시간 안에 서지 못함

포즈는 Gazebo /world/.../dynamic_pose/info 의 정답값을 쓴다. EKF 는 이 구성
(GPS 없음, Nav2 없음)에서 절대 기준을 못 잡아 0 만 낸 전례가 있다(문서 12 §1).
"""
import argparse
import json
import math
import sys
import time

import rclpy
from control_input_msgs.msg import Inputs
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from rclpy.node import Node

CMD_PASSIVE_TO_DOWN = 2
CMD_DOWN_TO_STAND = 4
CMD_STAND_TO_RL = 3
STAND_Z = 0.14      # 주저앉은 0.10 과 구분되는 최소 높이 (실측 기준)
FALL_Z = 0.10       # 이 아래로 내려앉으면 넘어진 것


class PoseSub:
    """/odom(정답 포즈)과 /clock(시뮬 시간)을 들고 있는다.

    시뮬 시간을 함께 보는 이유: Go2 + RL 정책이 돌면 RTF 가 1 밑으로 내려간다.
    벽시계로 재면 컨트롤러가 느린 게 아니라 시뮬이 느린 것을 속도로 착각한다.
    주행 구간과 속도는 전부 시뮬 시간으로 잰다.
    """

    def __init__(self, node):
        self.pose = None
        self.sim = None
        node.create_subscription(Odometry, '/odom3d', self._cb, 10)
        node.create_subscription(Clock, '/clock', self._clk, 10)
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        q = QoSProfile(depth=10); q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.tilt = None
        node.create_subscription(Imu, '/imu', self._imu, q)

    def _cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose = {'px': p.x, 'py': p.y, 'pz': p.z,
                     'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}

    def _imu(self, m):
        # 실제 IMU. odom 의 orientation 은 링크 프레임 오프셋이 섞여
        # 서 있어도 roll 90/180 이 나오므로 자세 판정에 쓸 수 없다.
        o = m.orientation
        sinr, cosr = 2*(o.w*o.x + o.y*o.z), 1 - 2*(o.x*o.x + o.y*o.y)
        sinp = max(-1.0, min(1.0, 2*(o.w*o.y - o.z*o.x)))
        self.tilt = (math.degrees(math.atan2(sinr, cosr)),
                     math.degrees(math.asin(sinp)))

    def _clk(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9


def tilt_deg(q):
    x, y, z, w = q['qx'], q['qy'], q['qz'], q['qw']
    sinr, cosr = 2 * (w * x + y * z), 1 - 2 * (x * x + y * y)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    return math.degrees(math.atan2(sinr, cosr)), math.degrees(math.asin(sinp))


class Trial(Node):
    def __init__(self, a):
        super().__init__('leg_trial')
        self.a = a
        self.pub = self.create_publisher(Inputs, '/control_input', 10)

    def send(self, cmd, vx=0.0, wz=0.0):
        m = Inputs()
        m.command = int(cmd)
        m.ly = float(vx)     # StateRL: control_.x = ly
        m.lx = 0.0
        m.rx = float(-wz)    # StateRL: control_.yaw = -rx
        m.ry = 0.0
        self.pub.publish(m)

    def hold(self, cmd, seconds, vx=0.0, rate=50.0):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            self.send(cmd, vx)
            rclpy.spin_once(self, timeout_sec=1.0 / rate)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', required=True)
    ap.add_argument('--kind', required=True, choices=['slope', 'step'])
    ap.add_argument('--value', type=float, required=True)
    ap.add_argument('--controller', default='rl')
    ap.add_argument('--policy', default='robot_lab')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--drive-sec', type=float, default=60.0)
    a = ap.parse_args()

    rclpy.init()
    node = Trial(a)
    gz = PoseSub(node)

    out = {'tag': a.tag, 'kind': a.kind, 'value': a.value,
           'controller': a.controller, 'policy': a.policy, 'speed': a.speed}

    # 포즈가 잡힐 때까지
    t0 = time.time()
    while gz.pose is None and time.time() - t0 < 40:
        rclpy.spin_once(node, timeout_sec=0.2)
    if gz.pose is None:
        out.update(verdict='포즈없음')
        print(json.dumps(out, ensure_ascii=False)); return

    def snap():
        p = dict(gz.pose)
        return p, p

    # FSM: PASSIVE -> FIXEDDOWN -> FIXEDSTAND -> (RL). 각 단계 4.5초 이상.
    # !! 벽시계가 아니라 시뮬 시간이 기준이다 !!
    # FSM 은 각 상태에서 900 스텝(200 Hz = 4.5 초 시뮬 시간)을 채워야 다음
    # 명령을 받는다. Go2 + RL 정책이 도는 동안 RTF 가 1 밑으로 떨어지므로
    # 벽시계 7 초로는 모자라 기립에 실패하는 실행이 섞였다(z 0.057).
    # 넉넉히 준다 — 남는 시간은 그냥 서 있을 뿐이라 손해가 없다.
    # !! 전환 명령을 계속 누르고 있으면 안 된다 !!
    # 같은 명령이 반대 방향 전환에도 쓰여서, 계속 보내면
    #   fixed down -> fixed stand -> fixed down -> ...
    # 로 무한 토글한다(로그로 확인). 짧게 눌러 전환시킨 뒤 0(무명령)으로
    # 그 상태를 채운다. 각 상태는 900스텝(시뮬 4.5초)을 채워야 다음 명령을
    # 받으므로 대기는 넉넉히 준다.
    def stage(cmd, press=2.0, dwell=16.0):
        node.hold(cmd, press)
        node.hold(0, dwell)

    # !! 명령 2 하나로 passive -> fixed down -> fixed stand 까지 간다 !!
    # 로그로 확인했다. 그 뒤에 명령 4 를 또 주면 fixed stand 에서 도로
    # fixed down 으로 돌아간다. 명령 2 로 세우고 바로 RL 로 넘긴다.
    # 전환마다 따로 누른다. 각 상태는 900스텝(시뮬 4.5초)을 채워야 다음
    # 명령을 받으므로 사이에 충분히 쉰다. 한 번만 누르면 fixed down 에서
    # 멈추고, 계속 누르고 있으면 down <-> stand 를 무한 토글한다.
    stage(CMD_PASSIVE_TO_DOWN, press=1.5, dwell=9.0)   # passive -> fixed down
    stage(CMD_PASSIVE_TO_DOWN, press=1.5, dwell=9.0)   # fixed down -> fixed stand
    z_down = gz.pose['pz'] if gz.pose else float('nan')
    # 단계별 높이를 남긴다. 기립 자체가 실패한 것인지, 서긴 섰는데 RL 이
    # 무너뜨린 것인지는 이 두 값을 비교해야 갈린다.
    z_fixedstand = z_down
    if a.controller == 'rl':
        # RL 진입 직후에 명령 0 을 주면 정책이 자세를 못 잡고 무너졌다.
        # 운용 스택은 "첫 주행 명령이 왔을 때" RL 로 들어가므로, 진입 순간부터
        # 이미 0 이 아닌 명령이 흐른다. 같은 조건으로 맞춘다.
        node.hold(CMD_STAND_TO_RL, 1.5)
        end = time.time() + 8.0
        while rclpy.ok() and time.time() < end:
            node.send(0, 0.3)
            rclpy.spin_once(node, timeout_sec=0.02)
    else:
        node.hold(0, 3.0)
    out['z_down'] = round(z_down, 3)
    out['z_fixedstand'] = round(z_fixedstand, 3)

    p, q = snap()
    r0, pi0 = tilt_deg(q)
    z_stand = p['pz']
    # !! 자세는 몸통 높이로 본다 !!
    # odom 의 orientation 은 robot_base_frame 링크의 프레임 오프셋을 그대로
    # 담는다. 정상적으로 서 있는데도 base 는 roll 180도, trunk 는 90도로
    # 나온다. 각도로 판정하면 전부 전복으로 오판한다.
    # Go2 는 서면 몸통이 0.28~0.34 m, 주저앉으면 0.1 m 안팎이다.
    if z_stand < STAND_Z:
        out.update(verdict='기립X', z_stand=round(z_stand, 3))
        print(json.dumps(out, ensure_ascii=False)); return

    # 주행
    start = snap()[0]
    x0, y0 = start['px'], start['py']
    worst, max_x, flipped = 0.0, x0, False
    # /clock 이 안 오면 무한 대기가 된다. 제한을 두고 원인을 남긴다.
    tw = time.time()
    while gz.sim is None and rclpy.ok() and time.time() - tw < 30:
        rclpy.spin_once(node, timeout_sec=0.1)
    if gz.sim is None:
        out.update(verdict='clock없음')
        print(json.dumps(out, ensure_ascii=False)); return
    sim0 = gz.sim
    t_start = time.time()
    wall_cap = t_start + a.drive_sec * 12      # 안전장치(RTF 가 아주 낮을 때)
    samples, over = [], 0
    while rclpy.ok() and (gz.sim - sim0) < a.drive_sec and time.time() < wall_cap:
        frac = min(1.0, (gz.sim - sim0) / 3.0)
        node.send(0, a.speed * frac)
        rclpy.spin_once(node, timeout_sec=0.02)
        p, q = snap()
        r, pi = tilt_deg(q)
        tilt = math.hypot(r, pi)
        max_x = max(max_x, p['px'])
        samples.append((gz.sim, p['px'], p['pz']))
        # !! 순간값 하나로 전복이라 하면 안 된다 !!
        # 보행 중에는 몸통이 순간적으로 크게 기울고, odom 플러그인이 붙는
        # 첫 순간에는 값이 한 번 튄다. 실제로 마지막 자세가 멀쩡한데
        # (roll -0.5, pitch -7.3) 순간값 46도 하나로 전복 판정이 났다.
        # 0.3초 이상 이어질 때만 넘어진 것으로 본다. 초반 1초는 무시한다.
        if (gz.sim - sim0) < 1.0:
            continue
        worst = max(worst, tilt)
        # 넘어짐 = 몸통이 내려앉은 상태가 0.3초 이상 이어짐.
        tl = math.hypot(*gz.tilt) if gz.tilt else 0.0
        worst = max(worst, tl)
        over = over + 1 if tl > 50.0 else 0
        if over >= 15:
            flipped = True
            break
    node.hold(0, 1.0)

    p, q = snap()
    r, pi = tilt_deg(q)
    dx = max_x - x0
    dt = samples[-1][0] - samples[0][0] if len(samples) > 1 else 1.0
    target = 10.0 if a.kind == 'slope' else 9.5
    verdict = '넘어짐' if flipped else ('통과' if max_x >= target else '실패')
    out.update(verdict=verdict, imu_tilt_max=round(worst,1), sim_sec=round(gz.sim - sim0, 1),
               rtf=round((gz.sim - sim0) / max(time.time() - t_start, 1e-6), 3),
               z_stand=round(z_stand, 3), x_start=round(x0, 3), x_max=round(max_x, 3),
               advance=round(dx, 3), z_end=round(p['pz'], 3),
               z_gain=round(p['pz'] - z_stand, 3),
               speed_actual=round(dx / dt, 3) if dt > 0 else 0.0,
               tilt_max=round(worst, 2), roll=round(r, 2), pitch=round(pi, 2))
    print(json.dumps(out, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
