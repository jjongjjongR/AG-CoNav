#!/usr/bin/env python3
"""실험 1 시험 1회. 로봇을 +x 로 밀어 경사/단차를 넘게 하고 JSON 을 낸다.

    bench_trial.py --tag slope15_rl_robot_lab_1 --kind slope --value 15 \
                   --controller rl --policy robot_lab --speed 1.0

계측 규약 (앞선 세션에서 하나씩 틀려 본 것들이라 이유를 남긴다)

1. **포즈는 /bench/odom3d 의 정답값을 쓴다.** 런치가 3D OdometryPublisher 를
   따로 끼워 넣어 내는 월드 절대좌표다. 운용 /leg/odom 은 xacro 의 발행기가
   2D 라 z 가 항상 0 이고, EKF 는 이 구성(GPS·Nav2 없음)에서 절대 기준을
   못 잡아 0 만 낸 전례가 있다.
2. **자세는 IMU 로 본다.** odom 의 orientation 은 링크 프레임 오프셋이 섞여
   정상 기립인데도 roll 이 90°/180° 로 나온다.
3. **시간·속도는 전부 /clock(시뮬 시간) 기준.** Go2 + RL 정책이 돌면 RTF 가
   1 밑으로 내려간다. 벽시계로 재면 컨트롤러가 느린 게 아니라 시뮬이 느린
   것을 속도로 착각한다.
4. **넘어짐은 지형 기준 몸통 여유고로 본다.** 25° 경사에서는 pitch 가 원래
   25° 라 절대 기울기만 보면 정상 등판을 전복으로 찍는다. 지형 높이를
   빼고 남은 몸통 높이가 꺼지는지를 본다. 순간 흔들림과 구분하려고 1초
   이상 지속될 때만 인정한다.
"""
import argparse
import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int32
from nav_msgs.msg import Odometry

OBST_START = 3.0        # 장애물 시작 x (make_terrain_worlds.py 와 같아야 한다)
OBST_RUN = 6.0          # 수평 투영 길이
SUMMIT_END = 15.0
FALL_CLEARANCE = 0.12   # 이 아래로 꺼지면 주저앉은 것 (기립 시 0.32)
FALL_HOLD_SEC = 1.0
STAND_MIN_Z = 0.25      # FIXEDSTAND 에서 이 정도는 나와야 선 것으로 본다


def terrain_z(kind, value, x):
    """장애물 윗면 높이. 몸통 여유고를 내려면 이걸 빼야 한다."""
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
    # 경사: 정상부로 1 m 올라서야 "올라탔다"가 아니라 "넘었다"가 된다.
    # 단차: 박스를 완전히 지나 내려서야 통과다(x=9 에서 끝난다).
    return 10.0 if kind == 'slope' else 9.5


class Trial(Node):
    def __init__(self, world):
        super().__init__('bench_trial')
        self.pose = None
        self.tilt = (0.0, 0.0)
        self.sim = None
        self.stage = None
        self.ready = False
        self.cmd = self.create_publisher(Twist, '/leg/cmd_vel', 10)
        self.create_subscription(Odometry, '/bench/odom3d', self._pose, 10)
        self.create_subscription(Clock, '/clock', self._clk, 10)
        self.create_subscription(Int32, '/bench/stage', self._stage, 10)
        # 기립 완료는 단계 번호가 아니라 이 신호로 본다. 단계 표는 컨트롤러
        # 종류에 따라 길이가 달라질 수 있어서 번호로 비교하면 깨진다.
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Bool, '/leg/controller_ready', self._ready, latched)
        q = QoSProfile(depth=10)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Imu, '/leg/imu', self._imu, q)

    def _pose(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def _imu(self, m):
        o = m.orientation
        sinr = 2 * (o.w * o.x + o.y * o.z)
        cosr = 1 - 2 * (o.x * o.x + o.y * o.y)
        sinp = max(-1.0, min(1.0, 2 * (o.w * o.y - o.z * o.x)))
        self.tilt = (math.degrees(math.atan2(sinr, cosr)),
                     math.degrees(math.asin(sinp)))

    def _clk(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def _stage(self, m):
        self.stage = m.data

    def _ready(self, m):
        if m.data:
            self.ready = True

    def spin(self, sim_seconds, drive=None):
        """시뮬 시간 기준으로 돈다. drive 가 있으면 그 속도를 계속 낸다."""
        t0 = self.sim
        while rclpy.ok():
            if drive is not None:
                tw = Twist()
                tw.linear.x = float(drive)
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
    ap.add_argument('--controller', default='rl')
    ap.add_argument('--policy', default='robot_lab')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--world', default='Seongdong_gu')
    ap.add_argument('--settle-sec', type=float, default=3.0)
    ap.add_argument('--drive-sec', type=float, default=60.0)
    ap.add_argument('--stand-timeout', type=float, default=420.0)
    a = ap.parse_args()

    rclpy.init()
    node = Trial(a.world)
    out = {'tag': a.tag, 'kind': a.kind, 'value': a.value,
           'controller': a.controller, 'policy': a.policy, 'speed': a.speed}

    def emit(**kw):
        out.update(kw)
        print(json.dumps(out, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()

    # --- 1. 컨트롤러가 붙고 기립할 때까지 (벽시계로 기다린다. 이 구간은
    #        libtorch 로드 등 시뮬 밖 대기가 대부분이라 시뮬 시간이 안 흐른다)
    import time
    t0 = time.time()
    # 기립 과정의 높이를 남긴다. 기립 실패가 "못 일어선 것"인지 "일어섰다가
    # 무너진 것"인지는 이 궤적을 봐야 갈린다.
    trace = []
    last_t = 0.0
    while rclpy.ok() and time.time() - t0 < a.stand_timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.pose is not None and node.sim is not None and node.sim - last_t >= 0.5:
            last_t = node.sim
            trace.append((round(node.sim, 1), round(node.pose[2], 3),
                          node.stage))
        if node.ready and node.pose is not None:
            break
    if not node.ready:
        emit(verdict='컨트롤러실패', stage=node.stage)
        return
    if node.pose is None:
        emit(verdict='포즈없음')
        return

    # --- 2. 선 자세를 안정시키고 기립 여부를 확인한다
    # (여기서 gz set_pose 로 모델을 다시 놓아 보기도 했다. 서비스는 data:true
    #  를 돌려주는데 모델은 꿈쩍도 안 했다 — 관절이 물린 모델에는 안 먹는다.
    #  대신 스폰 순간부터 관절을 붙잡는 쪽으로 해결했다: 런치가 kp/kd 명령
    #  인터페이스를 시딩하고, 하드웨어가 그 값을 쓰도록 고쳤다.)
    post = []
    lastp = 0.0
    for _ in node.spin(a.settle_sec):
        if node.sim is not None and node.pose is not None and node.sim - lastp >= 0.25:
            lastp = node.sim
            post.append((round(node.sim, 2), round(node.pose[2], 3)))
    x0, y0, z0 = node.pose
    stand_clear = z0 - terrain_z(a.kind, a.value, x0)
    out.update(z_trace=trace[-24:], z_post_reset=post,
               z_stand=round(z0, 3), stand_clearance=round(stand_clear, 3),
               x_start=round(x0, 3),
               stand_tilt=[round(node.tilt[0], 2), round(node.tilt[1], 2)])
    if stand_clear < STAND_MIN_Z:
        emit(verdict='기립X')
        return

    # --- 3. 주행. 넘어짐/도달/시간초과 중 먼저 오는 것으로 끝난다
    target = goal_x(a.kind)
    t_start = node.sim
    x_max = x0
    z_peak = z0
    tilt_max = 0.0
    low_since = None
    verdict = '실패'
    t_reach = None
    for _ in node.spin(a.drive_sec, drive=a.speed):
        if node.pose is None:
            continue
        x, y, z = node.pose
        x_max = max(x_max, x)
        z_peak = max(z_peak, z)
        tilt_max = max(tilt_max, abs(node.tilt[0]), abs(node.tilt[1]))
        clear = z - terrain_z(a.kind, a.value, x)
        # 순간 흔들림과 진짜 주저앉음을 가른다. 한 번 스친 값으로 끊으면
        # 정상 보행 중 접지 순간에도 전복으로 찍힌다.
        if clear < FALL_CLEARANCE:
            if low_since is None:
                low_since = node.sim
            elif node.sim - low_since > FALL_HOLD_SEC:
                verdict = '전복'
                break
        else:
            low_since = None
        if x >= target:
            verdict = '통과'
            t_reach = node.sim - t_start
            break

    x, y, z = node.pose
    elapsed = (node.sim - t_start) if node.sim and t_start else 0.0
    advance = x_max - x0
    emit(verdict=verdict,
         x_end=round(x, 3), y_end=round(y, 3), z_end=round(z, 3),
         x_max=round(x_max, 3), advance=round(advance, 3),
         # 단차는 x=9 에서 끝나므로 6.0 을 넘었으면 올라선 것이다.
         mounted=bool(x_max >= 6.0),
         climb=round(z_peak - z0, 3),
         drift_y=round(abs(y - y0), 3),
         sim_elapsed=round(elapsed, 2),
         t_reach=(round(t_reach, 2) if t_reach else None),
         speed_actual=round(advance / elapsed, 3) if elapsed > 0.5 else 0.0,
         tilt_max=round(tilt_max, 2),
         tilt_end=[round(node.tilt[0], 2), round(node.tilt[1], 2)])


if __name__ == '__main__':
    main()
