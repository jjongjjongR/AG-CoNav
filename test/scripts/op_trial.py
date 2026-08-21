#!/usr/bin/env python3
"""실험 1 시험 1회. 운용 스택이 띄운 Go2 를 +x 로 밀어 경사/단차를 넘게 한다.

    op_trial.py --tag slope15_robot_lab_1 --kind slope --value 15 \
                --policy robot_lab --speed 1.0

기립 FSM 은 **건드리지 않는다.** 운용 cmd_vel_to_control_input 이 몰고,
이 스크립트는 /leg/cmd_vel 에 Twist 만 낸다. 준비 완료는 그 노드가 내는
/leg/controller_ready(latched Bool)로 안다.

계측 규약 (앞선 세션들에서 하나씩 틀려 본 것들이라 이유를 남긴다)

1. **포즈와 자세는 `/leg/odom` 하나로 읽는다.** 운용 xacro 의
   OdometryPublisher 에 `<dimensions>3</dimensions>` 를 넣어 3D 로 만들었다
   (기본값은 2D 라 z 가 항상 0 이었다). Nav2 는 x, y, yaw 만 쓰므로 운용
   동작은 그대로다.

   !! 월드 포즈 스트림을 파이썬으로 파싱하지 말 것 !!
   `gz topic -e` 는 링크 17개의 위치+자세를 물리 주기마다 쏟아내서 파이썬
   파서가 못 따라간다. 파이프가 밀리면 **읽는 값이 몇 초씩 과거가 된다** —
   서 있을 때는 멀쩡해 보이다가 걷기 시작하면 위치가 초당 2~3 m 씩 튀고
   기립 높이(0.32)보다 높은 z 0.46 이 찍힌다. 로봇이 날뛰는 것처럼 보이지만
   계측이 밀린 것이다. 이것 때문에 등판 한계가 10도에서 0도로 잘못 나왔다.
   PoseArray 로 브리지해 첨자로 읽는 방법도 해 봤는데, 첨자를 찾으려면
   결국 텍스트를 한 번 읽어야 하고 그 subprocess 가 스핀을 막아 기립 신호를
   놓쳤다. 3D odom 이 가장 단순하고 확실하다.

2. **자세도 월드 정답 자세로 본다.** IMU(`/leg/imu`)는 이 모델에서 프레임
   오프셋이 섞여 **정상 보행 중에도 roll 이 179도로 나온다**(실측: 평지를
   10 m 걸어 통과한 실행에서 tilt_max 179.5). 그대로 쓰면 자세 비교가
   성립하지 않는다. 월드 자세에서 roll/pitch 를 직접 뽑는다.
   경사에서는 pitch 가 경사각만큼 나오는 것이 **정상**이다. 넘어짐을 보는
   지표는 **roll**(옆으로 기움)과 지형 대비 몸통 여유고다.
3. **시간·속도는 전부 /clock(시뮬 시간) 기준.** Go2 + RL 정책이 돌면 RTF 가
   1 밑으로 내려간다. 벽시계로 재면 컨트롤러가 느린 게 아니라 시뮬이 느린
   것을 속도로 착각한다.
4. **넘어짐은 지형 기준 몸통 여유고로 본다.** 25° 경사에서는 pitch 가 원래
   25° 라 절대 기울기만 보면 정상 등판을 전복으로 찍는다. 지형 높이를 빼고
   남은 몸통 높이가 꺼지는지를 본다. 순간 흔들림과 구분하려고 1초 이상
   지속될 때만 인정한다.
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

OBST_START = 3.0        # 장애물 시작 x (make_terrain_worlds.py 와 같아야 한다)
OBST_RUN = 6.0          # 수평 투영 길이
OBST_HALF_W = 10.0      # 폭 20 m 의 절반. 이 밖은 평지다.

# 단계식 경사로(kind='ramp'). make_ramp_world.py 의 표와 **같은 값**이어야 한다.
RAMP_SEGMENTS = [5, 10, 15, 20, 25, 30]
RAMP_SEG_RUN = 4.0

# 정지 판정. 등판이 목적이라 시간은 넉넉히 주되, 더 못 올라가는 것이
# 확실해지면 즉시 끝낸다. 이게 없으면 멈춘 로봇을 붙잡고 남은 시간을 전부
# 태운다 — 한 번에 7분씩 걸려 스윕이 몇 시간씩 늘어난다.
STALL_SEC = 20.0
STALL_EPS = 0.25        # 이만큼도 못 나아가면 정지로 본다 [m]
# 직진 유지. RL 정책은 직진 명령만 주면 요가 계속 틀어져 8 m 가는 동안 옆으로
# 4~4.8 m 밀린다(실측). 그러면 램프 밖으로 벗어나 등판 능력이 아니라 요 드리프트를
# 재게 된다. 운용에서는 Nav2 가 이 보정을 하므로 같은 역할을 넣는다. 두 컨트롤러에
# 똑같이 적용되므로 비교는 공정하다.
#
# !! 횡방향 위치만 보고 조향하면 안 된다 !!
# wz = -k*y 만 주면 기수각을 직접 잡지 못해 로봇이 선회하며 겉돈다 —
# 실측(15도): 드리프트가 4.8 m 에서 오히려 8.5 m 로 커지고 전진은 3.7 m 에
# 그쳤다. 기수각 항이 주도하고 횡방향 항이 보조해야 선을 탄다.
STEER_YAW = 0.8         # 기수각 오차 [rad] -> yaw rate
STEER_CROSS = 0.20      # 횡방향 오차 [m] -> yaw rate
STEER_LIMIT = 0.35      # rad/s. 세게 주면 정책이 흔들린다
FALL_CLEARANCE = 0.12   # 이 아래로 꺼지면 주저앉은 것 (정상 기립 0.33)
FALL_ROLL_DEG = 60.0    # 옆으로 이만큼 기울면 넘어진 것
FALL_HOLD_SEC = 1.0
# !! 여유고만으로는 부족하다 !!
# 경사에서는 지형 높이가 x 에 따라 빠르게 변해서, 옆으로 굴러도 여유고가
# 문턱을 아슬아슬하게 넘길 수 있다 — 실측: roll 179.9도(뒤집힘)인데 여유고가
# 0.124 라 전복으로 안 잡히고 "정지"로 기록됐다. roll 을 함께 본다.
# pitch 는 경사각만큼 나오는 것이 정상이므로 판정에 쓰지 않는다.
STAND_MIN_Z = 0.25      # 이 정도는 나와야 선 것으로 본다 (RL 기립 0.32 기준)
# !! CHAMP 는 문턱을 낮춰야 한다 !!
# CHAMP 의 nominal_height 는 0.225 라 RL 기준(0.25)으로 재면 정상 기립을
# "기립X" 로 오판한다. 문서 11 에도 같은 함정이 기록돼 있다.
STAND_MIN_Z_CHAMP = 0.18


def ramp_table():
    """[(x_start, x_end, z_start, 각도), ...] 와 정상부 높이."""
    rows, xx, zz = [], OBST_START, 0.0
    for deg in RAMP_SEGMENTS:
        rows.append((xx, xx + RAMP_SEG_RUN, zz, deg))
        xx += RAMP_SEG_RUN
        zz += RAMP_SEG_RUN * math.tan(math.radians(deg))
    return rows, zz


def ramp_angle_at(x):
    """그 x 에서 밟고 있는 경사 각도. 정상부까지 갔으면 마지막 각도."""
    rows, _ = ramp_table()
    if x < OBST_START:
        return 0
    for x0, x1, _z, deg in rows:
        if x < x1:
            return deg
    return RAMP_SEGMENTS[-1]


def terrain_z(kind, value, x, y=0.0):
    """장애물 윗면 높이. 몸통 여유고를 내려면 이걸 빼야 한다.

    폭 밖은 평지다. 이걸 빠뜨리면 옆으로 벗어난 로봇이 평지에 정상적으로
    서 있는데도 "램프 높이만큼 꺼졌다"로 계산돼 전복으로 오판된다.
    """
    if abs(y) > OBST_HALF_W:
        return 0.0
    if kind == 'ramp':
        rows, top = ramp_table()
        if x <= OBST_START:
            return 0.0
        for x0, x1, z0, deg in rows:
            if x < x1:
                return z0 + (x - x0) * math.tan(math.radians(deg))
        return top
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
    # 단계식: 30도 구간까지 다 오르고 정상부에 1 m 올라서면 완주다.
    if kind == 'ramp':
        return OBST_START + RAMP_SEG_RUN * len(RAMP_SEGMENTS) + 1.0
    return 10.0 if kind == 'slope' else 9.5


class Trial(Node):
    def __init__(self, world):
        super().__init__('op_trial')
        self.pose = None
        self.yaw = None
        self.steer = (STEER_YAW, STEER_CROSS, STEER_LIMIT)
        self.rpy = (0.0, 0.0)
        self.tilt = (0.0, 0.0)
        self.sim = None
        self.ready = False
        self.cmd = self.create_publisher(Twist, '/leg/cmd_vel', 10)
        self.create_subscription(Odometry, '/leg/odom', self._odom, 10)
        self.create_subscription(Clock, '/clock', self._clk, 10)
        # 기립 완료 신호. 운용 중계(cmd_vel_to_control_input)가 latched 로 낸다.
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Bool, '/leg/controller_ready', self._ready, latched)
        q = QoSProfile(depth=10)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Imu, '/leg/imu', self._imu, q)

    def _odom(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y, p.z)
        o = m.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                              1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        sinr = 2.0 * (o.w * o.x + o.y * o.z)
        cosr = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
        sinp = max(-1.0, min(1.0, 2.0 * (o.w * o.y - o.z * o.x)))
        self.rpy = (math.degrees(math.atan2(sinr, cosr)),
                    math.degrees(math.asin(sinp)))

    def _imu(self, m):
        o = m.orientation
        sinr = 2 * (o.w * o.x + o.y * o.z)
        cosr = 1 - 2 * (o.x * o.x + o.y * o.y)
        sinp = max(-1.0, min(1.0, 2 * (o.w * o.y - o.z * o.x)))
        self.tilt = (math.degrees(math.atan2(sinr, cosr)),
                     math.degrees(math.asin(sinp)))

    def _clk(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def _ready(self, m):
        if m.data:
            self.ready = True

    def spin(self, sim_seconds, drive=None):
        """시뮬 시간 기준으로 돈다. drive 가 있으면 그 속도를 계속 낸다.

        직진 유지 조향을 함께 낸다(위 STEER_GAIN 주석 참고).
        """
        t0 = self.sim
        while rclpy.ok():
            if drive is not None:
                tw = Twist()
                tw.linear.x = float(drive)
                ky, kc, lim = self.steer
                if (ky or kc) and self.pose is not None and self.yaw is not None:
                    wz = -ky * self.yaw - kc * self.pose[1]
                    tw.angular.z = max(-lim, min(lim, wz))
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
    ap.add_argument('--kind', required=True,
                    choices=['slope', 'step', 'ramp'])
    ap.add_argument('--value', type=float, required=True)
    ap.add_argument('--controller', default='rl')
    ap.add_argument('--policy', default='robot_lab')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--world', default='Seongdong_gu')
    ap.add_argument('--settle-sec', type=float, default=3.0)
    ap.add_argument('--drive-sec', type=float, default=120.0)
    ap.add_argument('--stand-timeout', type=float, default=420.0)
    # CHAMP 는 FSM 이 없어 /leg/controller_ready 를 내지 않는다. 기립 완료를
    # 몸통 높이로 판단한다(CHAMP nominal_height 0.225).
    ap.add_argument('--ready-mode', default='signal',
                    choices=['signal', 'height'])
    ap.add_argument('--ready-z', type=float, default=0.20)
    ap.add_argument('--stand-min-z', type=float, default=None)
    # 조향 이득. 0 이면 조향 없이 직진 명령만 낸다.
    ap.add_argument('--steer-yaw', type=float, default=STEER_YAW)
    ap.add_argument('--steer-cross', type=float, default=STEER_CROSS)
    ap.add_argument('--steer-limit', type=float, default=STEER_LIMIT)
    a = ap.parse_args()

    rclpy.init()
    node = Trial(a.world)
    node.steer = (a.steer_yaw, a.steer_cross, a.steer_limit)
    out = {'tag': a.tag, 'kind': a.kind, 'value': a.value,
           'controller': a.controller, 'policy': a.policy, 'speed': a.speed}

    def emit(**kw):
        out.update(kw)
        print(json.dumps(out, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()

    # --- 1. 운용 중계가 기립을 끝낼 때까지 (벽시계로 기다린다. 이 구간은
    #        libtorch 로드 등 시뮬 밖 대기가 대부분이다)
    t0 = time.time()
    trace, last = [], 0.0
    while rclpy.ok() and time.time() - t0 < a.stand_timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.pose is not None and node.sim is not None and node.sim - last >= 0.5:
            last = node.sim
            trace.append((round(node.sim, 1), round(node.pose[2], 3)))
        if node.pose is not None:
            if a.ready_mode == 'signal' and node.ready:
                break
            if a.ready_mode == 'height' and node.pose[2] >= a.ready_z:
                # 높이만 보면 스폰 직후 낙하 중에도 조건이 맞을 수 있다.
                # 잠깐 유지되는지 확인한다.
                if node.sim is not None and node.sim - (trace[0][0] if trace else 0) > 5.0:
                    break
    if a.ready_mode == 'signal' and not node.ready:
        emit(verdict='컨트롤러실패', z_trace=trace[-20:])
        return
    if node.pose is None:
        emit(verdict='포즈없음')
        return

    for _ in node.spin(a.settle_sec):
        pass
    x0, y0, z0 = node.pose
    stand_clear = z0 - terrain_z(a.kind, a.value, x0, y0)
    out.update(z_trace=trace[-20:], z_stand=round(z0, 3),
               stand_clearance=round(stand_clear, 3), x_start=round(x0, 3),
               stand_rpy=[round(node.rpy[0], 2), round(node.rpy[1], 2)])
    min_z = a.stand_min_z if a.stand_min_z is not None else (
        STAND_MIN_Z_CHAMP if a.controller == 'champ' else STAND_MIN_Z)
    if stand_clear < min_z:
        emit(verdict='기립X')
        return

    # --- 2. 주행. 넘어짐/도달/시간초과 중 먼저 오는 것으로 끝난다
    target = goal_x(a.kind)
    t_start = node.sim
    x_max, z_peak = x0, z0
    roll_max = pitch_max = 0.0
    clear_min = 9.9
    low_since, t_reach = None, None
    last_gain, best_x = t_start, x0
    verdict = '실패'
    drive_trace, last_d = [], 0.0
    for _ in node.spin(a.drive_sec, drive=a.speed):
        if node.pose is None:
            continue
        x, y, z = node.pose
        if node.sim is not None and node.sim - last_d >= 1.0:
            last_d = node.sim
            drive_trace.append((round(node.sim, 1), round(x, 2), round(y, 2),
                                round(z, 2), round(node.rpy[0], 1),
                                round(node.rpy[1], 1),
                                round(math.degrees(node.yaw or 0.0), 1)))
        x_max = max(x_max, x)
        z_peak = max(z_peak, z)
        roll_max = max(roll_max, abs(node.rpy[0]))
        pitch_max = max(pitch_max, abs(node.rpy[1]))
        clear = z - terrain_z(a.kind, a.value, x, y)
        clear_min = min(clear_min, clear)
        # 순간 흔들림과 진짜 주저앉음을 가른다. 한 번 스친 값으로 끊으면
        # 정상 보행 중 접지 순간에도 전복으로 찍힌다.
        if clear < FALL_CLEARANCE or abs(node.rpy[0]) > FALL_ROLL_DEG:
            if low_since is None:
                low_since = node.sim
            elif node.sim - low_since > FALL_HOLD_SEC:
                verdict = '전복'
                break
        else:
            low_since = None
        # 더 못 올라가면 즉시 끝낸다. 등판이 목적이라 "얼마나 높이"가
        # 중요하지 "얼마나 오래 버텼나"는 의미가 없다.
        if x > best_x + STALL_EPS:
            best_x, last_gain = x, node.sim
        elif node.sim - last_gain > STALL_SEC:
            verdict = '정지'
            break
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
         climb=round(z_peak - z0, 3), drift_y=round(abs(y - y0), 3),
         sim_elapsed=round(elapsed, 2),
         t_reach=(round(t_reach, 2) if t_reach else None),
         speed_actual=round(advance / elapsed, 3) if elapsed > 0.5 else 0.0,
         # 자세 지표. 경사에서 pitch 가 경사각만큼 나오는 것은 정상이고,
         # 안정성을 보는 것은 roll(옆으로 기움)과 몸통 여유고다.
         drive_trace=drive_trace[:40],
         roll_max=round(roll_max, 2), pitch_max=round(pitch_max, 2),
         clear_min=round(clear_min, 3),
         # 등판 지표. 이 실험의 판정 기준이다.
         max_angle=(ramp_angle_at(x_max) if a.kind == 'ramp' else int(a.value)),
         rpy_end=[round(node.rpy[0], 2), round(node.rpy[1], 2)])


if __name__ == '__main__':
    main()
