#!/usr/bin/env python3
"""실험 1용 FSM 구동 + cmd_vel 중계. 운용 노드(cmd_vel_to_control_input)와
같은 전이를 쓰되, 컨트롤러 두 종류를 몰 수 있고 기립 시점을 실제 로봇 상태로
동기화한다.

    /leg/cmd_vel (Twist) -> /control_input (control_input_msgs/Inputs)

## 전이 규칙 (소스에서 확인. 2차 세션 기록은 틀렸다)

    PASSIVE --(2)--> FIXEDDOWN --(2 또는 4)--> FIXEDSTAND --(walk_command)--> 보행
    StatePassive.cpp / StateFixedDown.cpp / 각 컨트롤러의 StateFixedStand.cpp

**관문은 1.8초다.** `duration_ = frequency_ * 1.2` 이고 go2 는 200 Hz 라
240스텝, 관문 `percent_ > 1.5` 는 360스텝 = 1.8초. 그전에 온 명령은 조용히
무시된다. 예전 주석의 "900스텝/4.5초"는 틀렸다.

## 왜 시간이 아니라 로봇 상태로 동기화하나

스폰부터 컨트롤러가 붙을 때까지는 하드웨어 시딩(kp 400)이 로봇을 기립
자세로 붙잡는다(실측 z 0.329). 그런데 컨트롤러가 활성화되는 순간 PASSIVE 로
들어가며 kp 를 0 으로 만들어(StatePassive::enter) 그대로 주저앉는다.
그 순간에 명령 2 가 이미 도착해 있어야 다음 주기에 FIXEDDOWN 이 kp 를
다시 건다. 그래서 **처음부터 조건 없이 명령 2 를 계속 낸다.**

그러면 "언제부터 단계를 세기 시작할 것인가"가 문제가 된다. 구독자가 생긴
시점을 쓰면 안 된다 — 구독은 on_configure 에서 만들어지는데, 그 직후
on_activate 안에서 `policy.pt`(libtorch) 를 읽느라 수 초가 더 걸린다.
그동안 컨트롤러의 update 는 돌지 않는다. 구독 시점부터 2초를 세면 컨트롤러가
일을 시작하기도 전에 명령 2 구간이 끝나 버리고, 뒤늦게 FIXEDDOWN 에 들어간
뒤로는 관문을 열어 줄 명령이 없어 **웅크린 채로 영영 멈춘다**
(실측: z 0.123 에서 고정).

그래서 **몸통 높이가 실제로 내려앉는 것**을 컨트롤러가 일을 시작한 신호로
쓴다. 그 뒤에 명령 2 를 관문 너머까지 유지해 FIXEDSTAND 로 넘긴다.

## 컨트롤러별로 다른 두 가지

| | rl_quadruped_controller | unitree_guide_controller |
|---|---|---|
| FIXEDSTAND 에서 보행 진입 | 3 (RL) | 4 (TROTTING) |
| ly 축의 의미 | m/s 그대로 | [-1,1] 정규화 (invNormalize) |

guide 의 StateTrotting 은 `v_cmd = invNormalize(ly, -0.4, 0.4)` 라 ly 를
조이스틱 축으로 읽는다. m/s 를 그대로 넣으면 1.0 이 곧 최대치(0.4 m/s)로
잘려 두 컨트롤러를 같은 명령으로 비교할 수 없다. norm_linear 로 나눠 단위를
맞춘다(guide 0.4, rl 1.0).
"""
import rclpy
from control_input_msgs.msg import Inputs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32

CMD_PASSIVE_TO_DOWN = 2
CMD_DOWN_TO_STAND = 4

TAKEOVER_Z = 0.30     # 이 아래로 내려앉으면 컨트롤러가 관절을 잡기 시작한 것 (기립 0.33, 웅크림 0.25)
HOLD_DOWN_SEC = 2.4   # 인수 확인 후 명령 2 유지 (관문 1.8초를 넘겨야 한다)
SETTLE_SEC = 2.5      # FIXEDSTAND 로 완전히 일어설 시간
WALK_HOLD_SEC = 1.0   # 보행 상태로 넘기는 펄스
TAKEOVER_TIMEOUT = 90.0   # 높이 신호를 못 받을 때의 탈출구 [s]

# 단계 번호 (/bench/stage 로 내보낸다)
S_WAIT_TAKEOVER = 0
S_HOLD_DOWN = 1
S_SETTLE = 2
S_READY = 3
S_TO_WALK = 4
S_RELAY = 5


class BenchRelay(Node):
    def __init__(self):
        super().__init__('bench_relay')
        self.declare_parameter('cmd_vel_topic', '/leg/cmd_vel')
        self.declare_parameter('output_topic', '/control_input')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 1.0)
        self.declare_parameter('max_linear', 1.5)
        self.declare_parameter('max_angular', 1.0)
        # FIXEDSTAND -> 보행 상태로 넘어가는 명령. rl 3 / guide 4.
        self.declare_parameter('walk_command', 3)
        # ly 축 1.0 이 몇 m/s 인가. rl 1.0(그대로) / guide 0.4(정규화 축).
        self.declare_parameter('norm_linear', 1.0)
        self.declare_parameter('norm_angular', 1.0)

        g = lambda n: self.get_parameter(n).value
        self._rate = float(g('publish_rate_hz'))
        self._timeout = float(g('cmd_timeout_sec'))
        self._max_lin = float(g('max_linear'))
        self._max_ang = float(g('max_angular'))
        self._walk_cmd = int(g('walk_command'))
        self._nl = float(g('norm_linear'))
        self._na = float(g('norm_angular'))

        self._out = self.create_publisher(
            Inputs, self.get_parameter('output_topic').value, 10)
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self._ready_out = self.create_publisher(Bool, '/leg/controller_ready', latched)
        self._ready_out.publish(Bool(data=False))
        self._stage_out = self.create_publisher(Int32, '/bench/stage', latched)
        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self._on_cmd, 10)
        self.create_subscription(Odometry, '/bench/odom3d', self._on_odom, 10)

        self._z = None
        self._vx = self._vy = self._wz = 0.0
        self._last_cmd = None
        self._stage = S_WAIT_TAKEOVER
        self._count = 0
        self._ready_sent = False
        self._publish_stage()
        self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            'bench_relay 시작 (보행명령 %d, 축 1.0 = %.2f m/s)'
            % (self._walk_cmd, self._nl))

    def _publish_stage(self):
        self._stage_out.publish(Int32(data=self._stage))

    def _on_odom(self, m):
        self._z = m.pose.pose.position.z

    def _on_cmd(self, msg):
        clamp = lambda v, m: max(-m, min(m, v))
        self._vx = clamp(msg.linear.x, self._max_lin)
        self._vy = clamp(msg.linear.y, self._max_lin)
        self._wz = clamp(msg.angular.z, self._max_ang)
        self._last_cmd = self.get_clock().now()

    def _send(self, command, vx=0.0, vy=0.0, wz=0.0):
        m = Inputs()
        m.command = int(command)
        # 축 변환은 두 컨트롤러가 같다(StateRL.cpp 348-350 / StateTrotting.cpp 88-93).
        m.ly = float(vx / self._nl)
        m.lx = float(-vy / self._nl)
        m.rx = float(-wz / self._na)
        m.ry = 0.0
        self._out.publish(m)

    def _advance(self, nxt, msg=None):
        self._stage = nxt
        self._count = 0
        self._publish_stage()
        if msg:
            self.get_logger().info(msg)

    def _ticks(self, seconds):
        return max(1, int(round(seconds * self._rate)))

    def _tick(self):
        # 단계 진행은 경과시간이 아니라 콜백 횟수로 센다. 시뮬이 과부하되면
        # /clock 이 크게 건너뛰어 경과시간 기준이 단계를 통째로 건너뛴다.
        if self._stage == S_WAIT_TAKEOVER:
            # 컨트롤러가 붙기 전에도 계속 낸다. 붙는 순간 kp 가 0 이 되므로
            # 그때 명령이 이미 가 있어야 한다. 아무도 안 듣는 동안 발행하는
            # 것은 공짜다.
            self._send(CMD_PASSIVE_TO_DOWN)
            self._count += 1
            took_over = self._z is not None and self._z < TAKEOVER_Z
            if took_over or self._count > self._ticks(TAKEOVER_TIMEOUT):
                self._advance(S_HOLD_DOWN,
                              '컨트롤러 인수 확인(z=%.3f) -> 기립 시작'
                              % (self._z if self._z is not None else float('nan')))
            return

        if self._stage == S_HOLD_DOWN:
            # 명령 2 를 관문(1.8초) 너머까지 유지하면 FIXEDDOWN 을 스치듯 지나
            # FIXEDSTAND 로 넘어간다. 여기서 명령 0 으로 눌러 웅크림을 완전히
            # 수렴시키면 안 된다 — 깊은 웅크림(calf -2.8)에서는 정강이가
            # 몸무게에 눌려 다시 못 일어난다(실측 몸통 0.114 m 고정).
            self._send(CMD_PASSIVE_TO_DOWN)
            self._count += 1
            if self._count >= self._ticks(HOLD_DOWN_SEC):
                self._advance(S_SETTLE, 'FIXEDSTAND 전환 -> 자세 고정')
            return

        if self._stage == S_SETTLE:
            # 명령 0 이면 지금 상태(FIXEDSTAND)에 머문다. 계속 2 를 누르고
            # 있으면 1.8초 뒤 FIXEDDOWN 으로 되돌아가 왕복한다.
            self._send(0)
            self._count += 1
            if self._count >= self._ticks(SETTLE_SEC):
                self._advance(S_READY, '기립 완료 — 주행 명령까지 대기')
            return

        if self._stage == S_READY:
            # 여기서 바로 보행으로 넘기지 않는다. RL 정책은 명령이 0 이어도
            # 가만히 서 있지 않고 요가 계속 틀어진다(운용에서 확인).
            # FIXEDSTAND 는 관절을 잡아 두는 상태라 드리프트가 없으므로,
            # 서서 기다리다가 첫 주행 명령이 올 때 넘어간다.
            if not self._ready_sent:
                self._ready_out.publish(Bool(data=True))
                self._ready_sent = True
            moving = (self._last_cmd is not None
                      and max(abs(self._vx), abs(self._vy), abs(self._wz)) > 1e-3)
            if not moving:
                self._send(0)
                return
            self._advance(S_TO_WALK, '주행 명령 수신 -> 보행 모드 진입')
            return

        if self._stage == S_TO_WALK:
            self._send(self._walk_cmd)
            self._count += 1
            if self._count >= self._ticks(WALK_HOLD_SEC):
                self._advance(S_RELAY, '보행 모드. cmd_vel 중계 시작')
            return

        # 보행 중. 정책은 연속 명령을 전제하므로 잠깐 끊겨도 마지막 값을 잇는다.
        if self._last_cmd is None:
            self._send(0)
            return
        stale = (self.get_clock().now() - self._last_cmd).nanoseconds / 1e9
        if stale > self._timeout:
            self._send(0)
        else:
            self._send(0, self._vx, self._vy, self._wz)


def main(args=None):
    rclpy.init(args=args)
    node = BenchRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
