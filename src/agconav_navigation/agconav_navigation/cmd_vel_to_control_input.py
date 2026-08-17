#!/usr/bin/env python3
"""Nav2 의 cmd_vel 을 quadruped_ros2_control 의 /control_input 으로 옮긴다.

    /leg/cmd_vel (geometry_msgs/Twist)  ->  /control_input (control_input_msgs/Inputs)

왜 필요한가: RL 컨트롤러(`rl_quadruped_controller`)는 조이스틱 축 형식의
`control_input_msgs/Inputs` 를 받는다. Nav2 는 Twist 를 낸다. 축 매핑은
StateRL.cpp 에서 확인했다.

    control_.x   = ly        (전진)
    control_.y   = -lx       (좌우)
    control_.yaw = -rx       (회전)

그리고 그 값은 그대로 정책 관측의 commands 로 들어가 params_.commands_scale
(go2 legged_gym/robot_lab 기준 [2.0, 2.0, 0.25])이 곱해진다. 즉 lx/ly/rx 는
m/s·rad/s 단위의 명령과 1:1 로 대응한다.

**FSM 도 이 노드가 몰아 준다.** RL 컨트롤러는 기동 직후 PASSIVE 상태이고
    PASSIVE --(2)--> FIXEDDOWN --(4)--> FIXEDSTAND --(3)--> RL
순서로만 보행 모드에 들어간다. 각 상태는 percent_ > 1.5, 즉 900스텝/200 Hz =
4.5초를 채워야 다음 명령을 받는다(그전 명령은 조용히 무시된다). 문서에 없어
소스에서 읽은 규칙이라 여기 적어 둔다.

**명령을 끊지 않는다.** 정책은 연속 명령을 전제한다. Nav2 가 잠깐 멈춰도
마지막 값을 유지해 계속 발행한다(cmd_timeout 이 지나면 0 으로). CHAMP 시절
끊긴 명령(평균 5.6 Hz, 58% 정지) 때문에 leg 이 아예 못 걸었던 전례가 있다
(`10. 종단 테스트 — 전체 맵 주행 검증.md` §7).
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from control_input_msgs.msg import Inputs
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

CMD_PASSIVE_TO_DOWN = 2
CMD_DOWN_TO_STAND = 4
CMD_STAND_TO_RL = 3
STAGE_PUBLISH_COUNT = 100  # 각 전환 명령을 실제 콜백 100회 동안 유지


class CmdVelToControlInput(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_control_input')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('output_topic', '/control_input')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 1.0)
        self.declare_parameter('max_linear', 1.0)     # 7번 실험의 안정 최대
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('auto_stand', True)

        self._out = self.create_publisher(
            Inputs, self.get_parameter('output_topic').value, 10)
        ready_qos = QoSProfile(depth=1)
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        self._ready_out = self.create_publisher(
            Bool, '/leg/controller_ready', ready_qos)
        self._ready_out.publish(Bool(data=False))
        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self._on_cmd, 10)

        self._rate = float(self.get_parameter('publish_rate_hz').value)
        self._timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self._max_lin = float(self.get_parameter('max_linear').value)
        self._max_ang = float(self.get_parameter('max_angular').value)

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._last_cmd = None
        # 단계: 0 FIXEDDOWN / 1 FIXEDSTAND / 2 서서 대기 / 3 RL 진입 / 4 중계.
        # auto_stand=false 면 기립을 남이 시킨다는 뜻이므로 곧장 중계(4)로 간다.
        self._stage = 0 if self.get_parameter('auto_stand').value else 4
        self._stage_publish_count = 0
        self._ready_sent = False

        self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            'cmd_vel -> control_input 시작 (%.0f Hz, 상한 %.2f m/s / %.2f rad/s)'
            % (self._rate, self._max_lin, self._max_ang))

    def _on_cmd(self, msg):
        clamp = lambda v, m: max(-m, min(m, v))
        self._vx = clamp(msg.linear.x, self._max_lin)
        self._vy = clamp(msg.linear.y, self._max_lin)
        self._wz = clamp(msg.angular.z, self._max_ang)
        self._last_cmd = self.get_clock().now()

    def _send(self, command, vx=0.0, vy=0.0, wz=0.0):
        m = Inputs()
        m.command = int(command)
        m.ly = float(vx)      # StateRL: control_.x = ly
        m.lx = float(-vy)     # StateRL: control_.y = -lx
        m.rx = float(-wz)     # StateRL: control_.yaw = -rx
        m.ry = 0.0
        self._out.publish(m)

    def _tick(self):
        now = self.get_clock().now()

        # 기립 시퀀스. 시뮬레이션이 과부하되면 /clock 이 크게 건너뛰므로
        # 경과시간이 아니라 실제 timer callback 횟수로 단계를 진행한다.
        if self._stage == 0:
            # 상태 전환 명령을 짧은 펄스로 보내면 대형 맵 초기화 중 타이머가
            # 밀릴 때 컨트롤러가 그 한 번을 놓칠 수 있다. 각 단계에 서로 다른
            # 명령을 사용하므로 체류 시간 내내 안전하게 유지할 수 있다.
            self._send(CMD_PASSIVE_TO_DOWN)
            self._stage_publish_count += 1
            if self._stage_publish_count >= STAGE_PUBLISH_COUNT:
                self._stage = 1
                self._stage_publish_count = 0
                self.get_logger().info('FIXEDDOWN 완료 -> 기립 명령')
            return
        if self._stage == 1:
            self._send(CMD_DOWN_TO_STAND)
            self._stage_publish_count += 1
            if self._stage_publish_count >= STAGE_PUBLISH_COUNT:
                self._stage = 2
                self._stage_publish_count = 0
                self.get_logger().info('FIXEDSTAND 완료 -> RL 모드 진입')
            return
        if self._stage == 2:
            # !! 여기서 바로 RL 로 넘어가면 안 된다 !!
            # RL 정책은 명령이 0 이어도 가만히 서 있지 않는다. 실측하면 명령
            # 0 인 상태로 요가 계속 틀어져 제자리에서 빙글빙글 돈다. 모듈 F
            # 지도 생성과 Nav2 활성화까지 몇 분이 걸리므로, 그 동안 로봇이
            # 스폰 지점에서 벗어나고 방향도 엉망이 된 채로 목표를 받게 된다.
            # FIXEDSTAND 는 관절 위치를 잡아 두는 상태라 드리프트가 없다.
            # 그래서 서 있는 채로 기다리다가 **첫 주행 명령이 올 때** RL 로
            # 들어간다. controller_ready 는 지금 올려야 목표 전송이 풀린다
            # (그래야 cmd_vel 이 오고, 그때 RL 로 넘어간다).
            if not self._ready_sent:
                self._ready_out.publish(Bool(data=True))
                self._ready_sent = True
                self.get_logger().info(
                    'FIXEDSTAND 유지 — 첫 주행 명령까지 서서 대기')
            moving = (self._last_cmd is not None
                      and max(abs(self._vx), abs(self._vy), abs(self._wz)) > 1e-3)
            if not moving:
                self._send(0)      # 0 = 상태 전환 명령 없음. 그대로 서 있는다.
                return
            self._stage = 3
            self._stage_publish_count = 0
            self.get_logger().info('주행 명령 수신 -> RL 모드 진입')
            return
        if self._stage == 3:
            self._send(CMD_STAND_TO_RL)
            self._stage_publish_count += 1
            if self._stage_publish_count >= STAGE_PUBLISH_COUNT:
                self._stage = 4
                self.get_logger().info('RL 보행 모드. cmd_vel 중계 시작')
            return

        # 보행 중. Nav2 가 잠깐 멈춰도 마지막 값을 유지한다 —
        # 정책은 연속 명령을 전제하고, 끊기면 걸음이 이어지지 않는다.
        if self._last_cmd is None:
            self._send(0)
            return
        stale = (now - self._last_cmd).nanoseconds / 1e9
        if stale > self._timeout:
            self._send(0)     # 오래 끊기면 정지 (안전)
        else:
            self._send(0, self._vx, self._vy, self._wz)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToControlInput()
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
