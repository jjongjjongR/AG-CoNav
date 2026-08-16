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
    PASSIVE --(2)--> FIXEDDOWN --(2)--> FIXEDSTAND --(3)--> RL
순서로만 보행 모드에 들어간다. 각 상태는 percent_ > 1.5, 즉 900스텝/200 Hz =
4.5초를 채워야 다음 명령을 받는다(그전 명령은 조용히 무시된다). 문서에 없어
소스에서 읽은 규칙이라 여기 적어 둔다.

**명령을 끊지 않는다.** 정책은 연속 명령을 전제한다. Nav2 가 잠깐 멈춰도
마지막 값을 유지해 계속 발행한다(cmd_timeout 이 지나면 0 으로). CHAMP 시절
끊긴 명령(평균 5.6 Hz, 58% 정지) 때문에 leg 이 아예 못 걸었던 전례가 있다
(`10. 종단 테스트 — 전체 맵 주행 검증.md` §7).
"""
import rclpy
from control_input_msgs.msg import Inputs
from geometry_msgs.msg import Twist
from rclpy.node import Node

CMD_PASSIVE_TO_DOWN = 2
CMD_DOWN_TO_STAND = 2
CMD_STAND_TO_RL = 3
STATE_DWELL_SEC = 6.0     # FSM 최소 체류 4.5초 + 여유


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
        self._t0 = self.get_clock().now()
        self._stage = 0 if self.get_parameter('auto_stand').value else 3

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
        elapsed = (now - self._t0).nanoseconds / 1e9

        # 기립 시퀀스. 각 단계는 STATE_DWELL_SEC 을 채운 뒤 다음으로 간다.
        if self._stage == 0:
            self._send(CMD_PASSIVE_TO_DOWN if elapsed < 1.0 else 0)
            if elapsed > STATE_DWELL_SEC:
                self._stage = 1
                self._t0 = now
                self.get_logger().info('FIXEDDOWN 완료 -> 기립 명령')
            return
        if self._stage == 1:
            self._send(CMD_DOWN_TO_STAND if elapsed < 1.0 else 0)
            if elapsed > STATE_DWELL_SEC:
                self._stage = 2
                self._t0 = now
                self.get_logger().info('FIXEDSTAND 완료 -> RL 모드 진입')
            return
        if self._stage == 2:
            self._send(CMD_STAND_TO_RL if elapsed < 1.0 else 0)
            if elapsed > 3.0:
                self._stage = 3
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
