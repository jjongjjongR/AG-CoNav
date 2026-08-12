#!/usr/bin/env python3
"""Fly the scan path with the multicopter velocity controller instead of teleporting.

drone_pose_controller applies each commanded pose with SetEntityPose, which
moves the model without the physics engine ever seeing motion: the IMU reads a
stationary, level vehicle for the whole flight (measured: peak gyro
0.0013 rad/s across two 180-degree turns). Here the same waypoints are flown by
commanding velocity into the model's MulticopterVelocityControl plugin, so the
rotors actually push the airframe and every derived signal -- IMU, odometry,
scan timing -- comes from real integrated motion.

The plugin takes body-frame velocity, so the world-frame command is rotated by
the current yaw. Position feedback comes from /tf (map -> drone/base_link),
which the world's OdometryPublisher provides.

Cruise speed and altitude are read from the same path file agconav_drone's
generator produced, so the flight matches the teleport runs.
"""

from __future__ import annotations

import math

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VelocityPathFollower(Node):
    def __init__(self):
        super().__init__('velocity_path_follower')
        self.declare_parameter('path_file', '')
        self.declare_parameter('cruise_speed_mps', 8.0)
        self.declare_parameter('arrive_radius_m', 1.5)
        # 선에서 벗어난 거리를 되돌리는 이득. 크면 빨리 붙지만 진동한다.
        self.declare_parameter('cross_gain', 0.8)
        self.declare_parameter('climb_speed_mps', 6.0)
        # 모퉁이 감속. v = decel_gain * (구간 남은 거리), 순항 속도로 상한.
        # 0.5면 남은 16 m에서 8 m/s, 4 m에서 2 m/s가 되어 관성이 실릴 시간이 생긴다.
        self.declare_parameter('decel_gain', 0.5)
        # 감속하다 멈춰버리면 도착 판정을 못 하므로 하한을 둔다.
        self.declare_parameter('min_speed_mps', 1.5)
        self.declare_parameter('gain', 0.8)
        # 50 Hz는 이 시뮬에서 과했다. 노드가 115개 도는 6코어 머신에서는 타이머가
        # 요청대로 뛰지도 못하면서 부하만 더한다. 제어는 위치 되먹임이라
        # 20 Hz로도 8 m/s 순항에서 명령 간 0.4 m로 충분하다.
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('settle_sec', 4.0)

        g = lambda n: self.get_parameter(n).value      # noqa: E731
        with open(g('path_file')) as f:
            data = yaml.safe_load(f)
        self.wps = [(w['position']['x'], w['position']['y'], w['position']['z'])
                    for w in data['waypoints']]
        self.altitude = float(data.get('altitude', self.wps[0][2]))
        self.speed = g('cruise_speed_mps')
        self.climb = g('climb_speed_mps')
        self.radius = g('arrive_radius_m')
        self.cross_gain = g('cross_gain')
        self.decel_gain = g('decel_gain')
        self.min_speed = g('min_speed_mps')
        self.gain = g('gain')
        self.rate = g('rate_hz')
        self.settle = g('settle_sec')

        self.pose = None
        self.idx = 0
        self.elapsed = 0.0
        self.armed = False
        self.finished = False
        self._reported = -1
        # 경과 시간은 시계에서 직접 잰다. 틱을 세어 elapsed += 1/rate 로 누적하면
        # "타이머가 요청한 주기대로 뛴다"고 가정하는 셈인데, 부하가 걸리면 그
        # 가정이 깨진다. 실측: 6코어 CPU 포화 + 실시간계수 0.4 상황에서 50 Hz로
        # 요청한 타이머가 초당 1회 미만으로 뛰었고, 그래서 실제 2분이 지나도
        # 노드가 아는 경과 시간은 3초에 못 미쳐 이륙조차 시작하지 않았다.
        self._t0 = None
        self._last_enable = None
        self.reached_alt = False

        self.cmd = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.enable = self.create_publisher(Bool, '/drone/enable',
                                            QoSProfile(depth=1,
                                                       reliability=ReliabilityPolicy.RELIABLE,
                                                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                       history=HistoryPolicy.KEEP_LAST))
        self.status = self.create_publisher(
            Bool, '/drone/path_status',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
        # 위치는 tf2 버퍼로 받는다. 예전에는 /tf를 직접 구독해 마지막 메시지를
        # 들고 있었는데, 부하가 걸리면 콜백이 밀려 '자기가 아는 위치'가 실제보다
        # 한참 뒤처진다. 그 상태로 아래 상승 판정을 하면 이미 목표 고도를 넘었는데도
        # "아직 못 올라갔다"고 보고 상승 명령을 계속 낸다.
        # 실측: 목표 84 m인데 드론이 1,043 m까지 올라갔다.
        # 전용 스레드(spin_thread=True)가 버퍼를 계속 채우므로 이 노드의 타이머가
        # 밀려도 조회하는 값은 항상 최신이다.
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self, spin_thread=True)
        self.get_logger().info('waypoint %d개, 고도 %.1f m, 순항 %.1f m/s'
                               % (len(self.wps), self.altitude, self.speed))
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    def read_pose(self):
        """map -> drone/base_link 의 최신 값. 없으면 None."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'drone/base_link', rclpy.time.Time())
            return t.transform
        except Exception:
            return None

    def tick(self):
        if self.finished:
            return
        now = self.get_clock().now()
        if self._t0 is None:
            self._t0 = now
        self.elapsed = (now - self._t0).nanoseconds * 1e-9

        # Keep asserting enable. It is published through a ros_gz bridge that
        # comes up alongside this node, so a single early message is simply lost
        # if the bridge has not connected yet -- which looks exactly like a
        # controller that ignores every twist it is sent.
        # 목표 고도에 한 번 닿기 전까지는 매 틱 보낸다. 시뮬 시간 기준으로
        # 간격을 두면 안 된다 - Nav2까지 함께 뜨는 동안 실시간계수가 0.014까지
        # 떨어져서, "시뮬 1초마다"가 실제로는 70초마다가 된다. 컨트롤러가 준비되기
        # 전에 온 한 번을 놓치면 그대로 꺼진 채 남는다. 실측: enable이 딱 한 번
        # 전달된 뒤 드론이 3.06 m에서 미동도 하지 않았다. Bool 하나라 비용은 없다.
        if not self.reached_alt:
            self.enable.publish(Bool(data=True))
        elif self._last_enable is None or self.elapsed - self._last_enable >= 1.0:
            self._last_enable = self.elapsed
            self.enable.publish(Bool(data=True))
        if not self.armed:
            if self.elapsed > 3.0:
                self.armed = True
                self.get_logger().info('아밍 완료 — 이륙')
            return
        self.pose = self.read_pose()
        if self.pose is None:
            # 위치를 모르면 명령을 내지 않는다. 모르는 채로 상승 명령을 내면
            # 그게 바로 폭주가 된다.
            self.cmd.publish(Twist())
            return

        p = self.pose.translation
        yaw = yaw_of(self.pose.rotation)

        # 안전장치. 위치 되먹임이 깨져 고도가 폭주하면 멈춘다(실측 1,043 m 사례).
        #
        # 단계를 나눠야 한다. 이륙 전에는 지상 3 m에서 출발하므로 설계 고도와의
        # 차이가 당연히 30 m를 넘는다. 그걸 그대로 위반으로 보면 정상 상승까지
        # 중단시킨다 - 실제로 그렇게 막혔다(고도 3.1 m에서 "30 m 넘게 벗어났다").
        #   상승 중  : 위로 넘치는 경우만 본다.
        #   순항 중  : 목표 고도에 한 번 닿은 뒤에는 위아래 모두 본다.
        if not self.reached_alt and abs(p.z - self.altitude) < 5.0:
            self.reached_alt = True
            self.get_logger().info('설계 고도 도달 (%.1f m)' % p.z)
        runaway = (p.z > self.altitude + 30.0) if not self.reached_alt \
            else (abs(p.z - self.altitude) > 30.0)
        if runaway and self.armed:
            self.get_logger().error(
                '고도 %.1f m — 설계 %.1f m에서 30 m 넘게 벗어났다. 비행 중단.'
                % (p.z, self.altitude))
            self.finish()
            return

        # Climb to survey altitude before chasing the first waypoint, otherwise
        # the drone would cut a diagonal across the box on the way up.
        # 상승은 비례 제어로 한다. 예전처럼 최대 상승률을 고정으로 내보내면
        # 위치 정보가 한 순간이라도 밀렸을 때 목표를 지나쳐 계속 올라간다.
        if self.elapsed < self.settle or p.z < self.altitude - 2.0:
            vz = max(-self.climb, min(self.climb, self.gain * (self.altitude - p.z)))
            self.publish_world(0.0, 0.0, vz, yaw)
            self.report(p, 'climb')
            return

        # 목표점만 보고 날면 도착 반경 안에서 진로가 흔들리고 모퉁이를 크게 돌아
        # 잔디깎기(ㄹ) 패턴이 뭉개진다. 대신 직전 waypoint에서 현재 waypoint로
        # 이어지는 '선'을 따라간다: 진행 방향 성분으로 전진하고, 선에서 벗어난
        # 만큼(cross-track)을 따로 잡아당긴다. 그래야 줄 간격이 설계값 그대로
        # 유지된다.
        ax, ay, _ = self.wps[self.idx - 1] if self.idx > 0 else (p.x, p.y, p.z)
        tx, ty, tz = self.wps[self.idx]

        ex, ey = tx - ax, ty - ay
        seg = math.hypot(ex, ey)
        if seg < 1e-6:
            self.idx += 1
            if self.idx >= len(self.wps):
                self.finish()
            return
        ux, uy = ex / seg, ey / seg                      # 선의 진행 방향
        rx, ry = p.x - ax, p.y - ay
        along = rx * ux + ry * uy                        # 선 위로 얼마나 왔나
        cross = -rx * uy + ry * ux                       # 선에서 옆으로 얼마나 벗어났나

        if along >= seg - self.radius:
            self.idx += 1
            if self.idx >= len(self.wps):
                self.finish()
                return
            self.get_logger().info('waypoint %d / %d 도달 (선 이탈 %.2f m)'
                                   % (self.idx, len(self.wps), abs(cross)))
            return

        # 모퉁이 앞에서는 미리 감속한다. 순항 속도를 끝까지 유지하다가 도착
        # 반경에서 방향만 바꾸면, 15.38 kg 기체의 관성 때문에 그대로 밀려 나간다.
        # 실측(감속 없음): 직선 구간 선 이탈은 0.6 m로 정확했는데 모퉁이에서
        # 설계 끝점 x=61.4를 지나 76.97까지, 이탈 15.6 m까지 부풀었다.
        # 줄 간격이 25 m이므로 이 정도면 ㄹ 패턴이 무너지고 스캔 폭이 어긋난다.
        # 남은 거리에 비례해 속도를 줄이면 관성이 실릴 시간이 생긴다.
        remain = seg - along
        v_cmd = min(self.speed, max(self.min_speed, self.decel_gain * remain))
        corr = max(-v_cmd, min(v_cmd, -self.cross_gain * cross))
        vx = v_cmd * ux - corr * uy
        vy = v_cmd * uy + corr * ux
        n = math.hypot(vx, vy)
        if n > v_cmd:                                     # 합이 지령 속도를 넘지 않게
            vx, vy = vx / n * v_cmd, vy / n * v_cmd
        vz = max(-self.climb, min(self.climb, self.gain * (tz - p.z)))
        self.publish_world(vx, vy, vz, yaw)
        self.report(p, 'wp %d/%d 이탈%.1fm' % (self.idx + 1, len(self.wps), abs(cross)))

    def publish_world(self, vx, vy, vz, yaw):
        """World-frame velocity -> body frame, which is what the plugin expects."""
        c, s = math.cos(-yaw), math.sin(-yaw)
        t = Twist()
        t.linear.x = vx * c - vy * s
        t.linear.y = vx * s + vy * c
        t.linear.z = vz
        t.angular.z = -0.5 * yaw          # hold heading at 0
        self.cmd.publish(t)

    def report(self, p, what):
        sec = int(self.elapsed)
        if sec % 5 or sec == self._reported:
            return
        self._reported = sec
        self.get_logger().info('%5.1fs  %-10s  위치 (%7.2f, %8.2f, %6.2f)'
                               % (self.elapsed, what, p.x, p.y, p.z))

    def finish(self):
        self.finished = True
        self.cmd.publish(Twist())
        self.get_logger().info('경로 완료 — /drone/path_status=True')
        self.status.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPathFollower()
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
