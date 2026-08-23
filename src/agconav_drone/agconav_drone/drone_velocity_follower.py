#!/usr/bin/env python3
"""스캔 경로를 실제 로터 추력으로 비행한다 (모듈 A의 표준 비행 방식).

drone_pose_controller의 SetEntityPose 순간이동을 대체한다. 순간이동은 물리엔진이
운동을 보지 못해 IMU가 죽고(실측 gyro 최대 0.0013 rad/s) 자세가 항상 수평이라
스캔 시야가 연직으로만 고정됐다. 실제 비행으로 바꾸면 기체가 기울고 오르내리며
근거리 반사를 더 얻어 지도가 좋아진다.

파라미터의 확정값과 근거는
docs/3. 최적 드론 움직임.md 를 따른다. 요지는 이 플러그인이 속도
지령을 지수 감쇠로 따라간다는 것(수평 tau 5.9 s, 수직 3.3 s)이고, 여기서
  - 제동 거리는 tau x 속도차 (등감속 v^2/(2a) 가 아니다)
  - 제동은 계단으로 (램프는 기체 한계의 54% 밖에 못 쓴다)
  - 횡방향 보정에는 위치항뿐 아니라 속도항이 필요하다
가 따라 나온다.

원문(영문) 설명:
Fly the scan path with the multicopter velocity controller instead of teleporting.

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
from rclpy.executors import ExternalShutdownException
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


class DroneVelocityFollower(Node):
    def __init__(self):
        super().__init__('drone_velocity_follower')
        self.declare_parameter('path_file', '')
        # !! 확정값은 6.0 이다 !!
        # 문서 10(종단 테스트): v10 x 6 은 wheel 계획 실패 19건으로 실패,
        # v6 x 3 은 실패 0건 + 191.5 m 주행 성공. 문서 9 도 최적 5.2~6.0 m/s.
        # 6 -> 8 m/s 사이에서 제어가 무너진다(선 이탈 0.053 -> 0.312 m,
        # 고도 0.61 -> 3.00 m). 고도 흔들림이 곧 지도 높이 잡음이 된다.
        self.declare_parameter('cruise_speed_mps', 6.0)
        # 도착 반경. 끝점 이 거리 안에 들어오면 다음 구간으로 넘어간다.
        #
        # 이 값은 끝점 초과에 거의 영향이 없다 — 실측으로 확인했다.
        #     반경 0.8 -> 초과 1.01 m
        #     반경 1.1 -> 초과 1.04 m   (x 도달 범위까지 동일)
        # 초과는 반경이 아니라 **코너 전환 직후의 과도 피크**이고, 그건 도착
        # 속도와 횡방향 보정이 정한다(cross_damping 참고). 반경은 접근 구간
        # 길이만 바꾸므로 시간 쪽으로만 의미가 있다.
        self.declare_parameter('arrive_radius_m', 1.1)
        # 선에서 벗어난 거리를 되돌리는 이득. 크면 빨리 붙지만 진동한다.
        self.declare_parameter('cross_gain', 0.8)
        # 횡방향 **속도**를 죽이는 이득. 코너 전환 직후 이전 구간의 잔여 속도가
        # 그대로 횡방향 성분이 되는데, 비례항은 이탈이 자란 뒤에야 반응해서
        # 늦다. 이 항이 전환 즉시 제동을 건다.
        self.declare_parameter('cross_damping', 3.0)
        self.declare_parameter('climb_speed_mps', 6.0)
        # 순항 중 수직 속도 상한. 이륙용 6.0 을 그대로 쓰면 고도가 한 번
        # 어긋났을 때 되돌리는 명령 자체가 커서 반대편으로 넘어간다.
        # 실측(8 m/s, 설계 84 m): 70.06 ~ 94.39 m, 표준편차 1.917 m 로 ±10 m 진동.
        # 순항에서는 애초에 고도를 크게 바꿀 일이 없으므로 좁게 잡는다.
        self.declare_parameter('cruise_climb_mps', 2.5)
        # 고도 제어 감쇠항. 비례항만 있으면 목표를 지나친 뒤 반대로 밀고 다시
        # 지나치기를 반복한다(위 ±10 m 가 그것). 측정 수직 속도를 빼서 잡는다.
        self.declare_parameter('alt_damping', 1.4)
        # 수직 속도 추정의 저역통과 계수. TF 지터가 미분에서 증폭되므로 필요하다.
        # 1.0 이면 필터 없음.
        self.declare_parameter('vz_filter', 0.3)
        # 기울어 날 때 추력의 수직 성분이 cos(기울기)만큼 줄어 고도가 처진다.
        # 8 m/s 순항의 기울기에서 수 %가 빠지는데, 비례항만으로 메우려면 그만큼
        # 정상 편차가 남는다. 기울기를 알고 있으니 미리 보상한다. 0 이면 끈다.
        self.declare_parameter('tilt_feedforward', 1.0)
        # 고도 적분 이득. **0 이 맞다** — 실측으로 확인했다.
        # 정상 편차를 지우려고 0.25 를 넣어봤더니 오히려 편차가 ±1.1 -> ±2.0 m,
        # RMS 0.67 -> 1.23 으로 나빠졌다. 순항 중 고도 변동은 정상 편차가 아니라
        # 스트립 왕복(주기 20.8 s)이 만드는 **주기 외란**이라, 적분기는 그걸
        # 뒤늦게 쫓아가며 위상만 더 밀어 증폭시킨다. 파라미터로 남겨두되 끈다.
        self.declare_parameter('alt_integral', 0.0)
        # 적분항이 만들 수 있는 수직 속도 상한 [m/s]. 와인드업 방지.
        self.declare_parameter('alt_integral_limit', 0.8)
        # 제동 모델의 시상수 [s].
        #
        # 처음에는 등감속으로 보고 제동 거리를 v^2/(2a) 로 잡았는데 틀렸다.
        # 0 을 지령하면 플러그인은 dv/dt = -v/tau 로 반응한다(지수 감쇠).
        # 그러면 dv/dx = -1/tau 라 **속도가 거리에 선형으로** 줄어든다.
        # 실측(clean1, x=61.4 접근):
        #     남은 26 m -> 6.60 m/s      남은 10 m -> 3.96
        #     남은 22 m -> 5.98          남은  6 m -> 3.22
        #     남은 18 m -> 5.29          남은  3 m -> 2.66
        #     남은 14 m -> 4.70          남은 1.5 m -> 2.38
        #   기울기 -0.172 (m/s)/m 로 일정  =>  tau = 5.81 s
        #
        # 등감속 모델은 저속 구간을 과대평가한다. v^2/(2a) 로 31.0 m 를 줬지만
        # 실제로 필요한 것은 tau*(v - v_도착) = 5.81*(7.1-0.8) = 36.6 m 였고,
        # 그 차이 5.6 m 가 그대로 끝단 초과로 남았다.
        self.declare_parameter('brake_tau_s', 5.9)
        # 제동 거리에 더할 여유 [m]. tau 추정 오차만 덮으면 된다 — 정확도는
        # 도착 게이트(속도 조건)가 이미 보장한다.
        #
        # 크게 잡으면 그대로 대기 시간이 된다. 앞의 tau*(v - v_도착) 항이 이미
        # 도착 속도까지 정확히 감속시키므로, 그 시점에 남은 거리가 곧 여유이고
        # 그만큼을 creep 속도로 기어가야 한다. 실측: 여유 4.5 m 일 때 코너당
        # (4.5-0.8)/0.25 = 14.8 s, 9 코너에 133 s. 비행 시간의 47% 가 0.5 m/s
        # 이하 구간이었다(iter4: 351 s 중 164 s).
        #
        # tau 는 5.81 실측에 5.9 를 쓰므로 39 m 제동 구간에서 오차가 0.6 m 다.
        # 1.5 m 면 충분히 덮으면서 코너당 대기가 2.8 s 로 준다.
        self.declare_parameter('brake_margin_m', 1.5)
        # 제동이 끝났는데 아직 끝점에 못 닿았을 때의 접근 속도. 이게 없으면
        # 제동 거리 안에서 멈춘 뒤 v_cmd 가 계속 0 이라 영영 도착하지 못한다.
        # arrive_speed 보다 낮아야 도착 판정이 난다.
        self.declare_parameter('creep_mps', 0.4)
        # 감속하다 멈춰버리면 도착 판정을 못 하므로 하한을 둔다.
        # 단 끝점 반경 안에서는 이 하한을 풀어 실제로 멈추게 한다.
        self.declare_parameter('min_speed_mps', 1.5)
        # 도착으로 인정할 속도. 이보다 빠르면 끝점에 닿아도 다음 구간으로
        # 넘기지 않는다 — 그 속도가 직각 방향 초과분이 되기 때문이다.
        # 전환 시점의 잔여 속도는 그대로 직각 방향 초과분이 된다. 관성 주행
        # 거리는 brake_tau x v 이므로 0.8 m/s 면 4.7 m 를 더 간다(횡방향 보정이
        # 되당겨 실측 1.26 m). 0.35 로 낮추면 2.1 m 로 준다.
        # 0.35 는 정확했지만(이탈 0.63 m) 느렸다 — 2 -> 0.35 m/s 의 지수 꼬리가
        # tau*ln(2/0.35) = 10.3 s, 9 코너에 93 s 였다. 0.6 이면 7.1 s 로 줄고
        # 이탈은 0.63 -> 약 0.85 m 로 목표(1 m) 안에 남는다.
        self.declare_parameter('arrive_speed_mps', 0.6)
        # 고도 비례 이득.
        #
        # 0.30 (= 1/tau_z) 으로 낮춰봤더니 상승 오버슈트는 잡혔지만 순항 편차가
        # ±1.1 -> ±2.0 m 로 나빠졌다. 순항 중 고도 외란의 주기가 스트립 왕복과
        # 같은 20.8 s 인데, 이득 0.30 은 루프 대역폭을 정확히 그 주기로 끌어내려
        # 외란을 증폭시킨다. 외란을 억누르려면 오히려 이득이 높아야 한다.
        #
        # 그래서 이득은 0.8 로 되돌리고, 상승 오버슈트는 이득이 아니라
        # **상승률 상한**으로 잡는다(아래 alt_tau_s).
        self.declare_parameter('gain', 1.1)
        # 수직 응답의 시상수 [s]. 상승 중 "남은 거리 안에 멈출 수 있는 상승률"
        # (= 남은거리/tau_z) 로 상한을 걸어 오버슈트를 막는다. 수평과 같은
        # 지수 응답이며, 오버슈트 궤적에서 역산했다: 남은 10 m 에서 5.35 m/s 로
        # 올라가 17.5 m 를 더 갔으므로 tau_z = 17.5/5.35 = 3.3 s.
        self.declare_parameter('alt_tau_s', 3.3)
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
        self.cruise_climb = g('cruise_climb_mps')
        self.alt_damping = g('alt_damping')
        self.vz_filter = g('vz_filter')
        self.tilt_ff = g('tilt_feedforward')
        self.alt_ki = g('alt_integral')
        self.alt_tau = g('alt_tau_s')
        self.alt_i_lim = g('alt_integral_limit')
        self.radius = g('arrive_radius_m')
        self.cross_gain = g('cross_gain')
        self.cross_damping = g('cross_damping')
        self.brake_tau = g('brake_tau_s')
        self.brake_margin = g('brake_margin_m')
        self.creep = g('creep_mps')
        self.min_speed = g('min_speed_mps')
        self.arrive_speed = g('arrive_speed_mps')
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
        # 속도 추정 상태. 플러그인이 속도를 되돌려주지 않으므로 TF 를 미분한다.
        # 수직은 고도 감쇠에, 수평은 도착 판정에 쓴다.
        self._last_p = None
        self._last_p_t = None
        self._vx_est = 0.0
        self._vy_est = 0.0
        self._vz_est = 0.0
        self._alt_i = 0.0          # 고도 적분항 [m/s], 순항에서만 쌓는다
        self._max_x_seen = None
        self._min_x_seen = None
        self._seg_idx = -1              # 최근접 통과 판정용
        self._min_remain = float('inf')

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
        self.update_vel(p, now)

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
        # 상승 종료 조건은 **양쪽**을 봐야 한다. 예전에는 p.z < altitude - 2.0
        # 만 봐서 아래쪽만 확인했고, 오버슈트로 90 m 를 찍어도 "도달"로 보고
        # 수평 비행을 시작했다. 그러면 상승 오버슈트가 순항 구간에 그대로
        # 섞인다 — 실측(tau1): 최고 91.48 m, 84+-1 m 안으로 들어오는 데 7 s.
        if self.elapsed < self.settle or abs(p.z - self.altitude) > 2.0:
            # 상승도 수평과 똑같이 **계단 제동**으로 세운다.
            #
            # 지령을 비례로 줄이는 방식(상한 = 남은거리/tau_z)은 오버슈트를 못
            # 막았다 — 실측 +8.02 m. 플랜트가 tau_z 로 지연 응답하므로 지령을
            # 낮춰도 이미 붙은 속도가 그대로 밀고 올라간다. 수평에서 확인한
            # 것과 같은 현상이다.
            #
            # 그래서 남은 거리가 tau_z * vz 이하가 되면 0 을 그대로 때린다.
            # 6 m/s 로 올라가면 19.8 m 전부터 제동이 시작되고, 그 구간에서
            # 속도가 지수적으로 죽어 목표에 0 으로 닿는다.
            #
            # 시간도 이쪽이 빠르다. 예전에는 비례 구간에서 질질 끌고 오버슈트를
            # 되돌리느라 상승·정착에 68 s (전체의 21%) 를 썼다.
            err_z = self.altitude - p.z
            brake_z = self.alt_tau * max(0.0, self._vz_est)
            if err_z > 0.0 and err_z <= brake_z:
                vz = 0.0
            elif err_z > 0.0:
                vz = self.climb
            else:
                vz = self.alt_cmd(self.altitude, p.z, self.cruise_climb, 0.0)
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

        remain = seg - along
        speed_now = math.hypot(self._vx_est, self._vy_est)

        # 도착 판정. 예전에는 끝점 1.5 m 전에서 곧바로 다음 waypoint 로 넘겼는데,
        # 그 순간 기체는 아직 8 m/s 로 달리고 있었다. 다음 구간은 직각 방향이라
        # 그 속도가 통째로 초과분이 된다 — 실측: 설계 끝 x=61.4 를 지나 80.0,
        # 반대쪽은 -38.6 을 지나 -57.3 까지. 양끝 각각 18.6 / 18.7 m 초과였고
        # 비행 시간의 39.9% 를 박스 밖에서 보냈다(줄 간격이 25 m 인데!).
        #
        # 그래서 위치와 속도를 함께 본다. 실제로 느려지기 전에는 넘기지 않는다.
        # 다만 이미 지나쳐 버린 경우에는 붙잡고 있어도 소용이 없으므로 통과시킨다.
        # 이 구간에서 목표에 가장 가까웠던 거리. 아래 '최근접 통과' 판정에 쓴다.
        if self._seg_idx != self.idx:
            self._seg_idx = self.idx
            self._min_remain = float('inf')
        self._min_remain = min(self._min_remain, remain)

        arrived = remain <= self.radius and speed_now <= self.arrive_speed
        # 포획 반경 안으로 못 들어오고 그 주위를 왕복하는 경우가 있다.
        # 원인: creep 으로 접근하다 0 을 지령해도 관성 주행이 tau x creep
        # (= 5.9 x 0.4 = 2.36 m) 이라 반경 1.1 m 창을 지나쳐 버린다. 반대편에서
        # 되돌아와도 같은 일이 반복돼 안정 극한주기가 된다.
        # 실측(간격 9.5 m, 10 m/s): 목표 y=-166.10 주위를 -164.2 ~ -167.9 로
        # 진폭 ±1.85 m 왕복하며 2,430 초를 소모하고도 도착 판정이 나지 않았다.
        #
        # 포획이 성립하려면 tau x creep < arrive_radius 여야 하는데 지금은
        # 2.36 > 1.1 이다. creep 을 0.19 m/s 로 낮추거나 반경을 2.4 m 로 넓히면
        # 되지만 둘 다 코너 동작을 바꾼다. 대신 **최근접점을 지나 멀어지기
        # 시작하면 도착으로 본다** — 지령값을 전혀 건드리지 않으므로 정상적으로
        # 포획되는 경우의 동작은 그대로다(그때는 arrived 가 먼저 성립한다).
        passed = (speed_now <= self.arrive_speed
                  and self._min_remain < self.radius * 2.5
                  and remain > self._min_remain + 0.3)
        overshot = along >= seg + self.radius
        if arrived or passed or overshot:
            self.idx += 1
            if self.idx >= len(self.wps):
                self.finish()
                return
            self.get_logger().info(
                'waypoint %d / %d 도달 (선 이탈 %.2f m, 진입 속도 %.2f m/s%s)'
                % (self.idx, len(self.wps), abs(cross), speed_now,
                   ', 초과 통과' if overshot else (', 최근접 통과' if passed else '')))
            return

        # 모퉁이 앞 감속. 예전에는 v = decel_gain x 남은거리 였는데, 이 프로파일은
        # 감속을 시작하는 순간 가장 큰 감가속도를 요구한다(v dv/dx = -k^2 d 이므로
        # 8 m/s 진입 시 시작점에서 4 m/s^2, 끝에서 1 m/s^2). 요구가 앞쪽에 몰려
        # 플러그인이 못 따라가고 그대로 밀렸다.
        #
        # 대신 "남은 거리 안에 멈출 수 있는 속도" v = sqrt(2 a d) 를 쓴다. 이건
        # 구간 전체에서 감가속도 요구가 a 로 일정해서 추종이 쉽다. 감속 시작
        # 지점은 v^2/(2a) 로 자동으로 정해진다 — 8 m/s, a=2.0 이면 16 m 전.
        #
        # 속도를 바꿔가며 실험할 때 이 방식이 중요하다: decel_gain 은 감속 거리가
        # 2v 로 속도에 비례해 커져서, 건너뛰기 구간(25 m)보다 길어지는 12 m/s
        # 이상에서는 모퉁이 제어가 아예 성립하지 않았다.
        # 제동은 계단으로 준다. 부드러운 램프(v = sqrt(2 a d))로 줄여봤더니
        # 플러그인이 지령과 실제의 *차이*에 반응하는 구조라, 차이가 작으면
        # 약하게만 제동했다. 실측(altfix_v8, x=61.4 접근):
        #     남은 20 m → 실제 7.13 m/s
        #     남은 10 m → 실제 6.88
        #     남은  2 m → 실제 6.27      18 m 동안 0.86 m/s 밖에 못 줄임
        #   램프 구간 최강 감속 -0.85 m/s^2  (기체 한계의 54%)
        #   계단 구간 최강 감속 -1.52 m/s^2  (한계의 97%)
        # 그래서 순항 속도를 유지하다가 제동 지점에서 0 을 그대로 때린다.
        # 큰 오차가 최대 제동을 끌어낸다.
        #
        # 제동 거리는 지령이 아니라 **실측 속도**로 계산한다. 지령 8 m/s 라도
        # 실제로는 7.2 m/s 밖에 안 나오므로, 지령으로 잡으면 필요 이상으로
        # 일찍 멈춰 스캔 구간을 낭비한다.
        # 지수 감쇠 모델: 0 을 지령하면 속도가 거리에 선형으로 줄어드므로
        # v -> v_도착 에 필요한 거리는 tau * (v - v_도착) 이다.
        brake_dist = (self.brake_tau * max(0.0, speed_now - self.arrive_speed)
                      + self.brake_margin)
        if remain <= self.radius:
            v_cmd = 0.0                       # 끝점 — 세운다
        elif remain <= brake_dist and speed_now > self.arrive_speed:
            v_cmd = 0.0                       # 계단 제동
        elif remain <= brake_dist:
            v_cmd = self.creep                # 이미 느리다 — 살살 붙인다
        else:
            v_cmd = self.speed
        # 제동 중에도 횡방향은 잡아야 한다. v_cmd 로 묶으면 0 이 되어 선에서
        # 벗어난 채로 멈춘다.
        corr_lim = max(v_cmd, self.min_speed)
        # 횡방향 보정에는 **속도항**이 필요하다.
        #
        # 비례항만 쓰면(-cross_gain * cross) 이탈이 0 일 때 보정도 0 이다.
        # 그런데 코너를 도는 순간이 정확히 그 상태다: 이탈은 0 인데 이전 구간
        # 방향으로 도착 속도만큼(0.6 m/s) 이미 달리고 있다. 보정은 이탈이
        # 자라난 뒤에야 시작되므로 그때는 이미 1 m 밀린 뒤다 — 실측 초과가
        # 도착 반경을 0.8 -> 1.1 로 바꿔도 1.01 -> 1.04 로 꿈쩍 안 한 이유가
        # 이것이다(반경 문제가 아니라 과도 피크였다).
        #
        # 횡방향 속도를 직접 넣으면 전환 즉시 제동이 걸린다. 이득이 1 보다
        # 커야 플랜트 지연(tau 5.9 s)을 이기고 속도를 실제로 죽인다.
        cross_v = -self._vx_est * uy + self._vy_est * ux
        corr = max(-corr_lim,
                   min(corr_lim,
                       -self.cross_gain * cross - self.cross_damping * cross_v))
        vx = v_cmd * ux - corr * uy
        vy = v_cmd * uy + corr * ux
        n = math.hypot(vx, vy)
        lim = math.hypot(v_cmd, corr_lim)
        if n > lim:
            vx, vy = vx / n * lim, vy / n * lim
        # 순항 고도는 좁은 상한으로 붙잡는다. 여기서 6 m/s 를 허용하면 한 번
        # 어긋난 뒤 되돌리는 명령이 그대로 반대편 이탈이 된다.
        vz = self.alt_cmd(tz, p.z, self.cruise_climb, self.tilt_ff)
        self.publish_world(vx, vy, vz, yaw)
        self.report(p, 'wp %d/%d 이탈%.1fm' % (self.idx + 1, len(self.wps), abs(cross)))

    def update_vel(self, p, now):
        """측정 속도를 갱신한다. 수직은 고도 감쇠에, 수평은 도착 판정에 쓴다.

        플러그인이 속도를 되돌려주지 않으므로 TF 위치를 미분한다. 미분은 지터를
        증폭하므로 1차 저역통과를 건다. dt는 시뮬 시계에서 직접 재는데, 타이머가
        요청한 주기대로 뛴다고 가정하면 부하 상황에서 그 가정이 깨지기
        때문이다(이 노드가 이미 elapsed 를 그렇게 다룬다).
        """
        if self._last_p is not None:
            dt = (now - self._last_p_t).nanoseconds * 1e-9
            if dt > 1e-3:
                a = self.vz_filter
                q = self._last_p
                self._vx_est += a * ((p.x - q.x) / dt - self._vx_est)
                self._vy_est += a * ((p.y - q.y) / dt - self._vy_est)
                self._vz_est += a * ((p.z - q.z) / dt - self._vz_est)
        self._last_p, self._last_p_t = p, now
        # 박스 초과를 즉시 알 수 있게 기록해 둔다(종료 시 로그).
        self._max_x_seen = p.x if self._max_x_seen is None else max(self._max_x_seen, p.x)
        self._min_x_seen = p.x if self._min_x_seen is None else min(self._min_x_seen, p.x)

    def alt_cmd(self, target_z, z, limit, tilt_ff):
        """고도 지령. 비례 − 감쇠 (+ 기울기 보상).

        비례항만 쓰면 목표를 지나친 뒤 반대로 밀기를 반복한다. 실측(8 m/s,
        설계 84 m): 70.06 ~ 94.39 m, 표준편차 1.917 m. 측정 수직 속도를 빼서
        그 진동을 잡는다.

        tilt_ff 는 기울여 날 때 빠지는 추력의 수직 성분을 미리 채운다. 추력이
        기체 z축이므로 수직 성분은 cos(기울기)배가 되고, 부족분은 g(cos⁻¹−1)에
        해당하는 가속이다. 속도 지령이므로 한 주기(1/rate) 동안의 속도 증분으로
        환산해 넣는다. 순항에서만 켠다 — 이륙 중에는 기울지 않는다.
        """
        err = target_z - z
        # 적분은 순항에서만(tilt_ff 가 켜진 호출). 상승 중에는 오차가 수십 m 라
        # 적분하면 그대로 와인드업이 된다.
        if tilt_ff and self.alt_ki > 0.0:
            self._alt_i += self.alt_ki * err / self.rate
            self._alt_i = max(-self.alt_i_lim, min(self.alt_i_lim, self._alt_i))
        v = self.gain * err + self._alt_i - self.alt_damping * self._vz_est
        if tilt_ff:
            tilt = self.tilt_angle()
            if tilt > 1e-3:
                # 부족한 수직 가속 g(1/cos-1) 을 **속도** 지령으로 옮긴다.
                # 플랜트가 시상수 tau_z 로 속도를 따라가므로 정상 상태에서
                # 가속 a 를 유지하려면 속도 오프셋 a*tau_z 가 필요하다.
                # 예전에는 /rate 로 나눠서 (20 Hz 기준) 1/66 크기밖에 안 됐고,
                # 그래서 보상이 사실상 없었다.
                v += tilt_ff * 9.81 * (1.0 / math.cos(tilt) - 1.0) * self.alt_tau
        return max(-limit, min(limit, v))

    def tilt_angle(self):
        """기체 z축과 연직 사이의 각 [rad]. 자세 쿼터니언에서 직접 뽑는다."""
        if self.pose is None:
            return 0.0
        q = self.pose.rotation
        # 회전행렬 3열(기체 z축의 world 성분)의 z 값 = 1 - 2(x²+y²)
        cz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        return math.acos(max(-1.0, min(1.0, cz)))

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
        """경로 종료. 정지까지 계속 제동한다.

        예전에는 여기서 빈 Twist 를 한 번 내고 끝냈다. 속도 지령 컨트롤러에
        0 을 한 번 준다고 기체가 그 자리에 서지 않는다 — 실측: 마지막 waypoint
        (x=61.4) 이후에도 계속 밀려 x=99.4, 설계 끝에서 38.0 m 밖에 멈췄다.
        타이머를 살려두고 멈출 때까지 0 을 계속 낸다.
        """
        self.finished = True
        self.cmd.publish(Twist())
        self.get_logger().info(
            '경로 완료 — /drone/path_status=True  (x 도달 범위 %.1f ~ %.1f)'
            % (self._min_x_seen if self._min_x_seen is not None else float('nan'),
               self._max_x_seen if self._max_x_seen is not None else float('nan')))
        self.status.publish(Bool(data=True))
        self.timer.cancel()
        self.timer = self.create_timer(1.0 / self.rate, self.brake)

    def brake(self):
        """정지할 때까지 0 지령을 유지한다."""
        now = self.get_clock().now()
        pose = self.read_pose()
        if pose is not None:
            self.update_vel(pose.translation, now)
        self.cmd.publish(Twist())
        if math.hypot(self._vx_est, self._vy_est) < 0.1:
            self.get_logger().info('정지 완료 (x %.1f)' % self._last_p.x)
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = DroneVelocityFollower()
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
