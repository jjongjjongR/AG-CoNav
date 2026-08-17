#!/usr/bin/env python3
"""Send wheel then leg goals after Module F maps and Nav2 are ready."""
import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class PipelineGoalSender(Node):
    def __init__(self):
        super().__init__('pipeline_goal_sender')
        defaults = {
            # 저장된 주행성 지도(maps/*_nav_map)에서 골랐다. 두 가지를 만족한다.
            #
            # 1) 첫 시도에 계획된다.
            #    Nav2 는 이진 지도가 아니라 팽창 코스트맵 위에서 계획한다.
            #    장애물에서 robot_radius 안쪽은 사실상 치명이고 거기서
            #    inflation_radius(0.55 m)까지 비용이 남는다. 그래서 로봇
            #    반지름만큼만 침식해 "이어져 있다"고 봐도 통로가 좁으면
            #    플래너가 길을 못 찾는다.
            #    예전 목표 (-59, -45) 는 여유가 0.99 m 뿐이라 첫 계획이
            #    "Failed to create plan with tolerance of: 0.5" 로 실패하고
            #    3번째 재시도에서야 성공했다. 아래 두 지점은 반지름 +
            #    팽창(=wheel 1.10 m, leg 0.95 m)으로 침식하고도 스폰과
            #    이어져 있고 여유가 15 m 넘는다.
            #
            # 2) wheel 과 leg 의 주행성 차이가 드러난다.
            #    wheel 목표는 둘 다 갈 수 있는 곳이고, leg 목표는 wheel 이
            #    반지름 침식만으로도 닿지 못하는 영역이다. leg 만 도착하면
            #    모듈 F 의 로봇별 분리가 실제로 동작한다는 증거가 된다.
            #    (안전 침식 기준 도달 면적: wheel 3.88%, leg 42.16%)
            #
            # 지도를 다시 만들면 이 좌표도 다시 골라야 한다.
            # !! 월드 가장자리에서 멀어야 한다 !!
            # 한때 wheel 목표를 (-272.40, -87.00) 으로 뒀다가 로봇을 잃었다.
            # 지도상으로는 여유 15.6 m 였지만 월드 서쪽 끝(x = -289.3)에서
            # 17 m 뿐이라, 경로 추종이 조금 넘어가자 지형 밖으로 나가
            # z = -32115 m 까지 떨어졌다. 주행성 지도의 여유와 **월드 경계까지의
            # 거리는 다른 값**이다. 목표를 옮길 때 둘 다 확인할 것.
            # 월드 범위: x -289.3~289.2, y -241.1~244.3.
            #
            # 둘 다 **중앙 평지**(빨간 표식 자리)다. 가장자리에서 200 m 넘게
            # 안쪽이고 두 로봇 모두 실제 도착 실적이 있다
            # (wheel 오차 0.27 m, leg 오차 4.3 m).
            # 두 점을 조금 띄워 둔 것은 거의 동시에 도착할 때 서로 부딪히지
            # 않게 하기 위해서다.
            'wheel_goal_x': -59.0, 'wheel_goal_y': -45.0,
            'leg_goal_x': -56.5, 'leg_goal_y': -47.0,
            'max_attempts': 3,
            'start_robot': 'wheel',
            'require_maps': True,
            'simultaneous': False,
            'max_tf_age_sec': 0.75,
            'ready_stable_ticks': 4,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.maps = {'wheel': False, 'leg': False}
        self.controller_ready = {'wheel': True, 'leg': False}
        self.create_subscription(
            OccupancyGrid, '/wheel/nav_map',
            lambda _msg: self._map_ready('wheel'), qos)
        self.create_subscription(
            OccupancyGrid, '/leg/nav_map',
            lambda _msg: self._map_ready('leg'), qos)
        self.create_subscription(
            Bool, '/leg/controller_ready',
            lambda msg: self._controller_ready('leg', msg.data), qos)
        # rclpy.node.Node.clients 는 읽기 전용 서비스-client 목록 속성이다.
        # 같은 이름에 action client dict를 대입하면 시작 즉시 AttributeError가 난다.
        self.action_clients = {
            robot: ActionClient(self, NavigateToPose, f'/{robot}/navigate_to_pose')
            for robot in ('wheel', 'leg')
        }
        # bt_navigator 가 정말 ACTIVE 인지 보기 위한 lifecycle 조회.
        self._state_clients = {
            robot: self.create_client(GetState, f'/{robot}/bt_navigator/get_state')
            for robot in ('wheel', 'leg')
        }
        self._nav_active = {'wheel': False, 'leg': False}
        self._state_futures = {'wheel': None, 'leg': None}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, None, spin_thread=True)
        # Module D must collect LiDAR only while a navigation goal is really
        # active.  A latched signal also gives late-starting mappers the
        # correct state instead of making them guess from timing.
        self.mapping_active_publishers = {
            robot: self.create_publisher(
                Bool, f'/{robot}/mapping_active', qos)
            for robot in ('wheel', 'leg')
        }
        for robot in self.mapping_active_publishers:
            self._publish_mapping_active(robot, False)
        self.goal_marker_publisher = self.create_publisher(
            MarkerArray, '/navigation_goal_markers', qos)
        self._publish_goal_markers()
        start_robot = str(self.get_parameter('start_robot').value)
        if start_robot not in ('wheel', 'leg'):
            raise ValueError('start_robot must be "wheel" or "leg"')
        self.robot = start_robot
        self.busy_robots = set()
        self.finished_robots = set()
        self.attempts = {'wheel': 0, 'leg': 0}
        self.ready_ticks = {'wheel': 0, 'leg': 0}
        self.create_timer(2.0, self._tick)
        self.get_logger().info('Module F 지도와 Nav2 action server 대기 중')

    def _map_ready(self, robot):
        if not self.maps[robot]:
            self.maps[robot] = True
            self.get_logger().info(f'/{robot}/nav_map 수신')

    def _controller_ready(self, robot, ready):
        ready = bool(ready)
        if self.controller_ready[robot] != ready:
            self.get_logger().info(
                f'{robot} controller_ready={ready}')
        self.controller_ready[robot] = ready

    def _tick(self):
        if (bool(self.get_parameter('require_maps').value)
                and not all(self.maps.values())):
            return
        if bool(self.get_parameter('simultaneous').value):
            pending = [
                robot for robot in ('wheel', 'leg')
                if robot not in self.finished_robots
                and robot not in self.busy_robots
            ]
            # Initial dispatch is a real barrier: both action servers and both
            # TF chains must be stable before either robot moves.  This keeps
            # "simultaneous=true" from degenerating into wheel-first merely
            # because its Nav2 stack happened to initialize earlier.
            if len(pending) == 2 and not self.finished_robots:
                ready = {
                    robot: self._ready_to_send(robot) for robot in pending
                }
                if all(ready.values()):
                    for robot in pending:
                        self._send_goal(robot)
            else:
                for robot in pending:
                    self._try_send(robot)
            return
        if self.robot != 'done':
            self._try_send(self.robot)

    def _try_send(self, robot):
        if robot in self.busy_robots:
            return
        if self._ready_to_send(robot):
            self._send_goal(robot)

    def _ready_to_send(self, robot):
        if not self.controller_ready[robot]:
            self.ready_ticks[robot] = 0
            self.get_logger().warn(
                f'{robot} 보행 컨트롤러 기립 완료 대기 중',
                throttle_duration_sec=10.0)
            return False
        client = self.action_clients[robot]
        if not client.server_is_ready():
            self.ready_ticks[robot] = 0
            self.get_logger().warn(
                f'/{robot}/navigate_to_pose 아직 비활성',
                throttle_duration_sec=10.0)
            return False
        # !! server_is_ready() 만으로는 부족하다 !!
        # bt_navigator 는 lifecycle 이 inactive 인 동안에도 액션을 광고한다.
        # 그래서 위 검사는 통과하는데 목표는
        #   [bt_navigator]: Action server is inactive. Rejecting the goal.
        # 로 거절된다. 실제로 leg 가 이것 때문에 첫 두 번을 거절당하고 세
        # 번째에야 출발했다. lifecycle 상태를 직접 확인한다.
        if not self._nav_is_active(robot):
            self.ready_ticks[robot] = 0
            self.get_logger().warn(
                f'{robot} bt_navigator 활성화 대기 중',
                throttle_duration_sec=10.0)
            return False
        if not self._tf_is_stably_ready(robot):
            return False
        max_attempts = int(self.get_parameter('max_attempts').value)
        if self.attempts[robot] >= max_attempts:
            self.get_logger().error(f'{robot} 목표 {max_attempts}회 실패')
            self._publish_mapping_active(robot, False)
            self.finished_robots.add(robot)
            if not bool(self.get_parameter('simultaneous').value):
                self.robot = 'done'
            return False
        return True

    def _nav_is_active(self, robot):
        """bt_navigator lifecycle 이 ACTIVE 인가.

        타이머 콜백 안에서 부르므로 **블로킹하면 안 된다**(같은 실행기에서
        응답을 처리해야 해서 spin_until_future_complete 는 교착한다).
        비동기로 한 번 물어보고 결과가 오면 캐시한다.
        """
        if self._nav_active.get(robot):
            return True
        client = self._state_clients[robot]
        if not client.service_is_ready():
            return False
        future = self._state_futures.get(robot)
        if future is None:
            self._state_futures[robot] = client.call_async(GetState.Request())
            return False
        if not future.done():
            return False
        self._state_futures[robot] = None
        try:
            active = future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        except Exception:                                   # noqa: BLE001
            active = False
        if active:
            self._nav_active[robot] = True
            self.get_logger().info(f'{robot} bt_navigator ACTIVE 확인')
        return active

    def _send_goal(self, robot):
        client = self.action_clients[robot]
        max_attempts = int(self.get_parameter('max_attempts').value)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(
            self.get_parameter(f'{robot}_goal_x').value)
        goal.pose.pose.position.y = float(
            self.get_parameter(f'{robot}_goal_y').value)
        goal.pose.pose.orientation.w = 1.0
        self.busy_robots.add(robot)
        self.get_logger().info(
            f'{robot} 목표 전송 (시도 {self.attempts[robot] + 1}/{max_attempts})')
        client.send_goal_async(goal).add_done_callback(
            lambda future, selected=robot: self._goal_response(future, selected))

    def _goal_response(self, future, robot):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().warn(f'{robot} 목표 거절됨; 재시도 예정')
            self._publish_mapping_active(robot, False)
            self.busy_robots.discard(robot)
            self.ready_ticks[robot] = 0
            return
        self.attempts[robot] += 1
        self.ready_ticks[robot] = 0
        self._publish_mapping_active(robot, True)
        handle.get_result_async().add_done_callback(
            lambda future, selected=robot: self._result(future, selected))

    def _result(self, future, robot):
        status = future.result().status
        self._publish_mapping_active(robot, False)
        if status == 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info(f'{robot} navigation 성공')
            self.finished_robots.add(robot)
            if not bool(self.get_parameter('simultaneous').value):
                self.robot = 'leg' if robot == 'wheel' else 'done'
        else:
            self.get_logger().warn(
                f'{robot} navigation 상태 {status}; 재시도 예정')
        self.busy_robots.discard(robot)

    def _tf_is_stably_ready(self, robot):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', f'{robot}/base_link', Time())
        except TransformException as ex:
            self.ready_ticks[robot] = 0
            self.get_logger().warn(
                f'{robot} 최신 TF 대기 중: {ex}',
                throttle_duration_sec=10.0)
            return False

        tf_time = Time.from_msg(transform.header.stamp)
        age_sec = (self.get_clock().now() - tf_time).nanoseconds / 1e9
        max_age = float(self.get_parameter('max_tf_age_sec').value)
        if age_sec < 0.0 or age_sec > max_age:
            self.ready_ticks[robot] = 0
            self.get_logger().warn(
                f'{robot} TF가 {age_sec:.2f}초 뒤처짐 '
                f'(허용 {max_age:.2f}초); 목표 전송 보류',
                throttle_duration_sec=5.0)
            return False

        self.ready_ticks[robot] += 1
        required = int(self.get_parameter('ready_stable_ticks').value)
        if self.ready_ticks[robot] < required:
            self.get_logger().info(
                f'{robot} TF 안정 확인 '
                f'{self.ready_ticks[robot]}/{required} (지연 {age_sec:.2f}초)')
            return False
        return True

    def _publish_mapping_active(self, robot, active):
        self.mapping_active_publishers[robot].publish(Bool(data=active))
        self.get_logger().info(
            f'{robot} mapping_active={active} '
            f'({"주행 점군 누적 시작" if active else "주행 점군 누적 중지"})')

    def _publish_goal_markers(self):
        markers = MarkerArray()
        # Saved elevation-map medians at the selected flat poses.  Marker z is
        # its centre, so add half of the 3 m cylinder length.
        # 목표 지점의 **지형 고도 + 1.5 m**. 표식이 땅에 묻히거나 공중에 뜨지
        # 않게 하려면 좌표를 옮길 때 이 값도 같이 옮겨야 한다.
        # 지형 고도는 저장된 드론 지도에서 읽는다(maps/drone_elevation_map).
        #   wheel (-272.40, -87.00) 지형 3.18 m
        #   leg   ( -90.80, 100.80) 지형 8.00 m
        # 읽을 때 함정: GridMap 은 **열 우선**으로 저장된다
        # (layout.dim[0]=column_index, dim[1]=row_index). 행/열을 바꿔 읽으면
        # 엉뚱한 곳의 고도가 나오고 NaN 이 섞인다.
        #   wheel (-59.00, -45.00) 지형 1.83 m / leg (-56.50, -47.00) 지형 1.67 m
        marker_z = {'wheel': 3.33, 'leg': 3.17}
        for marker_id, robot in enumerate(('wheel', 'leg')):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'navigation_goals'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(
                self.get_parameter(f'{robot}_goal_x').value)
            marker.pose.position.y = float(
                self.get_parameter(f'{robot}_goal_y').value)
            marker.pose.position.z = marker_z[robot]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 3.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            markers.markers.append(marker)
        self.goal_marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineGoalSender()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
