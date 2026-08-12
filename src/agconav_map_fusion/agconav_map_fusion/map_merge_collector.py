"""Collects the drone/wheel/leg elevation maps, validates them once wheel+leg finish.

design.md 6-1 / 8-2 / 8-3 / 9-1 / 9-2 / 9-3: map_merge_collector,
elevation_map_merger and merged_elevation_map_saver are 3 separate ROS2
nodes (i.e. 3 separate OS processes), wired together only by topics -- see
design.md "노드 간 연결 방식". A separate process cannot call another
process's Python functions directly, so despite older drafts of this
design describing some of these links as "내부 함수 호출", every link
between these 3 nodes is, and has to be, a topic.

This node subscribes to all 3 modules' absolute elevation_map topics (they
are the actual merge material) but only wheel/leg's elevation_map_status
(the merge trigger) -- deliberately NOT drone's. design.md's pipeline order
is A(드론) -> F -> B/C -> D: the drone produces its map once, up front,
before wheel/leg even start moving. By the time wheel AND leg have both
reported navigation complete, the drone's map generation is structurally
guaranteed to already be finished -- there is no ordering in this pipeline
that could have wheel/leg finish first. Subscribing to a 3rd completion
signal that has necessarily already fired earlier would only add a
dependency with no effect on the trigger condition.

Once wheel AND leg are both complete (checked once, ever), this node
validates the 3 collected maps: frame == map, resolution == 0.10 m/cell,
elevation layer present (design.md 6-2 / 9-3), and grid alignment -- each
map's origin must land on an exact multiple of the resolution away from the
computed output grid's origin, or its cells would straddle the output
grid's cell boundaries instead of coinciding with them. On any failure it
publishes merge_error + merge_status=False and stops. On success it
publishes merge_trigger=True; elevation_map_merger picks that up and, using
its own (transient_local-latched, so already-received) copies of the same 3
elevation_map topics, actually builds and publishes the merged map.
"""

from agconav_map_fusion.grid_math import (
    build_output_grid, check_grid_alignment, ROBOTS, validate_maps,
)
from grid_map_msgs.msg import GridMap
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

STATUS_ROBOTS = ('wheel', 'leg')
# ground_elevation_mapper의 수신 감시(check_period_sec)와 같은 개념의 내부
# 폴링 주기 -- merge_wait_timeout_sec처럼 외부에 노출할 필요는 없는 값이라
# 파라미터로 만들지 않고 상수로 둔다.
_TIMEOUT_CHECK_PERIOD_SEC = 1.0


class MapMergeCollector(Node):
    """Buffers the 3 robots' elevation maps and fires merge_trigger once."""

    def __init__(self):
        super().__init__('map_merge_collector')

        # design.md "구현 시 참고사항": absolute topic defaults (single
        # system-wide node), still parameterized for flexibility.
        for robot in ROBOTS:
            self.declare_parameter(f'{robot}_elevation_map_topic', f'/{robot}/elevation_map')
        # No drone_status_topic parameter -- see module docstring for why
        # drone's completion is intentionally not part of the trigger.
        for robot in STATUS_ROBOTS:
            self.declare_parameter(f'{robot}_status_topic', f'/{robot}/elevation_map_status')

        self.declare_parameter('merge_trigger_topic', '/merged/merge_trigger')
        self.declare_parameter('merge_error_topic', '/merged/merge_error')
        self.declare_parameter('merge_status_topic', '/merged/merge_status')
        # design.md 9-4: reject the output grid if it would need more cells
        # than this. Default covers the full 500x500m world at 0.10 m/cell
        # (5000x5000 = 25,000,000 cells, README 2.2) with headroom.
        self.declare_parameter('max_grid_cells', 30_000_000)
        # design.md 9-3: how close to an integer cell offset an input map's
        # origin must be, relative to the output grid's origin, to count as
        # aligned. (origin_i - output_origin) / resolution must round to
        # within this tolerance of a whole number.
        self.declare_parameter('grid_alignment_tolerance', 1e-6)
        # design.md 6-1: wheel/leg 완료 상태를 기다리는 데 원래 타임아웃이
        # 없었다 -- 한쪽이 저장 실패(Bool(False)) 또는 영영 미도착이면 이
        # 노드가 무한 대기한다. 정확한 근거는 없는 placeholder다: 이
        # 프로젝트의 로봇 이동 시나리오상 몇 초~몇십 초 걸릴 수 있어 너무
        # 짧게 잡지 않았다. 실측/시나리오 기반 재조정 필요.
        self.declare_parameter('merge_wait_timeout_sec', 30.0)

        self._maps = {robot: None for robot in ROBOTS}
        self._complete = {robot: False for robot in STATUS_ROBOTS}
        self._merge_triggered = False
        # ground_elevation_mapper의 data_timeout_sec 패턴(파라미터화된
        # 타임아웃 + create_timer 주기 체크)을 그대로 이식했다.
        self._merge_wait_timeout = Duration(
            seconds=self.get_parameter('merge_wait_timeout_sec').value)
        self._wait_start = self.get_clock().now()
        self._timeout_reported = False

        # design.md 9-1: elevation_map is reliable / transient_local / keep_last / depth 1.
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # design.md 9-2: elevation_map_status is reliable / transient_local
        # (history/depth not specified there, keep_last/1 as elsewhere in the
        # project's status topics).
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # design.md 9-6: merge_error / merge_status are reliable / transient_local.
        error_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        status_result_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # design.md 8-3 / "노드 간 연결 방식": merge_trigger is the topic that
        # replaces the old (impossible, cross-process) "내부 함수 호출" link
        # between this node and elevation_map_merger.
        trigger_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._merge_error_pub = self.create_publisher(
            String, self.get_parameter('merge_error_topic').value, error_qos)
        self._merge_status_pub = self.create_publisher(
            Bool, self.get_parameter('merge_status_topic').value, status_result_qos)
        self._merge_trigger_pub = self.create_publisher(
            Bool, self.get_parameter('merge_trigger_topic').value, trigger_qos)

        self._map_subs = {}
        for robot in ROBOTS:
            map_topic = self.get_parameter(f'{robot}_elevation_map_topic').value

            def make_map_callback(r):
                return lambda msg: self._map_callback(r, msg)

            self._map_subs[robot] = self.create_subscription(
                GridMap, map_topic, make_map_callback(robot), map_qos)

        self._status_subs = {}
        for robot in STATUS_ROBOTS:
            status_topic = self.get_parameter(f'{robot}_status_topic').value

            def make_status_callback(r):
                return lambda msg: self._status_callback(r, msg)

            self._status_subs[robot] = self.create_subscription(
                Bool, status_topic, make_status_callback(robot), status_qos)

        self._timeout_timer = self.create_timer(
            _TIMEOUT_CHECK_PERIOD_SEC, self._check_merge_timeout)

        self.get_logger().info(
            'Collecting elevation maps: '
            + ', '.join(
                f'{robot}='
                f'{self.get_parameter(f"{robot}_elevation_map_topic").value}'
                for robot in ROBOTS)
            + '; merge trigger on wheel+leg completion (drone not subscribed, see docstring).'
            + f' merge_wait_timeout_sec={self.get_parameter("merge_wait_timeout_sec").value}')

    def _map_callback(self, robot, msg):
        self._maps[robot] = msg

    def _status_callback(self, robot, msg):
        new_value = bool(msg.data)
        # design.md 6-1: 타임아웃은 "노드 시작 또는 마지막으로 완료 상태가
        # 바뀐 시점"부터 잰다 -- 상태가 실제로 바뀔 때만 기준 시각을 리셋.
        if self._complete[robot] != new_value:
            self._wait_start = self.get_clock().now()
        self._complete[robot] = new_value
        if msg.data:
            self._check_merge_ready()

    def _check_merge_timeout(self):
        # design.md 6-1: 병합이 이미 트리거됐다면(성공이든 검증 실패든, 1회성
        # 시도가 이미 끝났다면) 더 이상 타임아웃을 감시할 이유가 없다. 타임아웃
        # 자체를 이미 보고했다면 매 폴링마다 중복 발행하지 않도록 한 번만
        # 보고한다.
        if self._merge_triggered or self._timeout_reported:
            return
        elapsed = self.get_clock().now() - self._wait_start
        if elapsed <= self._merge_wait_timeout:
            return

        self._timeout_reported = True
        missing = [robot for robot in STATUS_ROBOTS if not self._complete[robot]]
        error = (
            f'merge_wait_timeout_sec={self._merge_wait_timeout.nanoseconds / 1e9:.1f}s '
            f'elapsed without both wheel/leg elevation_map_status=True '
            f'(still missing: {", ".join(missing)})')
        self.get_logger().error(f'merge wait timed out, aborting: {error}')
        self._merge_error_pub.publish(String(data=error))
        self._merge_status_pub.publish(Bool(data=False))

    def _check_merge_ready(self):
        # design.md: the merge trigger fires exactly once, ever.
        if self._merge_triggered:
            return
        if not all(self._complete.values()):
            return

        self._merge_triggered = True
        self.get_logger().info(
            'wheel and leg both report elevation_map completion -- validating (once).')

        error = self._validate()
        if error is not None:
            self.get_logger().error(f'map validation failed, aborting merge: {error}')
            self._merge_error_pub.publish(String(data=error))
            self._merge_status_pub.publish(Bool(data=False))
            return

        self.get_logger().info(
            'all 3 elevation maps passed validation (frame/resolution/layer/grid alignment).')
        self._merge_trigger_pub.publish(Bool(data=True))
        self.get_logger().info(
            'published merge_trigger -- elevation_map_merger takes over from here.')

    def _validate(self):
        """Run every validation step (design.md 6-2 / 9-3), in order.

        Returns an error message string on the first failure, or None if
        every map passes frame/resolution/layer checks and grid alignment.
        """
        error = validate_maps(self._maps)
        if error is not None:
            return error

        max_grid_cells = self.get_parameter('max_grid_cells').value
        output_grid, error = build_output_grid(self._maps, max_grid_cells=max_grid_cells)
        if error is not None:
            return error

        tolerance = self.get_parameter('grid_alignment_tolerance').value
        return check_grid_alignment(self._maps, output_grid, tolerance=tolerance)


def main(args=None):
    rclpy.init(args=args)
    node = MapMergeCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
