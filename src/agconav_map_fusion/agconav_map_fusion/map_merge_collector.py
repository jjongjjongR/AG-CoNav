"""Collects the drone/wheel/leg elevation maps, then triggers and validates the merge once.

design.md 6-1 / 8-1 / 8-2 / 9-1 / 9-2: unlike agconav_ground_mapping's nodes,
this node is a single instance for the whole system, not one per robot
namespace. It subscribes directly to the three modules' absolute elevation_map
and elevation_map_status topics (design.md "구현 시 참고사항": this is not a
per-namespace reusable node, so CONTRIBUTING 5's relative-topic-name rule does
not apply to these inputs -- topic names are still exposed as parameters for
flexibility).

design.md 7-1 explicitly leaves map_merge_validator's responsibility open to
be folded into map_merge_collector ("팀 결정 필요") -- that choice is made
here: right when the merge trigger fires (6-1), this node also validates the
collected set (6-2 / 9-3: frame == map, resolution == 0.10 m/cell, elevation
layer present) before handing it off. On failure it aborts the merge and
publishes merge_error (design.md 8-7 / 9-6); on success, the validated set is
exposed via get_validated_maps() for map_merge_grid_builder (6-3, not yet
wired up).
"""

import math

from grid_map_msgs.msg import GridMap
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

ROBOTS = ('drone', 'wheel', 'leg')

# README 3.1 / 3.3: single global `map` frame, fixed 0.10 m/cell resolution.
EXPECTED_FRAME_ID = 'map'
EXPECTED_RESOLUTION = 0.10
ELEVATION_LAYER = 'elevation'


class MapMergeCollector(Node):
    """Buffers the 3 robots' elevation maps and fires the merge trigger once."""

    def __init__(self):
        super().__init__('map_merge_collector')

        # design.md "구현 시 참고사항": absolute topic defaults (single
        # system-wide node), still parameterized for flexibility.
        for robot in ROBOTS:
            self.declare_parameter(f'{robot}_elevation_map_topic', f'/{robot}/elevation_map')
            self.declare_parameter(f'{robot}_status_topic', f'/{robot}/elevation_map_status')
        # design.md 8-7: absolute /merged/... topic, single system-wide node.
        self.declare_parameter('merge_error_topic', '/merged/merge_error')

        self._maps = {robot: None for robot in ROBOTS}
        self._complete = {robot: False for robot in ROBOTS}
        self._merge_triggered = False
        self._validated_maps = None

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
        # design.md 9-6: merge_error is reliable / transient_local.
        error_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._merge_error_pub = self.create_publisher(
            String, self.get_parameter('merge_error_topic').value, error_qos)

        self._map_subs = {}
        self._status_subs = {}
        for robot in ROBOTS:
            map_topic = self.get_parameter(f'{robot}_elevation_map_topic').value
            status_topic = self.get_parameter(f'{robot}_status_topic').value

            def make_map_callback(r):
                return lambda msg: self._map_callback(r, msg)

            def make_status_callback(r):
                return lambda msg: self._status_callback(r, msg)

            self._map_subs[robot] = self.create_subscription(
                GridMap, map_topic, make_map_callback(robot), map_qos)
            self._status_subs[robot] = self.create_subscription(
                Bool, status_topic, make_status_callback(robot), status_qos)

        self.get_logger().info(
            'Collecting elevation maps: '
            + ', '.join(
                f'{robot}='
                f'{self.get_parameter(f"{robot}_elevation_map_topic").value}'
                for robot in ROBOTS
            ))

    def _map_callback(self, robot, msg):
        self._maps[robot] = msg

    def _status_callback(self, robot, msg):
        self._complete[robot] = bool(msg.data)
        if msg.data:
            self._check_merge_ready()

    def _check_merge_ready(self):
        # design.md: the merge trigger fires exactly once, ever.
        if self._merge_triggered:
            return
        if not all(self._complete.values()):
            return

        # design.md 6-1: detect missing input (완료 상태인데 지도가 안 온 경우).
        # Not fatal here -- map_merge_validator decides whether to abort.
        missing = [robot for robot in ROBOTS if self._maps[robot] is None]
        if missing:
            self.get_logger().warn(
                f'completion status is True for all robots, but no elevation_map '
                f'was ever received for: {missing}')

        self._merge_triggered = True
        self.get_logger().info(
            'all 3 robots reported elevation_map completion -- merge triggered (once).')

        maps = self.get_collected_maps()
        error = self._validate_maps(maps)
        if error is not None:
            self.get_logger().error(f'map validation failed, aborting merge: {error}')
            self._merge_error_pub.publish(String(data=error))
            return

        self.get_logger().info(
            'all 3 elevation maps passed validation (frame/resolution/layer).')
        self._validated_maps = maps

    def _validate_maps(self, maps):
        """Check frame/resolution/layer for all 3 robots (design.md 6-2 / 9-3).

        Returns an error message string on failure, or None if every map
        passes. No resampling on mismatch -- validation only ever accepts
        or rejects the maps as-is.
        """
        for robot in ROBOTS:
            grid_map = maps[robot]
            if grid_map is None:
                return f'{robot}: elevation_map was never received'
            if grid_map.header.frame_id != EXPECTED_FRAME_ID:
                return (
                    f'{robot}: frame_id "{grid_map.header.frame_id}" != '
                    f'"{EXPECTED_FRAME_ID}"')
            if not math.isclose(grid_map.info.resolution, EXPECTED_RESOLUTION, abs_tol=1e-6):
                return (
                    f'{robot}: resolution {grid_map.info.resolution} != '
                    f'{EXPECTED_RESOLUTION} m/cell')
            if ELEVATION_LAYER not in grid_map.layers:
                return f'{robot}: missing required layer "{ELEVATION_LAYER}"'
        return None

    def get_collected_maps(self):
        """Return {robot: latest GridMap or None}, snapshotted at call time."""
        return dict(self._maps)

    def get_validated_maps(self):
        """Return the validated {robot: GridMap} set for map_merge_grid_builder.

        (6-3) Populated once the merge trigger has fired AND validation
        passed; stays None until then, and also None if validation failed.
        """
        return self._validated_maps

    def is_merge_triggered(self):
        return self._merge_triggered


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
