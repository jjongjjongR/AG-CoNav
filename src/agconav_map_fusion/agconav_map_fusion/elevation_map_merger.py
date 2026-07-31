"""Merges the drone/wheel/leg elevation maps once map_merge_collector's merge_trigger fires.

design.md 6-3 / 6-4 / 9-4 / 9-5 / "노드 간 연결 방식": this node is a separate
process from map_merge_collector, so it cannot receive the validated maps
via a direct function call -- it subscribes independently to the same 3
absolute elevation_map topics map_merge_collector does (reliable /
transient_local / keep_last / depth 1). Because that QoS is durability
TRANSIENT_LOCAL, a subscriber always receives the latest retained sample on
each topic regardless of when it subscribed, so by the time
map_merge_collector's merge_trigger arrives this node already holds the same
3 maps map_merge_collector validated. It also subscribes to merge_trigger
itself (published only after map_merge_collector's validation, including
grid-alignment, has passed).

On trigger, it computes the union output grid (design.md 6-3 / 9-4), merges
the 3 elevation layers into it (6-4 / 9-5: wheel > leg > drone priority,
valid values never overwritten by NaN), publishes /merged/elevation_map
once, and reports merge_status=True. If a map is still missing when the
trigger fires -- an ordering edge case worth handling explicitly, since 2
independent subscriptions on 2 different topics carry no cross-topic
ordering guarantee -- it aborts via merge_error + merge_status=False instead
of crashing.
"""

from agconav_map_fusion.grid_math import (
    build_merged_grid_map_message, build_output_grid, extract_elevation, MERGE_ORDER, ROBOTS,
)
from grid_map_msgs.msg import GridMap
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String


class ElevationMapMerger(Node):
    """Merges the 3 robots' elevation maps into one, exactly once."""

    def __init__(self):
        super().__init__('elevation_map_merger')

        # design.md "구현 시 참고사항": absolute topic defaults (single
        # system-wide node), still parameterized for flexibility. Same
        # defaults as map_merge_collector's -- both subscribe to the same
        # 3 source topics independently.
        for robot in ROBOTS:
            self.declare_parameter(f'{robot}_elevation_map_topic', f'/{robot}/elevation_map')
        self.declare_parameter('merge_trigger_topic', '/merged/merge_trigger')
        self.declare_parameter('merged_elevation_map_topic', '/merged/elevation_map')
        self.declare_parameter('merge_status_topic', '/merged/merge_status')
        self.declare_parameter('merge_error_topic', '/merged/merge_error')
        # design.md 9-4: same guard as map_merge_collector's -- normally
        # never trips here since map_merge_collector already checked it on
        # the same inputs, but this node re-derives the output grid itself
        # (see module docstring) so it re-checks rather than assuming.
        self.declare_parameter('max_grid_cells', 30_000_000)

        self._maps = {robot: None for robot in ROBOTS}
        self._merge_done = False

        # design.md 9-1: elevation_map is reliable / transient_local / keep_last / depth 1.
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        trigger_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # design.md 9-5: merged elevation_map is reliable / transient_local /
        # keep_last / depth 1, same shape as the 3 input elevation_map topics.
        merged_map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # design.md 9-6: merge_error / merge_status are reliable / transient_local.
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        error_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._merged_map_pub = self.create_publisher(
            GridMap, self.get_parameter('merged_elevation_map_topic').value, merged_map_qos)
        self._merge_status_pub = self.create_publisher(
            Bool, self.get_parameter('merge_status_topic').value, status_qos)
        self._merge_error_pub = self.create_publisher(
            String, self.get_parameter('merge_error_topic').value, error_qos)

        self._map_subs = {}
        for robot in ROBOTS:
            map_topic = self.get_parameter(f'{robot}_elevation_map_topic').value

            def make_map_callback(r):
                return lambda msg: self._map_callback(r, msg)

            self._map_subs[robot] = self.create_subscription(
                GridMap, map_topic, make_map_callback(robot), map_qos)

        self._trigger_sub = self.create_subscription(
            Bool, self.get_parameter('merge_trigger_topic').value, self._trigger_callback,
            trigger_qos)

        self.get_logger().info(
            'waiting for merge_trigger on '
            f'"{self.get_parameter("merge_trigger_topic").value}".')

    def _map_callback(self, robot, msg):
        self._maps[robot] = msg

    def _trigger_callback(self, msg):
        # design.md: the merge itself fires exactly once, ever.
        if not msg.data or self._merge_done:
            return
        self._merge_done = True

        missing = [robot for robot in ROBOTS if self._maps[robot] is None]
        if missing:
            error = (
                f'merge_trigger fired but elevation_map not yet received here for: '
                f'{missing} (see module docstring on transient_local timing)')
            self.get_logger().error(error)
            self._merge_error_pub.publish(String(data=error))
            self._merge_status_pub.publish(Bool(data=False))
            return

        max_grid_cells = self.get_parameter('max_grid_cells').value
        output_grid, error = build_output_grid(self._maps, max_grid_cells=max_grid_cells)
        if error is not None:
            self.get_logger().error(f'output grid computation failed, aborting merge: {error}')
            self._merge_error_pub.publish(String(data=error))
            self._merge_status_pub.publish(Bool(data=False))
            return

        self.get_logger().info(
            f'output grid: origin=({output_grid.origin_x:.2f}, {output_grid.origin_y:.2f}) '
            f'size={output_grid.n_rows}x{output_grid.n_cols} cells '
            f'({output_grid.n_rows * output_grid.resolution:.1f} x '
            f'{output_grid.n_cols * output_grid.resolution:.1f} m)')

        elevation = self._merge_elevation(output_grid)
        stamp = self.get_clock().now().to_msg()
        merged_map = build_merged_grid_map_message(elevation, output_grid, stamp)
        self._merged_map_pub.publish(merged_map)
        self._merge_status_pub.publish(Bool(data=True))
        self.get_logger().info('published /merged/elevation_map -- merge complete (once).')

    def _place_into_output(self, output, output_grid, grid_map):
        """Overwrite output's valid (non-NaN) cells with grid_map's, in place.

        design.md 6-4: "유효값을 NaN으로 덮지 않음" -- only cells where
        grid_map itself has a real value are written; everything else in
        output is left untouched. Called in MERGE_ORDER (drone, leg, wheel)
        so later calls take priority, per design.md's wheel > leg > drone
        rule.
        """
        elevation, origin_x, origin_y = extract_elevation(grid_map)
        n_rows, n_cols = elevation.shape
        row_offset = round((origin_x - output_grid.origin_x) / output_grid.resolution)
        col_offset = round((origin_y - output_grid.origin_y) / output_grid.resolution)
        region = output[row_offset:row_offset + n_rows, col_offset:col_offset + n_cols]
        valid = ~np.isnan(elevation)
        region[valid] = elevation[valid]

    def _merge_elevation(self, output_grid):
        """Build the merged elevation array (design.md 6-4 / 9-5).

        Starts all-NaN so cells no robot ever observed stay NaN, then places
        drone, leg, wheel in that fixed order so the higher-priority valid
        values always win -- never dict/set iteration order.
        """
        output = np.full((output_grid.n_rows, output_grid.n_cols), np.nan, dtype=np.float32)
        for robot in MERGE_ORDER:
            self._place_into_output(output, output_grid, self._maps[robot])
        return output


def main(args=None):
    rclpy.init(args=args)
    node = ElevationMapMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
