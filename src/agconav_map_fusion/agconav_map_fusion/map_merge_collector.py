"""Collects the drone/wheel/leg elevation maps, then triggers and validates the merge once.

design.md 6-1 / 8-1 / 8-2 / 9-1 / 9-2: unlike agconav_ground_mapping's nodes,
this node is a single instance for the whole system, not one per robot
namespace. It subscribes directly to the three modules' absolute elevation_map
and elevation_map_status topics (design.md "구현 시 참고사항": this is not a
per-namespace reusable node, so CONTRIBUTING 5's relative-topic-name rule does
not apply to these inputs -- topic names are still exposed as parameters for
flexibility).

design.md 7-1 explicitly leaves map_merge_validator's, map_merge_grid_builder's
and elevation_map_merger's responsibilities open to be folded into
map_merge_collector ("팀 결정 필요") -- that choice is made here: right when
the merge trigger fires (6-1), this node also validates the collected set
(6-2 / 9-3: frame == map, resolution == 0.10 m/cell, elevation layer present),
computes the union output grid's origin/size (6-3 / 9-4), and merges the 3
elevation layers into it (6-4 / 9-5: wheel > leg > drone priority, valid
values never overwritten by NaN). On any failure it aborts and publishes
merge_error + merge_status=False (design.md 8-7 / 9-6); on full success it
publishes /merged/elevation_map once, merge_status=True (design.md 8-6), and
saves that same message to an mcap rosbag2 (6-5 / 8-8 / 9-7).

The grid_map_msgs/GridMap wire-format packing/unpacking (axis flip,
column-major flatten) mirrors agconav_ground_mapping's ground_elevation_mapper
-- reimplemented independently here, not imported, per CONTRIBUTING 5. It has
to match exactly or the 3 input maps and the merged output won't line up. The
mcap save logic (rosbag2_py.SequentialWriter) similarly mirrors that
package's ground_elevation_map_saver, reimplemented rather than imported.
"""

from dataclasses import dataclass
import math
import os

from geometry_msgs.msg import Pose
from grid_map_msgs.msg import GridMap, GridMapInfo
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, String

ROBOTS = ('drone', 'wheel', 'leg')
# design.md 6-4 / 9-5: applied in this order, low -> high priority, so the
# later (higher-priority) valid values win. Deliberately a fixed tuple, not a
# dict/set, so the merge order is always deterministic.
MERGE_ORDER = ('drone', 'leg', 'wheel')

# README 3.1 / 3.3: single global `map` frame, fixed 0.10 m/cell resolution.
EXPECTED_FRAME_ID = 'map'
EXPECTED_RESOLUTION = 0.10
ELEVATION_LAYER = 'elevation'


@dataclass(frozen=True)
class OutputGrid:
    """Union output grid spec (design.md 6-3 / 9-4): origin + size, no data.

    origin_x/origin_y are the world (map-frame) coordinates of the grid's
    min-x/min-y corner -- same convention agconav_ground_mapping's
    ground_elevation_mapper uses internally for its own accumulation grid
    (reimplemented independently here, not imported, per CONTRIBUTING 5).
    """

    origin_x: float
    origin_y: float
    resolution: float
    n_rows: int
    n_cols: int


class MapMergeCollector(Node):
    """Buffers the 3 robots' elevation maps and fires the merge trigger once."""

    def __init__(self):
        super().__init__('map_merge_collector')

        # design.md "구현 시 참고사항": absolute topic defaults (single
        # system-wide node), still parameterized for flexibility.
        for robot in ROBOTS:
            self.declare_parameter(f'{robot}_elevation_map_topic', f'/{robot}/elevation_map')
            self.declare_parameter(f'{robot}_status_topic', f'/{robot}/elevation_map_status')
        # design.md 8-6 / 8-7: absolute /merged/... topics, single system-wide node.
        self.declare_parameter('merge_error_topic', '/merged/merge_error')
        self.declare_parameter('merge_status_topic', '/merged/merge_status')
        self.declare_parameter('merged_elevation_map_topic', '/merged/elevation_map')
        # design.md 9-4: reject the output grid if it would need more cells
        # than this. Default covers the full 500x500m world at 0.10 m/cell
        # (5000x5000 = 25,000,000 cells, README 2.2) with headroom.
        self.declare_parameter('max_grid_cells', 30_000_000)
        # design.md 6-5 / 9-7 parameter table. use_sim_time is declared
        # automatically by rclpy.Node, not redeclared here (README 3.4).
        self.declare_parameter('output_directory', 'maps')
        self.declare_parameter('map_name', 'merged_elevation_map')
        self.declare_parameter('output_format', 'mcap')

        self._maps = {robot: None for robot in ROBOTS}
        self._complete = {robot: False for robot in ROBOTS}
        self._merge_triggered = False
        self._validated_maps = None
        self._output_grid = None
        self._merged_map = None

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
        # design.md 9-5: merged elevation_map is reliable / transient_local /
        # keep_last / depth 1, same shape as the 3 input elevation_map topics.
        merged_map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._merge_error_pub = self.create_publisher(
            String, self.get_parameter('merge_error_topic').value, error_qos)
        self._merge_status_pub = self.create_publisher(
            Bool, self.get_parameter('merge_status_topic').value, status_result_qos)
        self._merged_map_pub = self.create_publisher(
            GridMap, self.get_parameter('merged_elevation_map_topic').value, merged_map_qos)

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
            self._merge_status_pub.publish(Bool(data=False))
            return

        self.get_logger().info(
            'all 3 elevation maps passed validation (frame/resolution/layer).')
        self._validated_maps = maps

        output_grid, error = self._build_output_grid(maps)
        if error is not None:
            self.get_logger().error(f'output grid computation failed, aborting merge: {error}')
            self._merge_error_pub.publish(String(data=error))
            self._merge_status_pub.publish(Bool(data=False))
            return

        self._output_grid = output_grid
        self.get_logger().info(
            f'output grid: origin=({output_grid.origin_x:.2f}, {output_grid.origin_y:.2f}) '
            f'size={output_grid.n_rows}x{output_grid.n_cols} cells '
            f'({output_grid.n_rows * output_grid.resolution:.1f} x '
            f'{output_grid.n_cols * output_grid.resolution:.1f} m)')

        # design.md 6-4 / 9-5: merge, publish once, and report success.
        elevation = self._merge_elevation(maps, output_grid)
        stamp = self.get_clock().now().to_msg()
        merged_map = self._build_merged_grid_map_message(elevation, output_grid, stamp)
        self._merged_map = merged_map
        self._merged_map_pub.publish(merged_map)
        self._merge_status_pub.publish(Bool(data=True))
        self.get_logger().info('published /merged/elevation_map -- merge complete (once).')

        # design.md 6-5 / 8-8 / 9-7: save the same merged map to mcap.
        self._save_merged_map(merged_map)
        self.get_logger().info(
            'merged map saved -- module E merge process is complete.')

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

    @staticmethod
    def _map_bounds(grid_map):
        """Return (min_x, max_x, min_y, max_y) of grid_map in the map frame.

        grid_map_msgs/GridMap stores a center pose + length, not corners
        (design.md 9-4: "원점 계산 기준 | 공통 map 좌표").
        """
        half_x = grid_map.info.length_x / 2.0
        half_y = grid_map.info.length_y / 2.0
        center_x = grid_map.info.pose.position.x
        center_y = grid_map.info.pose.position.y
        return center_x - half_x, center_x + half_x, center_y - half_y, center_y + half_y

    def _build_output_grid(self, maps):
        """Union bounding box of the 3 maps -> OutputGrid (design.md 6-3 / 9-4).

        Returns (OutputGrid, None) on success, or (None, error message) if
        the resulting grid would exceed max_grid_cells.
        """
        bounds = [self._map_bounds(maps[robot]) for robot in ROBOTS]
        min_x = min(b[0] for b in bounds)
        max_x = max(b[1] for b in bounds)
        min_y = min(b[2] for b in bounds)
        max_y = max(b[3] for b in bounds)

        resolution = EXPECTED_RESOLUTION
        n_rows = round((max_x - min_x) / resolution)
        n_cols = round((max_y - min_y) / resolution)
        total_cells = n_rows * n_cols

        max_cells = self.get_parameter('max_grid_cells').value
        if total_cells > max_cells:
            return None, (
                f'output grid would need {n_rows}x{n_cols}={total_cells} cells, '
                f'over the max_grid_cells limit ({max_cells})')

        return OutputGrid(
            origin_x=min_x, origin_y=min_y,
            resolution=resolution, n_rows=n_rows, n_cols=n_cols,
        ), None

    @staticmethod
    def _extract_elevation(grid_map):
        """Undo ground_elevation_mapper-style wire packing for one layer.

        Recovers (elevation, origin_x, origin_y) in the same "min-corner
        origin, row grows +x, col grows +y" convention the source node built
        it in, from the packed grid_map_msgs wire format (matrix index (0, 0)
        at the (+x, +y) corner, column-major flattened -- see
        agconav_ground_mapping's ground_elevation_mapper for the packing this
        undoes). Assumes outer_start_index/inner_start_index == 0, true for
        every elevation_map this project publishes.
        """
        layer_index = grid_map.layers.index(ELEVATION_LAYER)
        layer = grid_map.data[layer_index]
        n_rows = layer.layout.dim[0].size
        n_cols = layer.layout.dim[1].size
        gm_matrix = np.asarray(layer.data, dtype=np.float32).reshape((n_rows, n_cols), order='F')
        elevation = gm_matrix[::-1, ::-1]
        origin_x = grid_map.info.pose.position.x - grid_map.info.length_x / 2.0
        origin_y = grid_map.info.pose.position.y - grid_map.info.length_y / 2.0
        return elevation, origin_x, origin_y

    def _place_into_output(self, output, output_grid, grid_map):
        """Overwrite output's valid (non-NaN) cells with grid_map's, in place.

        design.md 6-4: "유효값을 NaN으로 덮지 않음" -- only cells where
        grid_map itself has a real value are written; everything else in
        output is left untouched. Called in MERGE_ORDER (drone, leg, wheel)
        so later calls take priority, per design.md's wheel > leg > drone
        rule.
        """
        elevation, origin_x, origin_y = self._extract_elevation(grid_map)
        n_rows, n_cols = elevation.shape
        row_offset = round((origin_x - output_grid.origin_x) / output_grid.resolution)
        col_offset = round((origin_y - output_grid.origin_y) / output_grid.resolution)
        region = output[row_offset:row_offset + n_rows, col_offset:col_offset + n_cols]
        valid = ~np.isnan(elevation)
        region[valid] = elevation[valid]

    def _merge_elevation(self, maps, output_grid):
        """Build the merged elevation array (design.md 6-4 / 9-5).

        Starts all-NaN so cells no robot ever observed stay NaN, then places
        drone, leg, wheel in that fixed order so the higher-priority valid
        values always win -- never dict/set iteration order.
        """
        output = np.full((output_grid.n_rows, output_grid.n_cols), np.nan, dtype=np.float32)
        for robot in MERGE_ORDER:
            self._place_into_output(output, output_grid, maps[robot])
        return output

    def _build_merged_grid_map_message(self, elevation, output_grid, stamp):
        """Pack the merged elevation array into a grid_map_msgs/GridMap.

        Mirrors ground_elevation_mapper's _build_grid_map_message packing
        (axis flip + column-major flatten), reimplemented independently here
        per CONTRIBUTING 5 -- see that node for the convention this matches.
        """
        n_rows, n_cols = elevation.shape
        length_x = n_rows * output_grid.resolution
        length_y = n_cols * output_grid.resolution

        gm_matrix = elevation[::-1, ::-1]
        elevation_layer = Float32MultiArray()
        elevation_layer.layout.dim = [
            MultiArrayDimension(label='column_index', size=n_rows, stride=n_rows * n_cols),
            MultiArrayDimension(label='row_index', size=n_cols, stride=n_rows),
        ]
        elevation_layer.data = gm_matrix.flatten(order='F').tolist()

        info = GridMapInfo()
        info.resolution = output_grid.resolution
        info.length_x = length_x
        info.length_y = length_y
        info.pose = Pose()
        info.pose.position.x = output_grid.origin_x + length_x / 2.0
        info.pose.position.y = output_grid.origin_y + length_y / 2.0
        info.pose.position.z = 0.0
        info.pose.orientation.w = 1.0

        grid_map = GridMap()
        grid_map.header.stamp = stamp
        grid_map.header.frame_id = EXPECTED_FRAME_ID
        grid_map.info = info
        grid_map.layers = [ELEVATION_LAYER]
        grid_map.basic_layers = [ELEVATION_LAYER]
        grid_map.data = [elevation_layer]
        grid_map.outer_start_index = 0
        grid_map.inner_start_index = 0
        return grid_map

    def _save_merged_map(self, merged_map):
        """Serialize merged_map to an mcap rosbag2 (design.md 6-5 / 9-7).

        Mirrors agconav_ground_mapping's ground_elevation_map_saver: never
        raises out of this method -- a save failure must not take down a
        node that already successfully merged and published the map.
        """
        output_directory = self.get_parameter('output_directory').value
        map_name = self.get_parameter('map_name').value
        output_format = self.get_parameter('output_format').value
        topic_name = self.get_parameter('merged_elevation_map_topic').value
        bag_path = os.path.join(output_directory, map_name)

        try:
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id=output_format),
                rosbag2_py.ConverterOptions('', ''),
            )
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=0,
                name=topic_name,
                type='grid_map_msgs/msg/GridMap',
                serialization_format='cdr',
            ))
            stamp = merged_map.header.stamp
            timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            writer.write(topic_name, serialize_message(merged_map), timestamp_ns)
            del writer  # flush/close the bag now, rather than at GC time
        except Exception as ex:
            self.get_logger().error(f'failed to save merged map to "{bag_path}": {ex}')
            return

        self.get_logger().info(f'saved merged map to "{bag_path}" ({output_format})')

    def get_collected_maps(self):
        """Return {robot: latest GridMap or None}, snapshotted at call time."""
        return dict(self._maps)

    def get_validated_maps(self):
        """Return the validated {robot: GridMap} set for map_merge_grid_builder.

        (6-3) Populated once the merge trigger has fired AND validation
        passed; stays None until then, and also None if validation failed.
        """
        return self._validated_maps

    def get_output_grid(self):
        """Return the union OutputGrid for elevation_map_merger.

        (6-4) Populated once validation AND grid-size checks both pass;
        None until then, and also None if either step failed.
        """
        return self._output_grid

    def get_merged_map(self):
        """Return the final merged GridMap once published (design.md 6-4).

        None until the merge has fully succeeded; stays None on any failure.
        """
        return self._merged_map

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
