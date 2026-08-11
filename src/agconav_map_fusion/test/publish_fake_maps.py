#!/usr/bin/env python3
"""Manual validation script: feeds fake elevation maps into module E's 3 nodes.

Not a pytest test (no `test_` prefix, not collected by pytest) and not a
production node -- it exists purely so a developer can run
`map_merge_collector` / `elevation_map_merger` / `merged_elevation_map_saver`
against something without needing drone/wheel/leg's real mapping pipelines
up. Run it directly:

    python3 test/publish_fake_maps.py

It publishes a small 5x5 grid_map_msgs/GridMap on each of /drone, /wheel,
/leg's elevation_map topic immediately, then after 3 seconds publishes
Bool(True) on wheel and leg's elevation_map_status topics only -- NOT
drone's. map_merge_collector no longer subscribes to
/drone/elevation_map_status at all (design.md: drone's map generation is
structurally guaranteed to already be done by the time wheel/leg finish
moving, so its own completion signal is redundant as a trigger input) --
this script mirrors that by simply never sending it. This is the same
input sequence map_merge_collector expects before it fires merge_trigger
(design.md 6-1). QoS on every topic matches map_merge_collector's /
elevation_map_merger's subscriptions (reliable / transient_local /
keep_last / depth 1), and the GridMap wire packing mirrors
elevation_map_merger's own build_merged_grid_map_message (axis flip,
column-major flatten) -- reimplemented independently here per
CONTRIBUTING 5, not imported.

wheel and leg's elevation_map also carries a second layer,
`elevation_variance`, with independent random per-cell values -- mirroring
what agconav_ground_mapping's ground_elevation_mapper now actually
publishes (its per-cell Kalman filter's uncertainty), so
elevation_map_merger's variance-based wheel/leg cell selection
(design.md 6-4) has something real to compare. drone's map intentionally
has no `elevation_variance` layer, matching production -- module A is out
of this change's scope, and elevation_map_merger must not crash or treat
drone specially when the layer is simply absent (grid_math.py's
extract_elevation_variance returns None for it instead).

The node keeps spinning after publishing so its transient_local history
stays available to nodes started later; stop it with Ctrl+C once the merge
has been observed.
"""

from geometry_msgs.msg import Pose
from grid_map_msgs.msg import GridMap, GridMapInfo
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension

ROBOTS = ('drone', 'wheel', 'leg')
# map_merge_collector's merge trigger only ever waits on wheel/leg -- see
# map_merge_collector.py's module docstring for why drone is excluded.
STATUS_ROBOTS = ('wheel', 'leg')
# Robots that publish an `elevation_variance` layer -- drone doesn't (see
# module docstring), so it's excluded here, mirroring production.
VARIANCE_ROBOTS = ('wheel', 'leg')
# Distinct per-robot base elevation so which of wheel/leg's values won the
# variance-based selection (design.md 6-4), and drone's fallback, are easy
# to eyeball in the merged result.
BASE_ELEVATION = {'drone': 10.0, 'wheel': 20.0, 'leg': 30.0}
NAN_FRACTION = 0.2
# Per-cell variance sampled uniformly from this range for wheel/leg,
# independently -- wide enough that both "wheel wins" and "leg wins" cells
# show up in a single run.
VARIANCE_RANGE = (0.01, 1.0)

GRID_SIZE = 5
RESOLUTION = 0.10
FRAME_ID = 'map'
ELEVATION_LAYER = 'elevation'
ELEVATION_VARIANCE_LAYER = 'elevation_variance'
STATUS_DELAY_SEC = 3.0


def _build_fake_elevation(base_elevation, rng):
    """Return a GRID_SIZE x GRID_SIZE float32 array, some cells NaN."""
    elevation = (base_elevation + rng.uniform(0.0, 1.0, size=(GRID_SIZE, GRID_SIZE)))
    elevation = elevation.astype(np.float32)
    elevation[rng.random((GRID_SIZE, GRID_SIZE)) < NAN_FRACTION] = np.nan
    return elevation


def _build_fake_variance(rng):
    """Return a GRID_SIZE x GRID_SIZE float32 array of per-cell variance."""
    return rng.uniform(*VARIANCE_RANGE, size=(GRID_SIZE, GRID_SIZE)).astype(np.float32)


def _pack_layer(matrix, n_rows, n_cols):
    """Pack one layer's (n_rows, n_cols) float32 array into the wire format.

    Mirrors grid_math.build_merged_grid_map_message's packing (axis flip +
    column-major flatten) so the message round-trips through
    grid_math.extract_elevation/extract_elevation_variance exactly as a real
    elevation_map would.
    """
    gm_matrix = matrix[::-1, ::-1]
    layer = Float32MultiArray()
    # std_msgs/MultiArrayLayout: dim[0]=열(column_index), dim[1]=행(row_index),
    # 최내곽은 stride == size.
    layer.layout.dim = [
        MultiArrayDimension(label='column_index', size=n_cols, stride=n_rows * n_cols),
        MultiArrayDimension(label='row_index', size=n_rows, stride=n_rows),
    ]
    layer.data = gm_matrix.flatten(order='F').tolist()
    return layer


def _build_grid_map_message(elevation, stamp, variance=None):
    """Pack `elevation` (and optionally `variance`) into a GridMap, centered on the origin.

    `variance`, when given, is packed as a second `elevation_variance` layer
    -- omitted for drone (see module docstring), matching what
    ground_elevation_mapper actually publishes for wheel/leg.
    """
    n_rows, n_cols = elevation.shape
    length_x = n_rows * RESOLUTION
    length_y = n_cols * RESOLUTION

    layers = [ELEVATION_LAYER]
    data = [_pack_layer(elevation, n_rows, n_cols)]
    if variance is not None:
        layers.append(ELEVATION_VARIANCE_LAYER)
        data.append(_pack_layer(variance, n_rows, n_cols))

    info = GridMapInfo()
    info.resolution = RESOLUTION
    info.length_x = length_x
    info.length_y = length_y
    info.pose = Pose()
    info.pose.position.x = 0.0
    info.pose.position.y = 0.0
    info.pose.position.z = 0.0
    info.pose.orientation.w = 1.0

    grid_map = GridMap()
    grid_map.header.stamp = stamp
    grid_map.header.frame_id = FRAME_ID
    grid_map.info = info
    grid_map.layers = layers
    grid_map.basic_layers = [ELEVATION_LAYER]
    grid_map.data = data
    grid_map.outer_start_index = 0
    grid_map.inner_start_index = 0
    return grid_map


class FakeMapPublisher(Node):
    """Publishes fake elevation maps, then a delayed completion status, once."""

    def __init__(self):
        super().__init__('publish_fake_maps')

        # map_merge_collector's / elevation_map_merger's map_qos, and
        # map_merge_collector's status_qos (design.md 9-1 / 9-2).
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._map_pubs = {
            robot: self.create_publisher(GridMap, f'/{robot}/elevation_map', map_qos)
            for robot in ROBOTS
        }
        self._status_pubs = {
            robot: self.create_publisher(Bool, f'/{robot}/elevation_map_status', status_qos)
            for robot in STATUS_ROBOTS
        }

        self._publish_fake_maps()
        self._status_timer = self.create_timer(STATUS_DELAY_SEC, self._publish_status_once)

    def _publish_fake_maps(self):
        rng = np.random.default_rng()
        stamp = self.get_clock().now().to_msg()
        for robot in ROBOTS:
            elevation = _build_fake_elevation(BASE_ELEVATION[robot], rng)
            variance = _build_fake_variance(rng) if robot in VARIANCE_ROBOTS else None
            grid_map = _build_grid_map_message(elevation, stamp, variance)
            self._map_pubs[robot].publish(grid_map)
            layer_desc = (
                f'{len(grid_map.layers)} layers {grid_map.layers}' if variance is not None
                else f'{len(grid_map.layers)} layer {grid_map.layers}')
            self.get_logger().info(
                f'{robot}: published fake {GRID_SIZE}x{GRID_SIZE} elevation_map '
                f'(base={BASE_ELEVATION[robot]}, resolution={RESOLUTION}, {layer_desc})')
        self.get_logger().info(
            f'all 3 fake elevation maps published -- status will follow in '
            f'{STATUS_DELAY_SEC:.0f}s.')

    def _publish_status_once(self):
        self._status_timer.cancel()
        for robot in STATUS_ROBOTS:
            self._status_pubs[robot].publish(Bool(data=True))
            self.get_logger().info(f'{robot}: published elevation_map_status=True')
        self.get_logger().info(
            'wheel/leg status topics published (drone status intentionally never '
            'sent -- map_merge_collector does not subscribe to it). keeping node '
            'alive so transient_local history stays available -- Ctrl+C to stop.')


def main(args=None):
    rclpy.init(args=args)
    node = FakeMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
