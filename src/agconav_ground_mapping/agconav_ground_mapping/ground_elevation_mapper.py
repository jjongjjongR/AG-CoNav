"""Accumulates a robot's map-frame point cloud into a 2.5D elevation grid.

design.md 4-3 / 6-5 / 7-4: this node is launched once per robot (wheel/leg),
each in its own namespace, running the exact same code. It subscribes to the
map-frame point cloud published by ground_lidar_tf_transformer, bins points
into a resolution x resolution grid, and republishes the running-average
height per cell as a grid_map_msgs/GridMap with an `elevation` layer.

The grid is implemented directly with numpy rather than the grid_map C++/
Python bindings: this node only ever needs one layer (`elevation`), plain
running-average accumulation, and dynamic growth -- none of which benefit
from grid_map's iterator/interpolation machinery. Pulling in the grid_map
library just to wrap a single Eigen matrix would add a large native
dependency for no functional gain, and numpy already gives fast vectorized
binning (`np.bincount`) and resizing (`np.pad`). We do still have to emit a
spec-correct `grid_map_msgs/msg/GridMap` message by hand -- see
`_build_grid_map_message` for the wire-format details and a note on what to
verify once this runs against a real grid_map consumer (RViz2 / a future
map-fusion module).

design.md 4-4 / 6-7 / 7-6 (완료 상태 제공 책임) now lives in
ground_elevation_map_saver, not here -- this node only ever does
accumulation/publish.
"""

from geometry_msgs.msg import Pose
from grid_map_msgs.msg import GridMap, GridMapInfo
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class GroundElevationMapper(Node):
    """Bins a robot's map-frame point cloud into a growing elevation grid."""

    def __init__(self):
        super().__init__('ground_elevation_mapper')

        # CONTRIBUTING 5: relative topic names, resolved by the launch
        # namespace (e.g. /wheel/points_map -> /wheel/elevation_map).
        self.declare_parameter('points_map_topic', 'points_map')
        self.declare_parameter('elevation_map_topic', 'elevation_map')
        # README 3.3: fixed 0.10 m/cell resolution across all maps.
        self.declare_parameter('resolution', 0.10)
        # design.md 7-4: default 1 Hz publish rate.
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('frame_id', 'map')

        points_map_topic = self.get_parameter('points_map_topic').value
        elevation_map_topic = self.get_parameter('elevation_map_topic').value
        self._resolution = float(self.get_parameter('resolution').value)
        self._frame_id = self.get_parameter('frame_id').value

        # Grid state, in OUR OWN convention (not grid_map's wire convention,
        # see _build_grid_map_message): (row, col) = (0, 0) is the min-x/
        # min-y corner of the observed area; row grows with +x, col grows
        # with +y. None until the first point arrives -- shape and origin
        # both grow on demand (README 3.3: dynamic extent, not fixed size).
        self._sum = None
        self._count = None
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._last_stamp = None

        # design.md 7-3: input matches ground_lidar_tf_transformer's output QoS.
        input_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # design.md 7-4: output is reliable + transient_local, depth 1.
        output_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._points_map_sub = self.create_subscription(
            PointCloud2, points_map_topic, self._points_map_callback, input_qos)
        self._elevation_map_pub = self.create_publisher(
            GridMap, elevation_map_topic, output_qos)

        publish_period = self.get_parameter('publish_period_sec').value
        self._publish_timer = self.create_timer(
            publish_period, self._publish_elevation_map)

        self.get_logger().info(
            f'Accumulating "{points_map_topic}" -> "{elevation_map_topic}" '
            f'(resolution={self._resolution} m/cell)')

    def _points_map_callback(self, msg):
        points = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.shape[0] == 0:
            return

        xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
        row_idx = np.floor((xs - self._origin_x) / self._resolution).astype(np.int64)
        col_idx = np.floor((ys - self._origin_y) / self._resolution).astype(np.int64)
        row_idx, col_idx = self._grow_to_fit(row_idx, col_idx)

        # design.md 3/4-3: bin points into cells and accumulate ("누적") a
        # running average height per cell, vectorized via bincount.
        n_rows, n_cols = self._sum.shape
        cell_count = n_rows * n_cols
        flat_idx = row_idx * n_cols + col_idx
        self._sum += np.bincount(
            flat_idx, weights=zs, minlength=cell_count).reshape(n_rows, n_cols)
        self._count += np.bincount(
            flat_idx, minlength=cell_count).reshape(n_rows, n_cols)

        self._last_stamp = msg.header.stamp

    def _grow_to_fit(self, row_idx, col_idx):
        """Pad the grid so row_idx/col_idx fit, remapped into the new array.

        README 3.3: the map only grows to cover what has actually been
        observed, it is never pre-sized.
        """
        if self._sum is None:
            min_row, max_row = int(row_idx.min()), int(row_idx.max())
            min_col, max_col = int(col_idx.min()), int(col_idx.max())
            self._sum = np.zeros((max_row - min_row + 1, max_col - min_col + 1))
            self._count = np.zeros_like(self._sum)
            self._origin_x += min_row * self._resolution
            self._origin_y += min_col * self._resolution
            return row_idx - min_row, col_idx - min_col

        n_rows, n_cols = self._sum.shape
        pad_before_row = max(0, -int(row_idx.min()))
        pad_after_row = max(0, int(row_idx.max()) - (n_rows - 1))
        pad_before_col = max(0, -int(col_idx.min()))
        pad_after_col = max(0, int(col_idx.max()) - (n_cols - 1))

        if pad_before_row or pad_after_row or pad_before_col or pad_after_col:
            pad_width = ((pad_before_row, pad_after_row), (pad_before_col, pad_after_col))
            self._sum = np.pad(self._sum, pad_width)
            self._count = np.pad(self._count, pad_width)
            self._origin_x -= pad_before_row * self._resolution
            self._origin_y -= pad_before_col * self._resolution
            row_idx = row_idx + pad_before_row
            col_idx = col_idx + pad_before_col

        return row_idx, col_idx

    def _publish_elevation_map(self):
        if self._sum is None:
            return
        self._elevation_map_pub.publish(self._build_grid_map_message())

    def _build_grid_map_message(self):
        n_rows, n_cols = self._sum.shape
        # README 3.3: unobserved cells stay NaN. count == 0 makes this a
        # 0/0 division, which numpy already turns into NaN; the warning is
        # expected and suppressed rather than worked around.
        with np.errstate(invalid='ignore'):
            elevation = (self._sum / self._count).astype(np.float32)

        length_x = n_rows * self._resolution
        length_y = n_cols * self._resolution

        # --- grid_map wire-format packing -----------------------------------
        # grid_map's convention (grid_map_ros GridMapRosConverter): matrix
        # index (0, 0) sits at the map's (+x, +y) corner, index rows grow
        # toward -x and columns grow toward -y. Our own array instead grows
        # with +x/+y from a min-corner origin (see _grow_to_fit), so both
        # axes are flipped before packing. Data is flattened column-major
        # (Eigen's default storage order), matching
        # matrixEigenCopyToMultiArrayMessage in grid_map_ros.
        # NOTE: this layout is implemented from the documented grid_map
        # convention, not runtime-verified in this sandbox (no ROS2/RViz2
        # available here) -- confirm cell orientation in RViz2's
        # grid_map_rviz_plugin once this runs in the real Jazzy environment.
        gm_matrix = elevation[::-1, ::-1]
        elevation_layer = Float32MultiArray()
        elevation_layer.layout.dim = [
            MultiArrayDimension(label='column_index', size=n_rows, stride=n_rows * n_cols),
            MultiArrayDimension(label='row_index', size=n_cols, stride=n_rows),
        ]
        elevation_layer.data = gm_matrix.flatten(order='F').tolist()
        # ---------------------------------------------------------------------

        info = GridMapInfo()
        info.resolution = self._resolution
        info.length_x = length_x
        info.length_y = length_y
        info.pose = Pose()
        info.pose.position.x = self._origin_x + length_x / 2.0
        info.pose.position.y = self._origin_y + length_y / 2.0
        info.pose.position.z = 0.0
        info.pose.orientation.w = 1.0

        grid_map = GridMap()
        grid_map.header.stamp = self._last_stamp
        grid_map.header.frame_id = self._frame_id
        grid_map.info = info
        grid_map.layers = ['elevation']
        grid_map.basic_layers = ['elevation']
        grid_map.data = [elevation_layer]
        grid_map.outer_start_index = 0
        grid_map.inner_start_index = 0
        return grid_map


def main(args=None):
    rclpy.init(args=args)
    node = GroundElevationMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
