"""Subscribes to a robot's raw LiDAR PointCloud2, transforms it into map, and
accumulates a 2.5D elevation grid, publishing the final map once on completion.

design.md 4-1 / 6-2 / 6-3 / 7-1 / 7-2: this node is launched once per
robot (wheel/leg), each in its own namespace, running the exact same code.
It subscribes to the raw sensor cloud directly, looks up the sensor-to-map
TF at the cloud's own stamp, transforms it into the map frame, and bins the
result into a resolution x resolution grid, keeping a running-average height
per cell. Unlike the earlier 4-node design, this absorbs what used to be two
separate nodes (`ground_pointcloud_collector` for receipt monitoring,
`ground_lidar_tf_transformer` for the TF lookup/transform) -- splitting the
TF transform into its own node meant re-publishing a full PointCloud2 (10Hz
x 32ch x 1024pts) purely to hand it to this node, for no benefit since
nothing else consumed the transformed cloud.

design.md 4-1 / 6-4 / 6-5 / 7-3 / 7-4: the elevation map itself is no
longer published on a 1Hz timer. It is published exactly once, when
`navigation_status` reports the robot's move as complete -- accumulation
keeps running regardless, but only that one final snapshot goes out.

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

design.md 4-2 (지도 저장 책임) now lives in ground_elevation_map_saver, not
here -- this node only ever does accumulation/publish.
"""

from geometry_msgs.msg import Pose
from grid_map_msgs.msg import GridMap, GridMapInfo
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class GroundElevationMapper(Node):
    """Transforms a robot's raw PointCloud2 into map and bins it into a grid."""

    def __init__(self):
        super().__init__('ground_elevation_mapper')

        # CONTRIBUTING 5: relative topic names, resolved by the launch
        # namespace (e.g. /wheel/points -> /wheel/elevation_map).
        self.declare_parameter('points_topic', 'points')
        self.declare_parameter('elevation_map_topic', 'elevation_map')
        self.declare_parameter('navigation_status_topic', 'navigation_status')
        # README 3.1: single global frame `map`.
        self.declare_parameter('target_frame', 'map')
        # 변경사항 5 검토 결과: /wheel/points, /leg/points의 실제 header.frame_id가
        # 사설 프레임인지 전역 프레임인지 이 환경에서 실측하지 못했고(브릿지 설정
        # 자체가 아직 리포에 없음), wheel/leg의 정적 분석 결과도 서로 달랐다. 실측
        # 없이 단정하는 대신 파라미터로 노출한다: 비워두면(기본값) 기존처럼
        # msg.header.frame_id를 신뢰하고, 값이 있으면 그 전역 프레임 이름으로
        # source_frame을 덮어쓴다. drone은 애초에 frame_id가 전역 이름이라 비워둔다.
        self.declare_parameter('target_source_frame', '')
        # README 3.3: fixed 0.10 m/cell resolution across all maps.
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('frame_id', 'map')
        # design.md 4-1: 수신 감시 책임(구 ground_pointcloud_collector)을 그대로
        # 이식. 원본 점군 수신이 끊겼는지 감지하는 헬스체크는 노드가 통합되어도
        # 여전히 유용하다는 판단(사용자 결정).
        self.declare_parameter('data_timeout_sec', 2.0)
        self.declare_parameter('check_period_sec', 1.0)

        points_topic = self.get_parameter('points_topic').value
        elevation_map_topic = self.get_parameter('elevation_map_topic').value
        navigation_status_topic = self.get_parameter('navigation_status_topic').value
        self._target_frame = self.get_parameter('target_frame').value
        self._target_source_frame = self.get_parameter('target_source_frame').value
        self._resolution = float(self.get_parameter('resolution').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._data_timeout = Duration(
            seconds=self.get_parameter('data_timeout_sec').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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
        self._last_received = None
        self._published = False

        # design.md 7-1: raw sensor cloud is best effort / volatile /
        # keep last / depth 5.
        points_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # design.md 7-3 / 7-4: elevation_map and navigation_status are both
        # one-shot signals (final map / completion), so both use reliable +
        # transient_local + keep_last + depth 1 -- a best_effort profile
        # would have no recovery if that single message were dropped. Same
        # reasoning as module E's merge_trigger.
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._points_sub = self.create_subscription(
            PointCloud2, points_topic, self._points_callback, points_qos)
        self._navigation_status_sub = self.create_subscription(
            Bool, navigation_status_topic, self._navigation_status_callback, latched_qos)
        self._elevation_map_pub = self.create_publisher(
            GridMap, elevation_map_topic, latched_qos)

        check_period = self.get_parameter('check_period_sec').value
        self._check_timer = self.create_timer(
            check_period, self._check_data_received)

        self.get_logger().info(
            f'Accumulating "{points_topic}" -> "{elevation_map_topic}" '
            f'(resolution={self._resolution} m/cell, target_frame='
            f'"{self._target_frame}"), publishing once on '
            f'"{navigation_status_topic}"')

    def _points_callback(self, msg):
        self._last_received = self.get_clock().now()

        source_frame = self._target_source_frame or msg.header.frame_id
        try:
            # design.md 7-2: look up the TF at the cloud's own measurement
            # stamp. No wait timeout: if it isn't available yet, drop this
            # cloud rather than blocking the single-threaded executor.
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, source_frame, Time.from_msg(msg.header.stamp))
        except TransformException as ex:
            self.get_logger().warn(
                f'TF lookup failed for "{source_frame}" -> '
                f'"{self._target_frame}" at {msg.header.stamp.sec}.'
                f'{msg.header.stamp.nanosec:09d}s, dropping cloud: {ex}')
            return

        # only frame_id changes on this internal map-frame conversion,
        # original stamp is kept (no longer a separate published topic).
        cloud_map = do_transform_cloud(msg, transform)
        cloud_map.header.stamp = msg.header.stamp
        cloud_map.header.frame_id = self._target_frame
        self._accumulate(cloud_map)

    def _accumulate(self, msg):
        points = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.shape[0] == 0:
            return

        xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
        row_idx = np.floor((xs - self._origin_x) / self._resolution).astype(np.int64)
        col_idx = np.floor((ys - self._origin_y) / self._resolution).astype(np.int64)
        row_idx, col_idx = self._grow_to_fit(row_idx, col_idx)

        # design.md 3/4-1: bin points into cells and accumulate ("누적") a
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

    def _navigation_status_callback(self, msg):
        # design.md 4-1 / 6-4 / 6-5: publish the accumulated map exactly once,
        # the moment navigation reports completion. Further True messages
        # (or a late-joining latched redelivery) must not republish.
        if not msg.data or self._published:
            return
        if self._sum is None:
            self.get_logger().warn(
                'navigation_status reported complete but no points were '
                'accumulated yet, nothing to publish.')
            return
        self._elevation_map_pub.publish(self._build_grid_map_message())
        self._published = True

    def _check_data_received(self):
        if self._last_received is None:
            self.get_logger().warn('no point cloud received yet.')
            return
        elapsed = self.get_clock().now() - self._last_received
        if elapsed > self._data_timeout:
            self.get_logger().warn(
                f'no point cloud in the last {elapsed.nanoseconds / 1e9:.2f}s '
                '(topic may be stalled).')

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
