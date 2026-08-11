"""Subscribes to a robot's raw LiDAR PointCloud2, transforms it into map, and
accumulates a 2.5D elevation grid, publishing the final map once on completion.

design.md 4-1 / 6-2 / 6-3 / 7-1 / 7-2: this node is launched once per
robot (wheel/leg), each in its own namespace, running the exact same code.
It subscribes to the raw sensor cloud directly, looks up the sensor-to-map
TF at the cloud's own stamp, transforms it into the map frame, and bins the
result into a resolution x resolution grid, fusing each cell's height with
a per-cell scalar Kalman filter (see `_kalman_update_cells`) rather than a
plain running average -- this also yields a per-cell variance estimate,
published as the `elevation_variance` layer for module E to use when
choosing between overlapping wheel/leg observations. Unlike the earlier
4-node design, this absorbs what used to be two separate nodes
(`ground_pointcloud_collector` for receipt monitoring,
`ground_lidar_tf_transformer` for the TF lookup/transform) -- splitting the
TF transform into its own node meant re-publishing a full PointCloud2 (10Hz
x 32ch x 1024pts) purely to hand it to this node, for no benefit since
nothing else consumed the transformed cloud.

design.md 4-1 / 6-4 / 6-5 / 7-3 / 7-4: the elevation map itself is no
longer published on a 1Hz timer. It is published exactly once, when
`navigation_status` reports the robot's move as complete -- accumulation
keeps running regardless, but only that one final snapshot goes out.

The grid is implemented directly with numpy rather than the grid_map C++/
Python bindings: this node only ever needs two layers (`elevation`,
`elevation_variance`), per-cell Kalman filtering, and dynamic growth --
none of which benefit from grid_map's iterator/interpolation machinery.
Pulling in the grid_map
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
        # 칼만필터 거리 기반 측정 노이즈(R) 파라미터 -- wheel(A300 lidar3d_0)과
        # leg(Go2 OS1-32) 모두 데이터시트상 동일한 구간별 정밀도(1시그마) 스펙을
        # 가진다는 게 확인되어, 계수 기반 근사식 대신 실측 구간표를 그대로 쓴다
        # (measurement_noise_max_distances[i]까지의 거리에는
        # measurement_noise_sigmas[i]가 적용되는 구간별 조회 방식 -- 자세한
        # 계산은 _measurement_noise 참고). 그래도 wheel/leg가 서로 다른 LiDAR
        # 기종인 건 변함없으므로 로봇별 yaml에서 각각 독립적으로 채운다.
        self.declare_parameter(
            'measurement_noise_max_distances', [1.0, 20.0, 50.0, 100.0])
        self.declare_parameter(
            'measurement_noise_sigmas', [0.007, 0.010, 0.020, 0.050])
        # cos(80°) ≈ 0.17 -- grazing angle(입사각이 90°에 가까워질 때) 근처에서
        # R_point가 1/cos_theta**2로 발산하는 것을 막는 하한.
        self.declare_parameter('incidence_cos_floor', 0.17)
        # 카이제곱분포 자유도 1, 유의수준 약 0.27%(대략 3-시그마)에 해당하는
        # 표준 게이팅 임계값 (Bar-Shalom, "Estimation with Applications to
        # Tracking and Navigation"의 고전적 추적이론에서 흔히 쓰이는 값).
        self.declare_parameter('innovation_gate_threshold', 9.0)

        points_topic = self.get_parameter('points_topic').value
        elevation_map_topic = self.get_parameter('elevation_map_topic').value
        navigation_status_topic = self.get_parameter('navigation_status_topic').value
        self._target_frame = self.get_parameter('target_frame').value
        self._target_source_frame = self.get_parameter('target_source_frame').value
        self._resolution = float(self.get_parameter('resolution').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._data_timeout = Duration(
            seconds=self.get_parameter('data_timeout_sec').value)
        self._measurement_noise_max_distances = np.array(
            self.get_parameter('measurement_noise_max_distances').value, dtype=np.float64)
        self._measurement_noise_sigmas = np.array(
            self.get_parameter('measurement_noise_sigmas').value, dtype=np.float64)
        # 운영 중 조용히 잘못된 구간표로 계산하느니 시작 시점에 바로 죽는 게
        # 낫다 -- 두 배열 길이가 안 맞거나 max_distances가 정렬돼 있지 않으면
        # np.searchsorted(_measurement_noise 참고)의 결과가 의미 없어진다.
        if len(self._measurement_noise_max_distances) != len(self._measurement_noise_sigmas):
            raise ValueError(
                'measurement_noise_max_distances and measurement_noise_sigmas must have '
                f'the same length, got {len(self._measurement_noise_max_distances)} and '
                f'{len(self._measurement_noise_sigmas)}')
        if np.any(np.diff(self._measurement_noise_max_distances) <= 0):
            raise ValueError(
                'measurement_noise_max_distances must be strictly increasing, got '
                f'{self._measurement_noise_max_distances.tolist()}')
        self._incidence_cos_floor = float(self.get_parameter('incidence_cos_floor').value)
        self._innovation_gate_threshold = float(
            self.get_parameter('innovation_gate_threshold').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Grid state, in OUR OWN convention (not grid_map's wire convention,
        # see _build_grid_map_message): (row, col) = (0, 0) is the min-x/
        # min-y corner of the observed area; row grows with +x, col grows
        # with +y. None until the first point arrives -- shape and origin
        # both grow on demand (README 3.3: dynamic extent, not fixed size).
        # self._elevation (x) is the per-cell best height estimate, and
        # self._variance (P) its uncertainty -- together the state of an
        # independent per-cell scalar Kalman filter (design.md 6-2/7-3,
        # see _kalman_update_cells). Unobserved cells are NaN, never 0 --
        # 0 would silently claim "observed, height 0m", and (critically for
        # the variance array) a padded 0 variance would mean "perfectly
        # certain", making the Kalman gain K = P/(P+R) permanently 0 for
        # that cell (see _grow_to_fit).
        self._elevation = None
        self._variance = None
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
        sensor_origin = transform.transform.translation
        self._accumulate(cloud_map, (sensor_origin.x, sensor_origin.y, sensor_origin.z))

    def _accumulate(self, msg, sensor_origin):
        points = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.shape[0] == 0:
            return

        xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
        row_idx = np.floor((xs - self._origin_x) / self._resolution).astype(np.int64)
        col_idx = np.floor((ys - self._origin_y) / self._resolution).astype(np.int64)
        row_idx, col_idx = self._grow_to_fit(row_idx, col_idx)

        # design.md 3/4-1: bin points into per-cell sums via bincount, one
        # batch (this callback's scan) at a time -- the Kalman update itself
        # (2-2: batch unit, not point-by-point) happens once per cell below,
        # not once per point.
        n_rows, n_cols = self._elevation.shape
        cell_count = n_rows * n_cols
        flat_idx = row_idx * n_cols + col_idx
        batch_count = np.bincount(flat_idx, minlength=cell_count).reshape(n_rows, n_cols)
        batch_sum_x = np.bincount(
            flat_idx, weights=xs, minlength=cell_count).reshape(n_rows, n_cols)
        batch_sum_y = np.bincount(
            flat_idx, weights=ys, minlength=cell_count).reshape(n_rows, n_cols)
        batch_sum_z = np.bincount(
            flat_idx, weights=zs, minlength=cell_count).reshape(n_rows, n_cols)

        # np.nonzero gives explicit (row, col) index arrays rather than a
        # boolean mask, so every array built from them below via fancy
        # indexing is its own independent copy -- no risk of the aliasing/
        # view bugs a chain of boolean masks on the same base array can
        # cause.
        touched_rows, touched_cols = np.nonzero(batch_count)
        if touched_rows.size == 0:
            return

        counts = batch_count[touched_rows, touched_cols]
        batch_mean_x = batch_sum_x[touched_rows, touched_cols] / counts
        batch_mean_y = batch_sum_y[touched_rows, touched_cols] / counts
        batch_mean_z = batch_sum_z[touched_rows, touched_cols] / counts

        r_eff = self._measurement_noise(
            touched_rows, touched_cols, batch_mean_x, batch_mean_y, batch_mean_z,
            counts, sensor_origin)
        self._kalman_update_cells(touched_rows, touched_cols, batch_mean_z, r_eff)

        self._last_stamp = msg.header.stamp

    def _measurement_noise(
            self, rows, cols, mean_x, mean_y, mean_z, counts, sensor_origin):
        """Return R_eff (design.md 2-2) for each of the given touched cells.

        R_point combines distance, incidence angle, and point density; R_eff
        divides it by how many of this batch's points landed in that cell.
        """
        sx, sy, sz = sensor_origin
        distance = np.sqrt((mean_x - sx) ** 2 + (mean_y - sy) ** 2 + (mean_z - sz) ** 2)

        # design.md 2-3: approximate each cell's surface normal from the
        # *previously accumulated* self._elevation via np.gradient -- no
        # per-point neighbor search or point-cloud library, just the
        # already-binned height grid. Computed before this batch's own
        # Kalman update below, so it reflects prior scans only.
        if self._elevation.shape[0] < 2 or self._elevation.shape[1] < 2:
            # np.gradient raises (not NaN) when an axis is too short to
            # differentiate along -- a real possibility early on, e.g. the
            # very first scan's points all landing in a single row/column
            # of cells. Treat it the same as a gradient full of NaN: no
            # neighbor info yet, fall back to cos_theta=1.0 below.
            dzdx = np.full_like(self._elevation, np.nan)
            dzdy = np.full_like(self._elevation, np.nan)
        else:
            with np.errstate(invalid='ignore'):
                dzdx, dzdy = np.gradient(self._elevation, self._resolution)
        normal_norm = np.sqrt(dzdx ** 2 + dzdy ** 2 + 1.0)
        normal_x = -dzdx / normal_norm
        normal_y = -dzdy / normal_norm
        normal_z = 1.0 / normal_norm

        ray_x, ray_y, ray_z = mean_x - sx, mean_y - sy, mean_z - sz
        ray_norm = np.sqrt(ray_x ** 2 + ray_y ** 2 + ray_z ** 2)
        with np.errstate(invalid='ignore'):
            cos_theta = np.abs(
                (ray_x / ray_norm) * normal_x[rows, cols]
                + (ray_y / ray_norm) * normal_y[rows, cols]
                + (ray_z / ray_norm) * normal_z[rows, cols])
        # No neighbor info to derive a normal from -- either this cell has
        # never been observed before (self._elevation is NaN there) or its
        # neighbors don't give np.gradient enough to work with (result is
        # NaN). Fall back to "hit straight-on" (cos_theta=1.0), the most
        # conservative assumption (smallest possible R_point contribution
        # from incidence). In practice this makes incidence correction a
        # no-op through the whole cold-start phase of the map, since almost
        # every cell is being seen for the first time.
        cos_theta = np.where(np.isnan(cos_theta), 1.0, cos_theta)
        cos_theta = np.clip(cos_theta, self._incidence_cos_floor, 1.0)

        # 거리 기반 R: 계수 근사식이 아니라 데이터시트 실측 구간별 정밀도(1시그마)
        # 조회. np.searchsorted(..., side='left')로 distance가 속하는 구간을
        # 찾고, np.clip으로 마지막 구간 밖(> max_distances[-1])도 마지막 구간의
        # 시그마로 클램프한다 -- 이 프로젝트가 쓰는 OS1-32는 최대 사거리가
        # 90~170m라 100m(마지막 구간 상한)를 넘는 관측이 실제로 들어올 수 있다.
        idx = np.searchsorted(self._measurement_noise_max_distances, distance, side='left')
        idx = np.clip(idx, 0, len(self._measurement_noise_sigmas) - 1)
        sigma_distance = self._measurement_noise_sigmas[idx]
        r_distance = sigma_distance ** 2

        r_point = r_distance / cos_theta ** 2
        return r_point / counts

    def _kalman_update_cells(self, rows, cols, batch_mean, r_eff):
        """Fuse this batch's per-cell mean height into self._elevation/_variance.

        Standard (linear) Kalman filter, not EKF/UKF -- no approximation is
        needed since there is no state transition (Q=0 below) and the
        observation model is already exactly linear (z = x + noise). Process
        noise Q is fixed at 0 rather than exposed as a parameter: the terrain
        is assumed static for the duration of one robot's mapping run, so
        there is nothing for a predict step to model between scans.
        """
        x_prev = self._elevation[rows, cols]
        p_prev = self._variance[rows, cols]

        # A cell with P still NaN has never been observed before -- initialize
        # it directly from this batch, with no gating (there is no prior
        # estimate yet to gate against).
        is_new = np.isnan(p_prev)
        new_rows, new_cols = rows[is_new], cols[is_new]
        self._elevation[new_rows, new_cols] = batch_mean[is_new]
        self._variance[new_rows, new_cols] = r_eff[is_new]

        is_existing = ~is_new
        ex_rows, ex_cols = rows[is_existing], cols[is_existing]
        x, p, r, z = (
            x_prev[is_existing], p_prev[is_existing],
            r_eff[is_existing], batch_mean[is_existing])

        # Innovation gating (design.md 2-2): reject outlier batches on cells
        # that already have an estimate, rather than letting a stray point
        # corrupt the filter. Threshold 9.0 -- chi-squared, 1 degree of
        # freedom, ~3-sigma equivalent, the standard gate from classical
        # tracking theory (Bar-Shalom).
        innovation = z - x
        innovation_covariance = p + r
        passed_gate = (innovation ** 2 / innovation_covariance) <= self._innovation_gate_threshold

        upd_rows, upd_cols = ex_rows[passed_gate], ex_cols[passed_gate]
        gain = p[passed_gate] / innovation_covariance[passed_gate]
        self._elevation[upd_rows, upd_cols] = x[passed_gate] + gain * innovation[passed_gate]
        self._variance[upd_rows, upd_cols] = (1.0 - gain) * p[passed_gate]
        # Cells that failed the gate are left untouched -- x, P both keep
        # their prior values, and this batch's reading for them is dropped.

    def _grow_to_fit(self, row_idx, col_idx):
        """Pad the grid so row_idx/col_idx fit, remapped into the new array.

        README 3.3: the map only grows to cover what has actually been
        observed, it is never pre-sized.
        """
        if self._elevation is None:
            min_row, max_row = int(row_idx.min()), int(row_idx.max())
            min_col, max_col = int(col_idx.min()), int(col_idx.max())
            shape = (max_row - min_row + 1, max_col - min_col + 1)
            # NaN, not 0 -- see the note on self._elevation/_variance in
            # __init__ for why a 0-filled variance array is a critical bug
            # here (permanently rejects the Kalman gain on every new cell).
            self._elevation = np.full(shape, np.nan)
            self._variance = np.full(shape, np.nan)
            self._origin_x += min_row * self._resolution
            self._origin_y += min_col * self._resolution
            return row_idx - min_row, col_idx - min_col

        n_rows, n_cols = self._elevation.shape
        pad_before_row = max(0, -int(row_idx.min()))
        pad_after_row = max(0, int(row_idx.max()) - (n_rows - 1))
        pad_before_col = max(0, -int(col_idx.min()))
        pad_after_col = max(0, int(col_idx.max()) - (n_cols - 1))

        if pad_before_row or pad_after_row or pad_before_col or pad_after_col:
            pad_width = ((pad_before_row, pad_after_row), (pad_before_col, pad_after_col))
            # constant_values=np.nan (not the np.pad default of 0) -- same
            # reasoning as the fresh-array case above.
            self._elevation = np.pad(self._elevation, pad_width, constant_values=np.nan)
            self._variance = np.pad(self._variance, pad_width, constant_values=np.nan)
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
        if self._elevation is None:
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
        n_rows, n_cols = self._elevation.shape
        # README 3.3: unobserved cells stay NaN -- self._elevation/_variance
        # already carry NaN for every never-observed cell (see __init__ /
        # _grow_to_fit), so no extra division-by-count step is needed here
        # the way the old running-average version required.
        elevation = self._elevation.astype(np.float32)
        variance = self._variance.astype(np.float32)

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
        # 축 뒤집기와 column-major 평탄화는 아직 실환경에서 검증되지 않았다 --
        # 이 프로젝트는 ROS2/RViz2가 없는 샌드박스에서 작업 중이라 런타임
        # 검증 자체가 불가능했다. 실제 Jazzy 환경에서 grid_map_rviz_plugin으로
        # 셀 방향(orientation)이 올바른지 재확인 필요.
        # dim 크기는 std_msgs/MultiArrayLayout 규약을 따른다: 차원은 바깥->안
        # 순서이고, 최내곽 차원은 stride == size 여야 한다. Eigen 열 우선 저장
        # 기준으로 바깥 차원이 열(column_index), 안쪽 차원이 행(row_index)이므로
        # dim[0].size = 열 개수, dim[1].size = dim[1].stride = 행 개수다.
        elevation_layer = self._pack_layer(elevation, n_rows, n_cols)
        # 'elevation_variance' packed with the exact same axis-flip +
        # column-major convention as 'elevation' above -- module E's
        # extract_elevation_variance (grid_math.py) relies on this matching.
        variance_layer = self._pack_layer(variance, n_rows, n_cols)
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
        grid_map.layers = ['elevation', 'elevation_variance']
        # basic_layers only ever holds 'elevation': it is the layer grid_map
        # consumers use to decide whether a cell counts as "observed" at
        # all -- variance is auxiliary uncertainty info about an already-
        # observed cell, not a second observed/unobserved criterion.
        grid_map.basic_layers = ['elevation']
        grid_map.data = [elevation_layer, variance_layer]
        grid_map.outer_start_index = 0
        grid_map.inner_start_index = 0
        return grid_map

    @staticmethod
    def _pack_layer(matrix, n_rows, n_cols):
        """Pack one layer's (n_rows, n_cols) float32 array into the wire format.

        See the comment block above (grid_map's axis-flip + column-major
        convention) -- shared here so 'elevation' and 'elevation_variance'
        can't drift into different packings by accident.
        """
        gm_matrix = matrix[::-1, ::-1]
        layer = Float32MultiArray()
        layer.layout.dim = [
            MultiArrayDimension(label='column_index', size=n_cols, stride=n_rows * n_cols),
            MultiArrayDimension(label='row_index', size=n_rows, stride=n_rows),
        ]
        layer.data = gm_matrix.flatten(order='F').tolist()
        return layer


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
