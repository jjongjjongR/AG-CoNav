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
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points


class GroundElevationMapper(Node):
    """Transforms a robot's raw PointCloud2 into map and bins it into a grid."""

    def __init__(self):
        super().__init__('ground_elevation_mapper')

        # CONTRIBUTING 5: relative topic names, resolved by the launch
        # namespace (e.g. /wheel/points -> /wheel/elevation_map).
        self.declare_parameter('points_topic', 'points')
        self.declare_parameter('elevation_map_topic', 'elevation_map')
        self.declare_parameter('navigation_status_topic', 'navigation_status')
        self.declare_parameter('mapping_active_topic', 'mapping_active')
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
        # 점군 측정 시각의 TF가 아직 안 왔을 때 이만큼 기다린다. 로봇마다 TF가
        # 도착하는 지연이 다르다 - 실측(테스트 월드 전체 스택): wheel은 중앙값
        # 0.005초로 사실상 정시라 드롭 13건뿐이었지만, leg는 중앙값 0.133초
        # 최대 0.256초 뒤처져서 점군 1354개를 통째로 버렸다. 0.3초면 leg 최대
        # 지연까지 덮는다. 0으로 두면 기다리지 않고 바로 버린다(원래 동작).
        self.declare_parameter('tf_timeout_sec', 0.3)
        # 전체 스택 기동처럼 CPU가 잠시 포화되면 점군이 TF보다 먼저 전달될 수
        # 있다. 정확한 측정 시각 TF를 우선하되, 그것만 미래 외삽으로 실패하면
        # 제한된 나이의 최신 TF를 사용한다. 상한을 넘은 좌표는 지도 왜곡을
        # 막기 위해 여전히 폐기한다.
        self.declare_parameter('max_latest_tf_age_sec', 5.0)
        # !! OOM 방지 !! _grow_to_fit 은 관측된 점을 전부 담도록 격자를 무제한
        # 으로 키운다. 셀당 float64 2개(_sum/_count)라 16바이트씩 붙는다.
        # 실제 사고: 전체 맵 종단 실행 26분째에 이 노드가 RSS 29 GB 까지 커져
        # OOM 킬러에 죽었고(커널 로그 "Killed process ... (ground_elevatio)
        # anon-rss:29035964kB") 시뮬레이션 전체가 함께 날아갔다. 29 GB 는 18억
        # 셀 = 0.1 m 격자로 4.3 km 사방이다. 정렬 월드는 578 x 482 m
        # (2790만 셀, 446 MB)이므로 65배 넘게 벗어난 값이다.
        #
        # 두 겹으로 막는다.
        #  1) max_point_range_m: 센서 원점에서 이만큼 넘게 떨어진 점을 버린다.
        #     라이다 최대 사거리는 Go2 4D 30 m / velodyne 131 m / OS1 170 m 라
        #     200 m 를 넘는 반사는 물리적으로 나올 수 없다. 즉 수치 이상이다.
        #  2) max_cells: 그래도 격자가 이 한도를 넘기려 하면 그 점군을 통째로
        #     버린다. 센서 자체가 먼 좌표로 튀면 (1)로는 못 막기 때문이다.
        #     6000만 셀 = 약 960 MB, 정렬 월드의 2.2배 여유다.
        # 0 으로 두면 각각 끈다.
        self.declare_parameter('max_point_range_m', 200.0)
        self.declare_parameter('max_cells', 60_000_000)

        points_topic = self.get_parameter('points_topic').value
        elevation_map_topic = self.get_parameter('elevation_map_topic').value
        navigation_status_topic = self.get_parameter('navigation_status_topic').value
        mapping_active_topic = self.get_parameter('mapping_active_topic').value
        self._target_frame = self.get_parameter('target_frame').value
        self._target_source_frame = self.get_parameter('target_source_frame').value
        self._resolution = float(self.get_parameter('resolution').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._data_timeout = Duration(
            seconds=self.get_parameter('data_timeout_sec').value)

        self._tf_buffer = Buffer()
        # spin_thread=True — TF 수신을 이 노드의 실행기가 아니라 리스너 전용
        # 스레드에서 돌린다. 이게 없으면 _points_callback 안에서 transform을
        # 기다리는 순간 TF 메시지를 받을 주체가 사라져 영원히 안 오는 것을
        # 기다리게 되고(자기 교착), 그래서 원래 코드는 아예 대기를 못 했다.
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_timeout_sec').value))
        self._max_latest_tf_age = float(
            self.get_parameter('max_latest_tf_age_sec').value)
        self._max_point_range = float(self.get_parameter('max_point_range_m').value)
        self._max_cells = int(self.get_parameter('max_cells').value)

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
        self._mapping_active = False

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

        # Keep the raw-cloud subscription physically absent while the robot is
        # waiting.  Together with ros_gz_bridge's lazy mode this prevents the
        # bridge from waking the Gazebo LiDAR merely because Module D exists.
        self._points_topic = points_topic
        self._points_qos = points_qos
        self._points_sub = None
        self._mapping_active_sub = self.create_subscription(
            Bool, mapping_active_topic, self._mapping_active_callback, latched_qos)
        self._navigation_status_sub = self.create_subscription(
            Bool, navigation_status_topic, self._navigation_status_callback, latched_qos)
        self._elevation_map_pub = self.create_publisher(
            GridMap, elevation_map_topic, latched_qos)

        check_period = self.get_parameter('check_period_sec').value
        self._check_timer = self.create_timer(
            check_period, self._check_data_received)

        self.get_logger().info(
            f'Waiting for "{mapping_active_topic}"; then accumulating '
            f'"{points_topic}" -> "{elevation_map_topic}" '
            f'(resolution={self._resolution} m/cell, target_frame='
            f'"{self._target_frame}"), publishing once on '
            f'"{navigation_status_topic}"')

    def _mapping_active_callback(self, msg):
        active = bool(msg.data) and not self._published
        if active == self._mapping_active:
            return
        self._mapping_active = active
        if active:
            self._points_sub = self.create_subscription(
                PointCloud2, self._points_topic,
                self._points_callback, self._points_qos)
            self.get_logger().info(
                'mapping_active=True: 주행 점군 누적을 시작합니다.')
        else:
            self._stop_point_collection()
            self.get_logger().info(
                'mapping_active=False: 주행 점군 누적을 중지합니다.')

    def _stop_point_collection(self):
        if self._points_sub is not None:
            self.destroy_subscription(self._points_sub)
            self._points_sub = None

    def _points_callback(self, msg):
        if not self._mapping_active or self._published:
            return
        self._last_received = self.get_clock().now()

        source_frame = self._target_source_frame or msg.header.frame_id
        try:
            # design.md 7-2: look up the TF at the cloud's own measurement
            # stamp. 아직 안 왔으면 tf_timeout_sec 만큼만 기다린다 - TF 리스너가
            # 전용 스레드에서 도니(위 spin_thread=True) 여기서 기다려도 TF는
            # 계속 들어온다. 그 안에 안 오면 그때 버린다.
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, source_frame, Time.from_msg(msg.header.stamp),
                timeout=self._tf_timeout)
        except TransformException as exact_ex:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame, source_frame, Time(),
                    timeout=self._tf_timeout)
            except TransformException as latest_ex:
                self.get_logger().warn(
                    f'TF lookup failed for "{source_frame}" -> '
                    f'"{self._target_frame}", dropping cloud: exact={exact_ex}; '
                    f'latest={latest_ex}', throttle_duration_sec=5.0)
                return

            cloud_time = Time.from_msg(msg.header.stamp)
            tf_time = Time.from_msg(transform.header.stamp)
            age_sec = (cloud_time - tf_time).nanoseconds / 1e9
            if age_sec < 0.0 or age_sec > self._max_latest_tf_age:
                self.get_logger().warn(
                    'Exact-time TF unavailable and latest TF is %.3fs old '
                    '(limit %.3fs), dropping cloud: %s'
                    % (age_sec, self._max_latest_tf_age, exact_ex),
                    throttle_duration_sec=5.0)
                return
            self.get_logger().warn(
                'Exact-time TF unavailable; using latest TF (%.3fs old).'
                % age_sec, throttle_duration_sec=5.0)

        self._accumulate(msg, transform.transform)

    def _accumulate(self, msg, transform):
        # 전체 PointCloud2를 do_transform_cloud로 재조립하지 않는다.
        # 그 함수는 create_cloud() 호출 시 point_step을 넘기지 않아, 필드 뒤에
        # trailing padding이 있는 클라우드에서 AssertionError로 죽는다.
        # 실측: /X/points(gz gpu_lidar)는 x/y/z/intensity/ring에 point_step=32,
        # 필드 총합 26바이트 - 6바이트 패딩. 모듈 A(drone_elevation_mapper)가
        # 같은 이유로 이미 이 방식을 쓴다.
        # 우리는 elevation 계산에 x,y,z만 필요하므로 좌표만 직접 변환한다.
        points = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.shape[0] == 0:
            return

        # skip_nans는 NaN만 거른다. gz gpu_lidar는 최대 사거리 밖 점을 NaN이
        # 아니라 Inf로 채우므로(실측 32768개 중 76%) 따로 걸러야 한다.
        # 안 걸러내면 _grow_to_fit이 배열을 무한히 키우려다 죽는다.
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] == 0:
            return

        points = transform_points(points, transform)

        # 센서에서 너무 먼 점을 버린다 (위 max_point_range_m 주석 참고).
        # 변환 뒤에 재는 이유는 센서 좌표계 거리가 아니라 격자를 키우는 원인인
        # map 좌표계 위치가 문제이기 때문이다. 둘은 강체변환이라 거리는 같지만,
        # 변환 자체가 깨진 경우(회전에 NaN/거대값)는 변환 후에만 잡힌다.
        if self._max_point_range > 0.0:
            t = transform.translation
            d2 = ((points[:, 0] - t.x) ** 2 + (points[:, 1] - t.y) ** 2
                  + (points[:, 2] - t.z) ** 2)
            keep = np.isfinite(d2) & (d2 <= self._max_point_range ** 2)
            if not keep.all():
                self.get_logger().warn(
                    '센서에서 %.0f m 를 넘는 점 %d/%d개를 버렸다 (최대 %.1f m). '
                    '라이다 사거리로는 나올 수 없는 값이라 수치 이상으로 본다.'
                    % (self._max_point_range, int((~keep).sum()), keep.size,
                       float(np.sqrt(np.nanmax(d2))) if np.isfinite(d2).any() else float('inf')),
                    throttle_duration_sec=5.0)
                points = points[keep]
                if points.shape[0] == 0:
                    return

        xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
        row_idx = np.floor((xs - self._origin_x) / self._resolution).astype(np.int64)
        col_idx = np.floor((ys - self._origin_y) / self._resolution).astype(np.int64)
        grown = self._grow_to_fit(row_idx, col_idx)
        if grown is None:      # 셀 한도 초과 -- 이 점군은 통째로 버린다
            return
        row_idx, col_idx = grown

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

        단, max_cells 를 넘기게 되면 키우지 않고 None 을 돌려준다 -- 호출부는
        그 점군을 버린다. 무제한 확장이 실제로 OOM 을 냈다(생성자 주석 참고).
        """
        if self._sum is None:
            min_row, max_row = int(row_idx.min()), int(row_idx.max())
            min_col, max_col = int(col_idx.min()), int(col_idx.max())
            if not self._fits(max_row - min_row + 1, max_col - min_col + 1):
                return None
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
            if not self._fits(n_rows + pad_before_row + pad_after_row,
                              n_cols + pad_before_col + pad_after_col):
                return None
            pad_width = ((pad_before_row, pad_after_row), (pad_before_col, pad_after_col))
            self._sum = np.pad(self._sum, pad_width)
            self._count = np.pad(self._count, pad_width)
            self._origin_x -= pad_before_row * self._resolution
            self._origin_y -= pad_before_col * self._resolution
            row_idx = row_idx + pad_before_row
            col_idx = col_idx + pad_before_col

        return row_idx, col_idx

    def _fits(self, n_rows, n_cols):
        """격자를 (n_rows, n_cols) 로 키워도 되는지. 안 되면 경고하고 False."""
        if self._max_cells <= 0:
            return True
        if n_rows * n_cols <= self._max_cells:
            return True
        self.get_logger().error(
            '격자를 %d x %d = %.1f억 셀로 키우려 해서 이 점군을 버렸다 '
            '(한도 %.1f억, 약 %.1f GB). 센서 위치나 TF 가 튀었을 가능성이 크다.'
            % (n_rows, n_cols, n_rows * n_cols / 1e8, self._max_cells / 1e8,
               self._max_cells * 16 / 1e9),
            throttle_duration_sec=10.0)
        return False

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
        occupied_cells = int((self._count > 0).sum())
        self.get_logger().info(
            'navigation 완료: 주행 중 관측 셀 %d개, 격자 %d x %d를 발행합니다.'
            % (occupied_cells, self._sum.shape[0], self._sum.shape[1]))
        self._elevation_map_pub.publish(self._build_grid_map_message())
        self._published = True
        self._mapping_active = False
        self._stop_point_collection()

    def _check_data_received(self):
        if not self._mapping_active or self._published:
            return
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
        # 축 뒤집기와 column-major 평탄화는 실환경(Jazzy)에서 검증 완료.
        # dim 크기는 std_msgs/MultiArrayLayout 규약을 따른다: 차원은 바깥->안
        # 순서이고, 최내곽 차원은 stride == size 여야 한다. Eigen 열 우선 저장
        # 기준으로 바깥 차원이 열(column_index), 안쪽 차원이 행(row_index)이므로
        # dim[0].size = 열 개수, dim[1].size = dim[1].stride = 행 개수다.
        gm_matrix = elevation[::-1, ::-1]
        elevation_layer = Float32MultiArray()
        elevation_layer.layout.dim = [
            MultiArrayDimension(label='column_index', size=n_cols, stride=n_rows * n_cols),
            MultiArrayDimension(label='row_index', size=n_rows, stride=n_rows),
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
