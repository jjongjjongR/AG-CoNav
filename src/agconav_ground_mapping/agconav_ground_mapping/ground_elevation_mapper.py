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
Pulling in the grid_map library just to wrap a single Eigen matrix would add
a large native dependency for no functional gain, and numpy already gives
fast vectorized binning (`np.bincount`) and resizing (`np.pad`). We do still
have to emit a spec-correct `grid_map_msgs/msg/GridMap` message by hand --
see `_build_grid_map_message` for the wire-format details.

design.md 4-2 (지도 저장 책임) now lives in ground_elevation_map_saver, not
here -- this node only ever does accumulation/publish.

칼만필터 적용 (agconav_drone/drone_elevation_mapper.py, Module A와 동일한 설계):
self._sum/self._count 러닝 애버리지를 셀별 독립 스칼라 칼만필터로 교체했다.
측정 노이즈(R)는 거리(구간표 조회) + 입사각(np.gradient로 이전 elevation에서
근사) + 점밀도를 결합해 계산하며(`_measurement_noise` 참고), 프로세스 노이즈
Q는 정적 지형 가정 하에 0으로 고정, 이미 값이 있는 셀에는 이노베이션 게이팅을
적용한다(`_kalman_update_cells` 참고). wheel(A300 lidar3d_0)과 leg(Go2
OS1-32) 모두 데이터시트상 동일한 구간별 정밀도(1시그마) 스펙을 갖는 것으로
확인되어 계수 기반 근사식 대신 실측 구간표를 그대로 쓴다 -- wheel/leg가 서로
다른 LiDAR 기종인 건 변함없으므로 로봇별 yaml에서 각각 독립적으로 채운다
(wheel이 실제로는 A300 lidar3d_0라는 다른 센서를 쓴다는 건 이미 알려진
미해결 이슈지만, 이번 작업 범위에서 새로 측정하지 않고 기존과 동일한 값을
유지한다).

이상치 방어 2겹: (1) `_accumulate`에서 센서 원점 기준 거리가
`max_sensor_range`를 넘는 점을 격자에 넣기 전에 버린다(np.isfinite로는 못
거르는, 유한하지만 물리적으로 말이 안 되게 먼 점 대비). (2) `_grow_to_fit`
에서 패딩 실행 직전에 예상 총 셀 개수를 계산해 `max_grid_cells`를 넘으면
실제 배열 확장을 하지 않고 error 로그만 남긴 뒤 그 배치를 버린다(기존 누적은
보존, 노드는 계속 살아있음) -- agconav_map_fusion/grid_math.py와 같은
기본값(30,000,000)으로 프로젝트 전체에서 일관되게 유지한다.

건물경계 혼합모델(1-8까지의 기본 칼만필터 위에 얹는 추가 레이어, Module A와
동일한 설계): 셀별로 두 개의 독립 칼만필터 가설을 병렬로 추적한다 -- 가설 A
("지형", self._elevation_a/_variance_a)와 가설 B("지붕/장애물 경계" 후보,
self._elevation_b/_variance_b). 표면 법선(np.gradient) 추정은 계속 가설
A(지형)만 참조한다.
- 신규 셀(가설 A조차 없음): 가설 A를 이번 배치로 초기화(게이팅 없음). 가설
  B는 아직 없음(NaN) -- 기존과 동일한 콜드스타트.
- 가설 A만 있는 셀: 새 관측의 이노베이션이 게이트(9.0)를 통과하면 A를 그대로
  갱신(표준 칼만 갱신). 통과 못하면(기본 칼만필터라면 버렸을 관측) 그
  관측으로 가설 B를 새로 만든다 -- 관측을 버리지 않고 "두 번째 후보"로
  보존한다.
- A/B가 둘 다 있는 셀: 새 관측을 정규화 이노베이션(innovation**2/(P+R))이 더
  작은(더 가까운) 가설에만 붙여 그 가설만 갱신한다. 둘 중 더 가까운 쪽조차
  게이트를 통과 못하면 3번째 후보를 만들지 않고 이번 배치를 그냥 버려
  복잡도를 2가설로 제한한다.
- 최종 발행: 셀마다 분산(P)이 더 작은(더 신뢰도 높은) 가설을 채택해
  elevation/elevation_variance로 내보낸다. 동률이면 A(지형)를 우선한다.
`_grow_to_fit`도 네 배열(A/B의 elevation/variance)을 항상 같은 shape/origin
으로 함께 NaN 패딩하도록 확장했다.

사전 검증: Module A(드론)의 5m AGL 실비행 bag에 대해 이 정확한 설계(2가설
병렬 추적)를 "변형 11"로 실험한 결과가 있다 -- wheel FN% -6.30pp(상대 59.0%
개선), leg FN% -3.61pp(상대 45.8% 개선), 커버리지도 +0.09pp 개선,
최대연결덩어리%도 wheel 27.58%->51.63%, leg 40.96%->54.51%로 크게 개선되어
실험한 11개 변형 중 전체 최고 성적으로 채택되었다(자세한 배경은
agconav_drone/drone_elevation_mapper.py의 module docstring 9번 항목 참고).
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
        # Tracking and Navigation"의 고전적 추적이론에서 흔히 쓰이는 값). 건물경계
        # 혼합모델(module docstring 참고)의 가설 A/B 게이팅에도 동일하게 쓰인다.
        self.declare_parameter('innovation_gate_threshold', 9.0)
        # 1겹 방어: 유한하지만 물리적으로 말이 안 되게 먼 점(np.isfinite로는
        # 못 거름)을 센서 원점 기준 거리로 걸러낸다.
        self.declare_parameter('max_sensor_range', 200.0)
        # 2겹 방어: 이상치 하나가 격자를 무한히 키우려 드는 것을 막는 최후
        # 방어선. agconav_map_fusion/grid_math.py가 쓰는 동명 파라미터와 같은
        # 기본값(30,000,000)으로 프로젝트 전체에서 일관되게 유지한다.
        self.declare_parameter('max_grid_cells', 30_000_000)

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
        self._max_sensor_range = float(self.get_parameter('max_sensor_range').value)
        self._max_grid_cells = int(self.get_parameter('max_grid_cells').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Grid state, in OUR OWN convention (not grid_map's wire convention,
        # see _build_grid_map_message): (row, col) = (0, 0) is the min-x/
        # min-y corner of the observed area; row grows with +x, col grows
        # with +y. None until the first point arrives -- shape and origin
        # both grow on demand (README 3.3: dynamic extent, not fixed size).
        # 건물경계 혼합모델(module docstring 참고): 가설 A("지형")가
        # self._elevation_a/_variance_a, 가설 B("지붕/장애물 경계" 후보)가
        # self._elevation_b/_variance_b -- 각각 독립된 셀별 스칼라 칼만필터
        # 상태다. 미관측 셀(A)과 "아직 만들어지지 않은 가설"(B)은 항상 NaN이다
        # -- 0으로 두면 "관측됐고 높이가 0m"으로 오인되고, (특히 variance
        # 배열이) 0으로 패딩되면 "완벽하게 확신한다"는 뜻이 되어 그 셀의 칼만
        # 게인 K = P/(P+R)가 영구히 0으로 고정되는 버그가 된다 (_grow_to_fit
        # 참고).
        self._elevation_a = None
        self._variance_a = None
        self._elevation_b = None
        self._variance_b = None
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
            f'"{navigation_status_topic}". max_sensor_range={self._max_sensor_range}m, '
            f'max_grid_cells={self._max_grid_cells}')

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

        self._accumulate(msg, transform.transform)

    def _accumulate(self, msg, transform):
        # 전체 PointCloud2를 do_transform_cloud로 재조립하지 않는다. 그 함수는
        # create_cloud() 호출 시 point_step을 넘기지 않아, 필드 뒤에 trailing
        # padding이 있는 클라우드에서 AssertionError로 죽는다. 실측:
        # /X/points(gz gpu_lidar)는 x/y/z/intensity/ring에 point_step=32,
        # 필드 총합 26바이트 - 6바이트 패딩. 모듈 A(drone_elevation_mapper)가
        # 같은 이유로 이미 이 방식을 쓴다. 우리는 elevation 계산에 x,y,z만
        # 필요하므로 좌표만 직접 변환한다.
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

        # 1겹 방어: np.isfinite로는 못 거르는, 유한하지만 물리적으로 말이 안
        # 되게 먼 점을 센서 원점(TF의 translation) 기준 거리로 걸러낸다.
        # transform.translation은 source_frame(센서) 원점의 target_frame(map)
        # 좌표이므로, 이미 map으로 변환된 points와 바로 거리 비교가 된다.
        sensor_origin = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z])
        in_range = np.linalg.norm(points - sensor_origin, axis=1) <= self._max_sensor_range
        n_dropped = points.shape[0] - int(in_range.sum())
        if n_dropped:
            self.get_logger().debug(
                f'dropping {n_dropped} point(s) beyond max_sensor_range='
                f'{self._max_sensor_range}m')
        points = points[in_range]
        if points.shape[0] == 0:
            return

        xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
        row_idx = np.floor((xs - self._origin_x) / self._resolution).astype(np.int64)
        col_idx = np.floor((ys - self._origin_y) / self._resolution).astype(np.int64)
        row_idx, col_idx = self._grow_to_fit(row_idx, col_idx)
        if row_idx is None:
            # 2겹 방어: 이 배치가 격자 크기 상한을 넘어 _grow_to_fit이 이미
            # 버렸다 (에러 로그도 거기서 남겼다). 기존 누적(A/B 둘 다)은 그대로
            # 보존된 상태이니 여기서도 조용히 이번 배치만 버린다.
            return

        # design.md 3/4-1: bin points into per-cell sums via bincount, one
        # batch (this callback's scan) at a time -- the Kalman update itself
        # happens once per cell below, not once per point.
        n_rows, n_cols = self._elevation_a.shape
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

        sensor_origin_xyz = (
            transform.translation.x, transform.translation.y, transform.translation.z)
        r_eff = self._measurement_noise(
            touched_rows, touched_cols, batch_mean_x, batch_mean_y, batch_mean_z,
            counts, sensor_origin_xyz)
        self._kalman_update_cells(touched_rows, touched_cols, batch_mean_z, r_eff)

        self._last_stamp = msg.header.stamp

    def _measurement_noise(
            self, rows, cols, mean_x, mean_y, mean_z, counts, sensor_origin):
        """Return R_eff for each of the given touched cells.

        R_point combines distance, incidence angle, and point density; R_eff
        divides it by how many of this batch's points landed in that cell.
        표면 법선은 가설 A("지형", module docstring 참고)에서만 추정한다.
        """
        sx, sy, sz = sensor_origin
        distance = np.sqrt((mean_x - sx) ** 2 + (mean_y - sy) ** 2 + (mean_z - sz) ** 2)

        # 각 셀의 표면 법선을 *이전까지 누적된* self._elevation_a(가설 A,
        # "지형")에서 np.gradient로 근사한다 -- 점 단위 이웃 탐색이나 별도
        # 포인트클라우드 라이브러리 없이, 이미 비닝된 높이 격자만 사용한다.
        # 이번 배치 자신의 칼만 갱신(아래) 전에 계산하므로 이전 스캔들의
        # 값만 반영한다.
        #
        # 성능: self._elevation_a 전체가 아니라, 이번 배치가 실제로 건드린 셀
        # 범위 + 중심차분에 필요한 가장자리 1칸만 잘라낸 국소 윈도우에만
        # np.gradient를 적용한다. np.gradient(edge_order=1 기본값)는 각 점의
        # 미분에 바로 이웃한 칸(내부는 i-1/i+1 중심차분, 배열 경계는 i/i±1
        # 편측차분)만 쓰므로, 윈도우가 이 이웃을 전부 포함하는 한 결과는
        # 전체 배열로 계산한 것과 수학적으로 완전히 동일하다 -- wheel/leg
        # 지도 규모(최대 max_grid_cells셀)에서 콜백마다 전체 배열을 미분하면
        # 지도가 커질수록 콜백 처리 시간이 계속 늘어나는 문제가 있었다.
        win_row_start = max(0, int(rows.min()) - 1)
        win_row_end = min(self._elevation_a.shape[0], int(rows.max()) + 2)
        win_col_start = max(0, int(cols.min()) - 1)
        win_col_end = min(self._elevation_a.shape[1], int(cols.max()) + 2)
        local_elevation = self._elevation_a[win_row_start:win_row_end, win_col_start:win_col_end]

        if local_elevation.shape[0] < 2 or local_elevation.shape[1] < 2:
            # np.gradient raises (not NaN) when an axis is too short to
            # differentiate along -- a real possibility early on, e.g. the
            # very first scan's points all landing in a single row/column
            # of cells. Treat it the same as a gradient full of NaN: no
            # neighbor info yet, fall back to cos_theta=1.0 below. "전체
            # 배열이 작을 때"가 아니라 "이 국소 윈도우가 작을 때" 기준 --
            # 윈도우는 항상 전체 배열 안에 들어가므로, 전체 배열이 이 조건을
            # 만족하지 않는 한(즉 전체가 이미 2보다 작은 한) 윈도우도 항상
            # 2 이상이라 두 기준은 동치다.
            dzdx = np.full_like(local_elevation, np.nan)
            dzdy = np.full_like(local_elevation, np.nan)
        else:
            with np.errstate(invalid='ignore'):
                dzdx, dzdy = np.gradient(local_elevation, self._resolution)
        normal_norm = np.sqrt(dzdx ** 2 + dzdy ** 2 + 1.0)
        normal_x = -dzdx / normal_norm
        normal_y = -dzdy / normal_norm
        normal_z = 1.0 / normal_norm

        ray_x, ray_y, ray_z = mean_x - sx, mean_y - sy, mean_z - sz
        ray_norm = np.sqrt(ray_x ** 2 + ray_y ** 2 + ray_z ** 2)
        # rows/cols는 self._elevation_a 전체 기준 인덱스인데, normal_x/y/z는
        # 윈도우로 잘라낸 국소 배열이라 좌표계가 다르다 -- 윈도우 시작
        # 오프셋(win_row_start/win_col_start)만큼 빼서 국소 좌표로 변환한다.
        local_rows = rows - win_row_start
        local_cols = cols - win_col_start
        with np.errstate(invalid='ignore'):
            cos_theta = np.abs(
                (ray_x / ray_norm) * normal_x[local_rows, local_cols]
                + (ray_y / ray_norm) * normal_y[local_rows, local_cols]
                + (ray_z / ray_norm) * normal_z[local_rows, local_cols])
        # No neighbor info to derive a normal from -- either this cell has
        # never been observed before (self._elevation_a is NaN there) or its
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
        """Fuse this batch's per-cell mean height into the A/B hypothesis grids.

        건물경계 혼합모델(module docstring 참고) -- 셀별로 최대 2개의 독립
        스칼라 칼만필터 가설(A="지형", B="지붕/장애물 경계" 후보)을 병렬로
        추적한다. Q=0(정적 지형 가정), 관측모델이 이미 선형(z = x + noise)
        이라 표준 (선형) 칼만필터로 충분하고 EKF/UKF 근사가 필요 없다.
        """
        # p_prev(=pa_prev)만 신규/기존 판정(is_new)에 쓰인다 -- elevation
        # 값(xa)이나 가설 B 관련 배열은 그 판정에 쓰이지 않으므로, 이번
        # 배치 전체(rows/cols)가 아니라 실제로 필요한 부분집합에서만 직접
        # 읽는다(이전에 발견된 낭비 제거 -- 예를 들어 신규 셀 위치에서
        # self._elevation_a를 읽어봐야 그 결과는 버려진다). 결과값은
        # 수학적으로 기존 방식과 완전히 동일하다.
        pa_prev = self._variance_a[rows, cols]

        # 가설 A의 P가 아직 NaN인 셀은 한 번도 관측된 적이 없다 -- 이번
        # 배치로 가설 A를 바로 초기화하고(게이팅 없음), 가설 B는 아직
        # 만들지 않는다(기존과 동일한 콜드스타트).
        is_new = np.isnan(pa_prev)
        new_rows, new_cols = rows[is_new], cols[is_new]
        self._elevation_a[new_rows, new_cols] = batch_mean[is_new]
        self._variance_a[new_rows, new_cols] = r_eff[is_new]

        # 가설 A가 있는(기존) 셀 위치에서만 xa/가설 B의 P를 읽는다 -- is_new
        # 위치는 제외.
        has_a = ~is_new
        ha_rows, ha_cols = rows[has_a], cols[has_a]
        xa_ha = self._elevation_a[ha_rows, ha_cols]
        pa_ha = pa_prev[has_a]
        r_ha = r_eff[has_a]
        z_ha = batch_mean[has_a]
        pb_ha = self._variance_b[ha_rows, ha_cols]
        has_b = ~np.isnan(pb_ha)
        a_only = ~has_b

        # --- 가설 A만 있는 셀: 게이트 통과 시 A 갱신, 실패 시 B를 새로 생성 ---
        ao_rows, ao_cols = ha_rows[a_only], ha_cols[a_only]
        xa, pa, r, z = xa_ha[a_only], pa_ha[a_only], r_ha[a_only], z_ha[a_only]
        innovation_a = z - xa
        cov_a = pa + r
        normalized_a = innovation_a ** 2 / cov_a
        passed = normalized_a <= self._innovation_gate_threshold

        upd_rows, upd_cols = ao_rows[passed], ao_cols[passed]
        gain = pa[passed] / cov_a[passed]
        self._elevation_a[upd_rows, upd_cols] = xa[passed] + gain * innovation_a[passed]
        self._variance_a[upd_rows, upd_cols] = (1.0 - gain) * pa[passed]

        # 게이트를 통과하지 못한 관측은 버리지 않고 가설 B로 보존한다 --
        # 기본 칼만필터와의 핵심 차이.
        spawn_rows, spawn_cols = ao_rows[~passed], ao_cols[~passed]
        self._elevation_b[spawn_rows, spawn_cols] = z[~passed]
        self._variance_b[spawn_rows, spawn_cols] = r[~passed]

        # --- A/B가 둘 다 있는 셀: 더 가까운(정규화 이노베이션이 더 작은)
        # 가설에만 붙여 갱신. 둘 다 게이트를 통과 못하면(3번째 후보를
        # 만들지 않고) 버린다 -- "더 가까운" 쪽의 정규화 이노베이션이 항상
        # 더 작거나 같으므로, "더 가까운 쪽이 게이트를 통과하는지"만 보면
        # "둘 중 하나라도 통과하는지"와 동치다.
        bh_rows, bh_cols = ha_rows[has_b], ha_cols[has_b]
        xa2, pa2 = xa_ha[has_b], pa_ha[has_b]
        xb2, pb2 = self._elevation_b[bh_rows, bh_cols], pb_ha[has_b]
        r2, z2 = r_ha[has_b], z_ha[has_b]

        innovation_a2 = z2 - xa2
        cov_a2 = pa2 + r2
        normalized_a2 = innovation_a2 ** 2 / cov_a2

        innovation_b2 = z2 - xb2
        cov_b2 = pb2 + r2
        normalized_b2 = innovation_b2 ** 2 / cov_b2

        # 동률이면 가설 A(지형)를 우선한다.
        closer_is_a = normalized_a2 <= normalized_b2
        min_normalized = np.where(closer_is_a, normalized_a2, normalized_b2)
        passed2 = min_normalized <= self._innovation_gate_threshold

        upd_a2 = passed2 & closer_is_a
        a2_rows, a2_cols = bh_rows[upd_a2], bh_cols[upd_a2]
        gain_a2 = pa2[upd_a2] / cov_a2[upd_a2]
        self._elevation_a[a2_rows, a2_cols] = xa2[upd_a2] + gain_a2 * innovation_a2[upd_a2]
        self._variance_a[a2_rows, a2_cols] = (1.0 - gain_a2) * pa2[upd_a2]

        upd_b2 = passed2 & ~closer_is_a
        b2_rows, b2_cols = bh_rows[upd_b2], bh_cols[upd_b2]
        gain_b2 = pb2[upd_b2] / cov_b2[upd_b2]
        self._elevation_b[b2_rows, b2_cols] = xb2[upd_b2] + gain_b2 * innovation_b2[upd_b2]
        self._variance_b[b2_rows, b2_cols] = (1.0 - gain_b2) * pb2[upd_b2]
        # 둘 다 게이트를 통과하지 못한 셀은 A, B 둘 다 이전 값을 그대로
        # 유지하고, 이번 배치의 관측값은 버려진다.

    def _grow_to_fit(self, row_idx, col_idx):
        """Pad the grid so row_idx/col_idx fit, remapped into the new array.

        README 3.3: the map only grows to cover what has actually been
        observed, it is never pre-sized. 건물경계 혼합모델(module docstring
        참고): 가설 A/B의 네 배열(elevation/variance x2)을 항상 같은
        shape/origin으로 함께 패딩한다.

        2겹 방어(1겹은 _accumulate의 거리 필터): 패딩/생성 직전에 예상 총
        셀 개수가 max_grid_cells를 넘으면 실제 배열 확장/생성을 하지 않고
        (None, None)을 반환한다 -- 기존 누적은 손대지 않은 채 그대로
        보존되고, 이번 배치만 호출자가 버린다.
        """
        if self._elevation_a is None:
            min_row, max_row = int(row_idx.min()), int(row_idx.max())
            min_col, max_col = int(col_idx.min()), int(col_idx.max())
            n_rows = max_row - min_row + 1
            n_cols = max_col - min_col + 1
            total_cells = n_rows * n_cols
            if total_cells > self._max_grid_cells:
                self.get_logger().error(
                    f'grid would grow to {total_cells} cells, exceeding '
                    f'max_grid_cells={self._max_grid_cells}, dropping this batch')
                return None, None
            shape = (n_rows, n_cols)
            # 0이 아니라 NaN -- __init__의 self._elevation_a/_variance_a 등
            # 주석 참고. variance 배열이 0으로 채워지면 그 셀의 칼만 게인이
            # 영구히 0으로 고정되는 치명적인 버그가 된다.
            self._elevation_a = np.full(shape, np.nan)
            self._variance_a = np.full(shape, np.nan)
            self._elevation_b = np.full(shape, np.nan)
            self._variance_b = np.full(shape, np.nan)
            self._origin_x += min_row * self._resolution
            self._origin_y += min_col * self._resolution
            return row_idx - min_row, col_idx - min_col

        n_rows, n_cols = self._elevation_a.shape
        pad_before_row = max(0, -int(row_idx.min()))
        pad_after_row = max(0, int(row_idx.max()) - (n_rows - 1))
        pad_before_col = max(0, -int(col_idx.min()))
        pad_after_col = max(0, int(col_idx.max()) - (n_cols - 1))

        if pad_before_row or pad_after_row or pad_before_col or pad_after_col:
            new_n_rows = n_rows + pad_before_row + pad_after_row
            new_n_cols = n_cols + pad_before_col + pad_after_col
            total_cells = new_n_rows * new_n_cols
            if total_cells > self._max_grid_cells:
                self.get_logger().error(
                    f'grid would grow to {total_cells} cells, exceeding '
                    f'max_grid_cells={self._max_grid_cells}, dropping this batch')
                return None, None
            pad_width = ((pad_before_row, pad_after_row), (pad_before_col, pad_after_col))
            # constant_values=np.nan (not the np.pad default of 0) -- same
            # reasoning as the fresh-array case above. 가설 A/B 네 배열 모두
            # 동일하게 패딩한다.
            self._elevation_a = np.pad(self._elevation_a, pad_width, constant_values=np.nan)
            self._variance_a = np.pad(self._variance_a, pad_width, constant_values=np.nan)
            self._elevation_b = np.pad(self._elevation_b, pad_width, constant_values=np.nan)
            self._variance_b = np.pad(self._variance_b, pad_width, constant_values=np.nan)
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
        if self._elevation_a is None:
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

    def _select_hypothesis(self):
        """Pick, per cell, whichever of hypothesis A/B has lower variance.

        건물경계 혼합모델(module docstring 참고)의 최종 발행 규칙: 분산(P)이
        더 작은(더 신뢰도 높은) 가설을 채택한다. B가 아직 없는(NaN) 셀은
        당연히 A가 채택되고, 동률이면 A(지형)를 우선한다.
        """
        b_valid = ~np.isnan(self._variance_b)
        with np.errstate(invalid='ignore'):
            a_wins = ~b_valid | (self._variance_a <= self._variance_b)
        elevation = np.where(a_wins, self._elevation_a, self._elevation_b)
        variance = np.where(a_wins, self._variance_a, self._variance_b)
        return elevation, variance

    def _build_grid_map_message(self):
        n_rows, n_cols = self._elevation_a.shape
        # README 3.3: unobserved cells stay NaN -- 가설 선택(_select_hypothesis)
        # 결과가 이미 미관측 셀에 NaN을 갖고 있으므로, 예전 running-average
        # 버전처럼 별도로 count로 나눌 필요가 없다.
        elevation, variance = self._select_hypothesis()
        elevation = elevation.astype(np.float32)
        variance = variance.astype(np.float32)

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
        # elevation_variance도 elevation과 완전히 동일한 규약(_pack_layer)을
        # 재사용한다. dim 크기는 std_msgs/MultiArrayLayout 규약을 따른다:
        # 차원은 바깥->안 순서이고, 최내곽 차원은 stride == size 여야 한다.
        # Eigen 열 우선 저장 기준으로 바깥 차원이 열(column_index), 안쪽
        # 차원이 행(row_index)이므로 dim[0].size = 열 개수, dim[1].size =
        # dim[1].stride = 행 개수다.
        elevation_layer = self._pack_layer(elevation, n_rows, n_cols)
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
