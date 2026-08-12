"""Subscribes to the drone's raw LiDAR PointCloud2, transforms it into map, and
accumulates a 2.5D elevation grid, publishing the final map once the scan path
is complete.

Structure/누적 로직은 채현우님의 agconav_ground_mapping/ground_elevation_mapper.py를
그대로 재사용한다 (TF 조회 방식, numpy 기반 칼만필터 누적, 동적 그리드 확장,
grid_map 메시지 수동 패킹, QoS 구성 전부 동일). 드론이라서 실제로 달라지는 부분만
아래에 정리한다 - 그 외 로직은 스캔 방향/로봇 종류와 무관하게 map 프레임으로
변환된 x,y,z만 다루므로 그대로 통한다.

1. TF 소스: ground는 모듈 B(위치추정 스택)가 발행하는 TF를 조회하지만, 드론은
   drone_path_player가 kinematic 방식으로 map -> drone/base_link를 직접
   발행한다(모듈 B 안 거침, README 3.1). 그 TF를 그대로 조회하면 된다.

2. target_source_frame: ground(wheel)는 실측 결과 사설 프레임과 전역 프레임 이름이
   달라 target_source_frame(wheel/lidar3d_0_sensor_link)으로 강제 오버라이드해야
   했다. 드론은 agconav_description의 model.sdf에서 라이다 센서의
   `<gz_frame_id>drone/os1_lidar</gz_frame_id>`를 이미 전역 이름으로 지정해뒀고,
   실제 `/drone/points`를 echo해 header.frame_id가 정확히 "drone/os1_lidar"로
   찍히는 것도 확인했다 - 그래서 기본값을 비워두고 msg.header.frame_id를 그대로
   신뢰한다.

3. TF 체인: map -> drone/base_link(drone_path_player가 발행)까지는 있었지만
   drone/base_link -> drone/os1_lidar 구간이 비어 있었다(드론은 URDF/xacro +
   robot_state_publisher가 아니라 SDF로 직접 스폰되는 구조라, wheel/leg와 달리
   이 링크를 자동으로 발행해주는 노드가 없었음). model.sdf의 os1_lidar_mount/
   os1_lidar 링크 pose(둘 다 model 프레임 기준, relative_to 없음)를 근거로
   drone_sim_test.launch.py에 static_transform_publisher를 추가해 채웠다.

4. 토픽: 드론은 1대뿐이라 wheel/leg처럼 launch 네임스페이스로 나누지 않고
   /drone/points, /drone/elevation_map으로 고정.

5. 완료 트리거: ground의 navigation_status(Bool, latched) 대신 오늘 만든
   drone_path_player의 /drone/path_status(Bool, latched)를 구독한다 - 경로
   재생이 끝나는 시점(drone_path_player._finish())에 한 번만 True가 온다.

6. resolution(0.10 m/cell, README 3.3 고정값) 관련 우려: 드론은 84m 상공에서
   지면을 내려다보는 OS1-32 스캔이라, 지상 라이다처럼 근거리 조밀한 점군이
   아니다. path.yaml의 strip_spacing(32.581m)과 수직 채널 32개 기준으로
   추정하면 한 패스당 지면 점 간격이 0.10m 셀보다 훨씬 성긴 수 미터 단위일
   가능성이 높다 - 즉 셀당 관측 횟수가 매우 적어 count==0인 빈 셀(NaN)이
   지상 로봇 대비 훨씬 많이 남을 수 있다. lawnmower 패턴의 스트립 오버랩이
   여러 패스에 걸쳐 누적되며 이를 어느 정도 메워주긴 하겠지만, 실측 없이
   확신할 수 없어 일단 0.10m 그대로 두고 실측하면서 튜닝하기로 함(아래
   __init__의 info 로그로 이 우려를 남긴다).

7. 칼만필터 적용: ground_elevation_mapper(Module D)가 이미 적용한 것과 동일한
   설계로, self._sum/self._count 러닝 애버리지를 셀별 독립 스칼라 칼만필터
   (self._elevation/self._variance)로 교체했다. 측정 노이즈(R)는 거리(구간표
   조회) + 입사각(np.gradient로 이전 self._elevation에서 근사) + 점밀도를
   결합해 계산하며(`_measurement_noise` 참고), 프로세스 노이즈 Q는 정적 지형
   가정 하에 0으로 고정, 이미 값이 있는 셀에는 이노베이션 게이팅을 적용한다
   (`_kalman_update_cells` 참고). 거리 구간표(measurement_noise_max_distances/
   measurement_noise_sigmas)는 leg_elevation_mapper.yaml과 완전히 동일한 값을
   쓴다 - README가 명시하듯("센서는 OS1-32로 통일") 드론도 leg(Go2)와 똑같이
   실제 Ouster OS1-32를 장착하므로, wheel/leg에 적용한 데이터시트 실측 구간별
   정밀도(1시그마) 스펙이 드론에도 그대로 적용된다. 드론 전용으로 다시 추정한
   placeholder가 아니다.

   드론은 6번 항목의 84m 고도 특성 때문에, 지상 로봇보다 셀당 점 밀도가 훨씬
   낮고 이웃 셀이 비어있는(NaN) 경우가 잦을 것으로 예상된다 - 즉 콜드스타트
   초기화(첫 관측이라 게이팅 없이 바로 값을 넣는 경우) 비율과, 이웃 정보가
   없어 `_measurement_noise`가 cos_theta=1.0으로 폴백하는 빈도가 Module D
   (지상 로봇)보다 훨씬 높을 수 있다. 이는 고도 때문에 원래 그런 것이지
   버그가 아니므로, 이 로직을 드론 상황에 맞춰 임의로 다르게 바꾸지 않고
   Module D와 동일한 계산을 그대로 적용했다.

   `elevation_variance` 레이어를 elevation과 함께 새로 발행하지만, 이 브랜치의
   agconav_map_fusion 병합 로직은 아직 고정 우선순위(wheel > leg > 드론, 드론은
   최하위 fallback -- README 132행)를 쓰고 있어 이 값을 소비하지 않는다.
   agconav_map_fusion을 분산 비교 방식으로 바꾸는 것은 이번 작업 범위 밖이다.
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


class DroneElevationMapper(Node):
    """Transforms the drone's raw PointCloud2 into map and bins it into a grid."""

    def __init__(self):
        super().__init__('drone_elevation_mapper')

        # 드론은 1대뿐이라 wheel/leg처럼 launch 네임스페이스로 나누지 않고
        # 계약 토픽 이름을 기본값으로 그대로 고정한다.
        self.declare_parameter('points_topic', '/drone/points')
        self.declare_parameter('elevation_map_topic', '/drone/elevation_map')
        self.declare_parameter('path_status_topic', '/drone/path_status')
        # README 3.1: single global frame `map`.
        self.declare_parameter('target_frame', 'map')
        # ground_elevation_mapper와 동일한 안전장치(파라미터로 노출)지만, 드론은
        # model.sdf의 gz_frame_id가 이미 전역 이름("drone/os1_lidar")이라
        # 실측으로 확인된 만큼 기본값은 비워두고 msg.header.frame_id를 신뢰한다.
        self.declare_parameter('target_source_frame', '')
        # README 3.3: fixed 0.10 m/cell resolution across all maps.
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('data_timeout_sec', 2.0)
        self.declare_parameter('check_period_sec', 1.0)
        # 칼만필터 거리 기반 측정 노이즈(R) 파라미터 -- 드론은 leg(Go2)와 동일하게
        # 실제 Ouster OS1-32를 장착하므로(README: "센서는 OS1-32로 통일 -- 3대
        # 통일"), wheel/leg에 적용한 것과 완전히 동일한 데이터시트 구간별
        # 정밀도(1시그마) 스펙을 그대로 쓴다 (top-of-file 7번 항목 참고) --
        # 드론 전용으로 다시 추정한 placeholder가 아니다.
        self.declare_parameter(
            'measurement_noise_max_distances', [1.0, 20.0, 50.0, 100.0])
        self.declare_parameter(
            'measurement_noise_sigmas', [0.007, 0.010, 0.020, 0.050])
        # cos(80°) ≈ 0.17 -- grazing angle(입사각이 90°에 가까워질 때) 근처에서
        # R_point가 1/cos_theta**2로 발산하는 것을 막는 하한. Module D와 동일.
        self.declare_parameter('incidence_cos_floor', 0.17)
        # 카이제곱분포 자유도 1, 유의수준 약 0.27%(대략 3-시그마)에 해당하는
        # 표준 게이팅 임계값 (Bar-Shalom, "Estimation with Applications to
        # Tracking and Navigation"). Module D와 동일.
        self.declare_parameter('innovation_gate_threshold', 9.0)

        points_topic = self.get_parameter('points_topic').value
        elevation_map_topic = self.get_parameter('elevation_map_topic').value
        path_status_topic = self.get_parameter('path_status_topic').value
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
        # self._elevation (x)은 셀별 최선 추정 높이, self._variance (P)는 그
        # 불확실성 -- 셀별 독립 스칼라 칼만필터의 상태다 (top-of-file 7번 항목,
        # _kalman_update_cells 참고). 미관측 셀은 0이 아니라 항상 NaN이다 --
        # 0으로 두면 "관측됐고 높이가 0m"으로 오인되고, (특히 variance 배열이)
        # 0으로 패딩되면 "완벽하게 확신한다"는 뜻이 되어 그 셀의 칼만 게인
        # K = P/(P+R)가 영구히 0으로 고정되는 버그가 된다 (_grow_to_fit 참고).
        self._elevation = None
        self._variance = None
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._last_stamp = None
        self._last_received = None
        self._published = False

        # design.md 7-1과 동일: raw sensor cloud는 best effort / volatile /
        # keep last / depth 5.
        points_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # elevation_map과 path_status는 둘 다 한 번만 오는 신호(최종 지도 /
        # 완료)이므로 reliable + transient_local + keep_last + depth 1 -
        # best_effort였다면 그 한 번의 메시지가 유실됐을 때 복구할 방법이 없다.
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._points_sub = self.create_subscription(
            PointCloud2, points_topic, self._points_callback, points_qos)
        self._path_status_sub = self.create_subscription(
            Bool, path_status_topic, self._path_status_callback, latched_qos)
        self._elevation_map_pub = self.create_publisher(
            GridMap, elevation_map_topic, latched_qos)

        check_period = self.get_parameter('check_period_sec').value
        self._check_timer = self.create_timer(
            check_period, self._check_data_received)

        self.get_logger().info(
            f'Accumulating "{points_topic}" -> "{elevation_map_topic}" '
            f'(resolution={self._resolution} m/cell, target_frame='
            f'"{self._target_frame}"), publishing once on '
            f'"{path_status_topic}"')
        self.get_logger().info(
            '드론은 84m 상공에서 내려다보는 스캔이라 지상 로봇보다 셀당 점군 '
            '밀도가 훨씬 낮을 수 있음 - resolution=0.10m 기준으로 count==0인 '
            '빈 셀(NaN)이 많이 남는지, 콜드스타트/cos_theta=1.0 폴백 비율이 '
            '높은지 실측 필요 (top-of-file 7번 항목 참고).')

    def _points_callback(self, msg):
        self._last_received = self.get_clock().now()

        source_frame = self._target_source_frame or msg.header.frame_id
        try:
            # ground_elevation_mapper와 동일: cloud 자체의 측정 시점(stamp)으로
            # TF를 조회한다. wait timeout 없이 즉시 조회하고, 아직 없으면
            # 이 cloud는 버린다(단일 스레드 executor가 블로킹되지 않도록).
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
        # ground_elevation_mapper는 여기서 tf2_sensor_msgs.do_transform_cloud로
        # 전체 PointCloud2를 재조립한 뒤 다시 xyz만 뽑아 썼는데, 그 함수가
        # create_cloud() 호출 시 point_step을 넘기지 않아서(라이브러리 버그) 필드
        # 뒤에 trailing padding이 있는 클라우드에서 항상 AssertionError로
        # 죽는다. /drone/points(gz gpu_lidar, x/y/z/intensity/ring,
        # point_step=32바이트인데 필드 총합은 26바이트 - 6바이트 패딩)에서 실측으로
        # 확인됨. 우리는 애초에 elevation 계산에 x,y,z만 필요하므로 굳이 전체
        # 클라우드를 재조립하지 않고, xyz만 뽑아 tf2_sensor_msgs의 회전행렬
        # 변환 함수(transform_points)로 직접 좌표만 바꾼다 - intensity/ring은
        # 애초에 안 쓰므로 버려도 무방.
        points = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.shape[0] == 0:
            return

        # skip_nans=True는 NaN만 걸러낸다(그리고 cloud.is_dense가 False일 때만
        # 동작함 - read_points 내부 로직). gz의 gpu_lidar는 최대 사거리(170m)
        # 밖이라 반사가 없는 점을 NaN이 아니라 Inf로 채운다 - 실측 결과 32768개
        # 점 중 25139개(76%)가 Inf였다. 이걸 안 걸러내면 원점 기준 좌표가
        # 무한대가 되어 _grow_to_fit이 배열을 무한히 키우려다 죽는다
        # (ValueError: array is too big).
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] == 0:
            return

        points = transform_points(points, transform)
        xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
        row_idx = np.floor((xs - self._origin_x) / self._resolution).astype(np.int64)
        col_idx = np.floor((ys - self._origin_y) / self._resolution).astype(np.int64)
        row_idx, col_idx = self._grow_to_fit(row_idx, col_idx)

        # Module D와 동일: 점을 셀별 합계(bincount)로 벡터화 비닝한 뒤, 칼만
        # 갱신 자체는 점 단위가 아니라 이 콜백(스캔) 배치 단위로 셀당 한 번만
        # 수행한다.
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

        # np.nonzero가 불리언 마스크가 아니라 명시적인 (row, col) 인덱스
        # 배열을 주므로, 아래에서 이 인덱스로 팬시 인덱싱해 만드는 배열들은
        # 전부 독립된 복사본이다 -- 같은 베이스 배열에 불리언 마스크를 연쇄
        # 적용할 때 생길 수 있는 앨리어싱/뷰 버그 위험이 없다.
        touched_rows, touched_cols = np.nonzero(batch_count)
        if touched_rows.size == 0:
            return

        counts = batch_count[touched_rows, touched_cols]
        batch_mean_x = batch_sum_x[touched_rows, touched_cols] / counts
        batch_mean_y = batch_sum_y[touched_rows, touched_cols] / counts
        batch_mean_z = batch_sum_z[touched_rows, touched_cols] / counts

        sensor_origin = (transform.translation.x, transform.translation.y, transform.translation.z)
        r_eff = self._measurement_noise(
            touched_rows, touched_cols, batch_mean_x, batch_mean_y, batch_mean_z,
            counts, sensor_origin)
        self._kalman_update_cells(touched_rows, touched_cols, batch_mean_z, r_eff)

        self._last_stamp = msg.header.stamp

    def _measurement_noise(
            self, rows, cols, mean_x, mean_y, mean_z, counts, sensor_origin):
        """Return R_eff for each of the given touched cells (top-of-file 7번 항목).

        R_point combines distance, incidence angle, and point density; R_eff
        divides it by how many of this batch's points landed in that cell.
        Module D의 _measurement_noise와 동일한 설계.
        """
        sx, sy, sz = sensor_origin
        distance = np.sqrt((mean_x - sx) ** 2 + (mean_y - sy) ** 2 + (mean_z - sz) ** 2)

        # 각 셀의 표면 법선을 *이전까지 누적된* self._elevation에서
        # np.gradient로 근사한다 -- 점 단위 이웃 탐색이나 별도 포인트클라우드
        # 라이브러리 없이, 이미 비닝된 높이 격자만 사용한다. 이번 배치 자신의
        # 칼만 갱신(아래) 전에 계산하므로 이전 스캔들의 값만 반영한다.
        if self._elevation.shape[0] < 2 or self._elevation.shape[1] < 2:
            # np.gradient는 미분할 축이 너무 짧으면(예: 첫 스캔의 점들이 전부
            # 한 행/열에만 들어간 경우) NaN이 아니라 예외를 던진다. 이웃
            # 정보가 아직 없는 것과 똑같이 취급해, 아래에서 cos_theta=1.0으로
            # 폴백시킨다.
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
        # 이 셀에서 법선을 구할 이웃 정보가 없다 -- 처음 관측되는 셀
        # (self._elevation이 아직 NaN)이거나, 이웃들이 np.gradient에 충분한
        # 정보를 못 주는 경우(결과가 NaN)다. "수직으로 정면 입사"
        # (cos_theta=1.0, 입사각 보정의 R_point 기여가 가장 작아지는 가장
        # 보수적인 가정)로 폴백한다. 드론은 84m 고도 특성상(top-of-file 6/7번
        # 항목) 지상 로봇보다 이 폴백이 훨씬 자주 발동할 것으로 예상되며,
        # 이는 버그가 아니라 고도로 인한 정상적인 특성이다.
        cos_theta = np.where(np.isnan(cos_theta), 1.0, cos_theta)
        cos_theta = np.clip(cos_theta, self._incidence_cos_floor, 1.0)

        # 거리 기반 R: 계수 근사식이 아니라 데이터시트 실측 구간별 정밀도(1시그마)
        # 조회. np.searchsorted(..., side='left')로 distance가 속하는 구간을
        # 찾고, np.clip으로 마지막 구간 밖(> max_distances[-1])도 마지막 구간의
        # 시그마로 클램프한다 -- 이 프로젝트가 쓰는 OS1-32는 최대 사거리가
        # 90~170m라 100m(마지막 구간 상한)를 넘는 관측이 실제로 들어올 수 있다
        # (드론은 84m 고도 수직 스캔이라 특히 그렇다).
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
        is assumed static for the duration of the drone's scan path, so
        there is nothing for a predict step to model between scans. Module D의
        _kalman_update_cells와 동일한 설계 (top-of-file 7번 항목).
        """
        x_prev = self._elevation[rows, cols]
        p_prev = self._variance[rows, cols]

        # P가 아직 NaN인 셀은 한 번도 관측된 적이 없다 -- 이번 배치로 바로
        # 초기화하고, 아직 비교할 기존 추정치가 없으니 게이팅하지 않는다.
        is_new = np.isnan(p_prev)
        new_rows, new_cols = rows[is_new], cols[is_new]
        self._elevation[new_rows, new_cols] = batch_mean[is_new]
        self._variance[new_rows, new_cols] = r_eff[is_new]

        is_existing = ~is_new
        ex_rows, ex_cols = rows[is_existing], cols[is_existing]
        x, p, r, z = (
            x_prev[is_existing], p_prev[is_existing],
            r_eff[is_existing], batch_mean[is_existing])

        # 이노베이션 게이팅: 이미 추정치가 있는 셀에서 이상치 배치가 필터를
        # 오염시키지 않도록 거부한다. 임계값 9.0 -- 카이제곱분포 자유도 1,
        # 약 3-시그마에 해당하는 고전적 추적이론의 표준 게이팅 값(Bar-Shalom).
        innovation = z - x
        innovation_covariance = p + r
        passed_gate = (innovation ** 2 / innovation_covariance) <= self._innovation_gate_threshold

        upd_rows, upd_cols = ex_rows[passed_gate], ex_cols[passed_gate]
        gain = p[passed_gate] / innovation_covariance[passed_gate]
        self._elevation[upd_rows, upd_cols] = x[passed_gate] + gain * innovation[passed_gate]
        self._variance[upd_rows, upd_cols] = (1.0 - gain) * p[passed_gate]
        # 게이트를 통과하지 못한 셀은 x, P 둘 다 이전 값을 그대로 유지하고,
        # 이번 배치의 관측값은 버려진다.

    def _grow_to_fit(self, row_idx, col_idx):
        """Pad the grid so row_idx/col_idx fit, remapped into the new array.

        README 3.3: the map only grows to cover what has actually been
        observed, it is never pre-sized.
        """
        if self._elevation is None:
            min_row, max_row = int(row_idx.min()), int(row_idx.max())
            min_col, max_col = int(col_idx.min()), int(col_idx.max())
            shape = (max_row - min_row + 1, max_col - min_col + 1)
            # 0이 아니라 NaN -- __init__의 self._elevation/_variance 주석 참고.
            # variance 배열이 0으로 채워지면 그 셀의 칼만 게인이 영구히 0으로
            # 고정되는 치명적인 버그가 된다.
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
            # constant_values=np.nan (np.pad 기본값인 0이 아님) -- 위와 동일한
            # 이유.
            self._elevation = np.pad(self._elevation, pad_width, constant_values=np.nan)
            self._variance = np.pad(self._variance, pad_width, constant_values=np.nan)
            self._origin_x -= pad_before_row * self._resolution
            self._origin_y -= pad_before_col * self._resolution
            row_idx = row_idx + pad_before_row
            col_idx = col_idx + pad_before_col

        return row_idx, col_idx

    def _path_status_callback(self, msg):
        # drone_path_player가 경로 재생을 완전히 끝낸 순간(_finish()) 딱 한 번
        # True를 발행한다 - 그 신호를 받으면 누적된 지도를 한 번만 발행한다.
        # 이후 재발행(또는 늦게 join한 구독자에게 latched로 다시 전달되는 것)
        # 때문에 중복 발행하지 않도록 self._published로 막는다.
        if not msg.data or self._published:
            return
        if self._elevation is None:
            self.get_logger().warn(
                'path_status가 완료를 알렸지만 누적된 점이 없어 발행할 지도가 '
                '없음.')
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
        # README 3.3: unobserved cells stay NaN -- self._elevation/_variance는
        # 미관측 셀에 이미 NaN을 갖고 있으므로(__init__/_grow_to_fit 참고),
        # 예전 running-average 버전처럼 별도로 count로 나눌 필요가 없다.
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
        # elevation_variance도 elevation과 완전히 동일한 축 뒤집기 +
        # column-major 패킹 규약(_pack_layer)을 재사용한다. 문서화된 규약대로
        # 구현했을 뿐, 실제 RViz2/grid_map_rviz_plugin으로 셀 방향이 맞는지
        # 시각적으로 확인한 적은 없다 -- 실환경 미검증.
        # dim 크기는 std_msgs/MultiArrayLayout 규약을 따른다: 차원은 바깥->안
        # 순서이고, 최내곽 차원은 stride == size 여야 한다. Eigen 열 우선 저장
        # 기준으로 바깥 차원이 열(column_index), 안쪽 차원이 행(row_index)이므로
        # dim[0].size = 열 개수, dim[1].size = dim[1].stride = 행 개수다.
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
        # basic_layers에는 'elevation'만 남긴다 -- 분산은 이미 관측된 셀의
        # 부가 불확실성 정보일 뿐, 관측 여부 판단 기준이 아니다. Module D와
        # 동일 (top-of-file 7번 항목).
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
    node = DroneElevationMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
