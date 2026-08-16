"""Subscribes to the drone's raw LiDAR PointCloud2, transforms it into map, and
accumulates a 2.5D elevation grid, publishing the final map once the scan path
is complete.

Structure/누적 로직은 채현우님의 agconav_ground_mapping/ground_elevation_mapper.py를
그대로 재사용한다 (TF 조회 방식, numpy 기반 running-average 누적, 동적 그리드 확장,
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
        # 이 거리(센서 기준 m)보다 가까운 반사는 기체 자기 반사로 보고 버린다.
        # 0으로 두면 끈다. 스캔 고도 84 m에서 실제 지형까지는 최소 75 m라
        # 2.5 m는 실제 반사를 하나도 건드리지 않는다.
        self.declare_parameter('min_range_m', 2.5)
        # 이 거리(센서 기준 m)보다 먼 반사를 버린다. 0이면 끈다. 높이 오차가
        # 센서 거리에 비례해서(80~88 m σ 0.076 vs 115~130 m σ 0.149, 상관 +0.278)
        # 먼 점을 버리면 지도가 좋아질 수 있는지 보려고 넣은 손잡이다.
        # min_range_m 과 같은 자리에서 같은 방식으로 적용된다.
        self.declare_parameter('max_range_m', 0.0)
        # 방위각 윈도우(센서 프레임, atan2(y, x) 도). 두 값이 같으면 끈다.
        # 실제 OS1 의 azimuth window 에 대응한다 -- 각해상도는 그대로 두고
        # 쓸모없는 방향의 출력만 버린다.
        self.declare_parameter('azimuth_min_deg', 0.0)
        self.declare_parameter('azimuth_max_deg', 0.0)
        # 들어오는 점군 중 이 비율만 누적에 쓴다(1.0 = 전부). 점의 개수(λ)만
        # 줄이고 **시점 분포는 그대로** 두려는 것이다 -- 같은 비행 데이터로
        # "점이 적어서 나쁜가, 시점이 몰려서 나쁜가"를 가르는 데 쓴다.
        # 균등하게 솎는다: n 번째 점군은 int(n*r) > int((n-1)*r) 일 때만 쓴다.
        self.declare_parameter('cloud_keep_ratio', 1.0)

        points_topic = self.get_parameter('points_topic').value
        elevation_map_topic = self.get_parameter('elevation_map_topic').value
        path_status_topic = self.get_parameter('path_status_topic').value
        self._target_frame = self.get_parameter('target_frame').value
        self._target_source_frame = self.get_parameter('target_source_frame').value
        self._resolution = float(self.get_parameter('resolution').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._min_range = float(self.get_parameter('min_range_m').value)
        self._max_range = float(self.get_parameter('max_range_m').value)
        self._az_min = float(self.get_parameter('azimuth_min_deg').value)
        self._az_max = float(self.get_parameter('azimuth_max_deg').value)
        self._keep_ratio = float(self.get_parameter('cloud_keep_ratio').value)
        self._seen_clouds = 0
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
        self._n_clouds = 0
        self._n_points = 0

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
            '빈 셀(NaN)이 많이 남는지 실측 필요 (module design analysis 참고).')

    def _points_callback(self, msg):
        self._last_received = self.get_clock().now()

        # 균등 솎기. 시점 분포는 그대로 두고 개수만 줄인다.
        if self._keep_ratio < 1.0:
            n = self._seen_clouds
            self._seen_clouds = n + 1
            if int((n + 1) * self._keep_ratio) == int(n * self._keep_ratio):
                return

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

        # 기체 자기 반사 제거. 라이다가 드론 자신의 팔/로터를 때린 반사는 센서
        # 좌표계에서 거리 1 m 남짓으로 들어오는데, 변환하고 나면 지면이 아니라
        # 드론 고도(84 m)에 찍힌다. 실측: 한 스캔 4061점 중 1.0~2.0 m가 2점,
        # 나머지 4059점은 전부 5 m 이상(드론 84 m 상공 -> 실제 지형까지 최소
        # 75 m)이었다. 이 2점/스캔이 비행 내내 쌓여 저장된 지도에서 80 m대
        # 셀 220개가 됐고, RViz 색상 눈금이 1~84 m로 늘어나 지면이 전부 한 색으로
        # 뭉갰다. 더 중요한 건 모듈 F가 그 셀 주변까지 급경사=주행불가로 본다는 점.
        #
        # max_range_m 은 그 대칭이다. 오차가 센서 거리에 비례하므로(§8) 먼 점을
        # 잘라내면 남은 점의 품질이 올라간다 -- 대신 커버리지와 밀도를 잃는다.
        if self._min_range > 0.0 or self._max_range > 0.0:
            ranges = np.linalg.norm(points, axis=1)
            keep = np.ones(points.shape[0], dtype=bool)
            if self._min_range > 0.0:
                keep &= ranges >= self._min_range
            if self._max_range > 0.0:
                keep &= ranges <= self._max_range
            points = points[keep]
            if points.shape[0] == 0:
                return

        # 방위각 윈도우. 센서 프레임에서 atan2(y, x)로 방향을 재고 창 밖을 버린다.
        # 창이 -180/+180 을 감싸는 경우도 처리한다.
        if self._az_min != self._az_max:
            az = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
            if self._az_min <= self._az_max:
                keep = (az >= self._az_min) & (az <= self._az_max)
            else:
                keep = (az >= self._az_min) | (az <= self._az_max)
            points = points[keep]
            if points.shape[0] == 0:
                return

        points = transform_points(points, transform)
        xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
        row_idx = np.floor((xs - self._origin_x) / self._resolution).astype(np.int64)
        col_idx = np.floor((ys - self._origin_y) / self._resolution).astype(np.int64)
        row_idx, col_idx = self._grow_to_fit(row_idx, col_idx)

        # 셀별로 점을 비닝해서 running average height를 누적. **이번 점군이
        # 실제로 닿은 셀만** 건드린다.
        #
        # 예전에는 np.bincount(flat_idx, minlength=지도전체) 로 지도 크기의
        # 배열을 만들어 통째로 더했다. 결과는 아래와 완전히 같지만(합성 데이터
        # 80회 누적으로 점유셀·누적점 일치 확인) 비용이 점 개수가 아니라 지도
        # 크기에 비례한다. 본 맵(5,010만 셀)에서는 점군당 351 ms 가 들어
        # 10 Hz 를 감당할 수 없다. 방문 셀만 쓰면 20 ms 다.
        n_rows, n_cols = self._sum.shape
        flat_idx = row_idx * n_cols + col_idx
        uniq, inv = np.unique(flat_idx, return_inverse=True)
        self._sum.reshape(-1)[uniq] += np.bincount(inv, weights=zs)
        self._count.reshape(-1)[uniq] += np.bincount(inv)

        self._last_stamp = msg.header.stamp
        # 실제로 누적에 들어간 점군/점의 개수. 지도만 봐서는 알 수 없고,
        # 필터 실험에서 셀당 점 개수(λ)와 "조건들이 정말 같은 입력을 받았는가"를
        # 확인하는 데 쓴다.
        self._n_clouds += 1
        self._n_points += int(zs.size)

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

    def _path_status_callback(self, msg):
        # drone_path_player가 경로 재생을 완전히 끝낸 순간(_finish()) 딱 한 번
        # True를 발행한다 - 그 신호를 받으면 누적된 지도를 한 번만 발행한다.
        # 이후 재발행(또는 늦게 join한 구독자에게 latched로 다시 전달되는 것)
        # 때문에 중복 발행하지 않도록 self._published로 막는다.
        if not msg.data or self._published:
            return
        if self._sum is None:
            self.get_logger().warn(
                'path_status가 완료를 알렸지만 누적된 점이 없어 발행할 지도가 '
                '없음.')
            return
        cells = int((self._count > 0).sum())
        self.get_logger().info(
            '누적 요약: 점군 %d개, 점 %d개, 점유 셀 %d개, 셀당 %.2f점'
            % (self._n_clouds, self._n_points, cells,
               self._n_points / max(1, cells)))
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
