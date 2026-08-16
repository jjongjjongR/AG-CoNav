#!/usr/bin/env python3
"""①번 R(측정노이즈) 실측 보정용 정적(teleport) 캘리브레이션.

목표 평탄 지점 P0(-25.0, -100.0, 1.4177, 실측 GT -- 아래 "P0 선정 경위" 참고) 상공에서,
12개 (거리,입사각) 조건마다
드론을 SetEntityPose로 순간이동시켜 정지 상태로 LiDAR 스캔을 반복 수집하고,
P0 주변(반경 TARGET_RADIUS)에 떨어진 점들의 실측 높이가 GT 평탄고도로부터
얼마나 흩어지는지(표준편차)를 조건별로 계산한다.

사전 조사(이 스크립트 작성 전에 확인, PROGRESS.md에도 기록):
- model.sdf의 os1_lidar 센서 정의에 <noise> 엘리먼트가 없다 -- 이 시뮬레이터의
  gpu_lidar는 기본적으로 노이즈가 주입되지 않는 결정론적 광선추적이다. 따라서
  "정지 상태에서 반복 스캔"만으로는 진짜 센서 노이즈를 재현 못 할 수 있고,
  이 스크립트가 실제로 측정하는 것은 (a) <range><resolution>0.01</resolution>
  양자화, (b) 목표 반경 내 지형 미세 요철(위 SurfaceModel 격자 캐시가 못 잡아낸
  0.5m 이하 스케일 굴곡), (c) 채널/방위각 이산화로 인한 목표점과 실제 히트점의
  근소한 위치 차이 -- 이 셋의 합성 효과다. 기존 R 표(0.007~0.05m)는 실물
  Ouster OS1-32 스펙에서 온 값이므로, 이번 실측값이 그보다 훨씬 작게 나올 수
  있다는 것을 미리 밝혀둔다(결과 해석 시 반드시 이 사실과 함께 보고).

P0 선정 경위(모델 예측만으로는 실패, 실측 프로빙으로 정정 -- PROGRESS.md 상세 기록):
height_map.png 기반 SurfaceModel(0.5m 격자) 1차 후보(-20.241,-112.194)는 실측
결과 R=0.5~2m 안에 z=3.9~4.1m 이질점이 섞여 있어 폐기(SurfaceModel이 못 잡은
정적 오브젝트로 추정). 대체 후보(53.587,-165.319, R=2m z_std<0.0023m)는
자연 지형치고 비현실적으로 평탄해 건물 지붕일 개연성으로 배제. 최종
(-25.0,-100.0)을 실측(8회 반복스캔, R=0.5m, 115점, std=0.0064m, 범위
[1.408,1.430]m)으로 검증해 채택 -- GT 고도는 SurfaceModel 예측(1.2021, 이
지점 실측과 0.22m 차이나며 0.5m 격자 해상도 한계로 판단)이 아니라 이 실측
평균(1.4177m)을 쓴다. "인접 픽셀 고도차 0.005m 미만" 완전 평탄 구역은 이
지형에서 사실상 못 찾았다(모델 기준 0.003m 임계값도 3m 반경에서 92픽셀뿐,
실측 기준도 R=0.5m std 0.006m가 최선) -- 지시문의 "대안 판단" 조항에 따라
목표 반경 0.5m로 좁히고 실측 검증을 모델 예측 위에 추가하는 것으로 대응.

사용법: python3 calibrate_noise.py [--pilot]
  --pilot: 조건 1개(5m/0도)만, 스캔 10회만 수집하는 빠른 사전 점검 모드.
"""
import sys
import time
import math
import json
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points
from rclpy.time import Time

P0 = np.array([-25.0, -100.0, 1.4177])
TARGET_RADIUS = 0.5      # m, P0 주변 이 반경 안의 map-frame 점만 채택
MIN_RANGE = 2.5          # drone_elevation_mapper.py와 동일 자기반사 필터
SENSOR_MAX_RANGE = 170.0
DISTANCES = [5.0, 20.0, 50.0, 84.0]
ANGLES_DEG = [0.0, 60.0, 75.0]
SCANS_PER_COND = 50
WARMUP_SCANS = 5   # 새 pose 발행 후 렌더/물리 안정화 대기용으로 버리는 스캔 수


def quat_pitch(theta_rad):
    """world -Z(down)를 -X쪽으로 theta만큼 기울이는 Y축 회전 쿼터니언 (x,y,z,w)."""
    return (0.0, math.sin(theta_rad / 2.0), 0.0, math.cos(theta_rad / 2.0))


def sensor_pose_for(distance, angle_deg):
    theta = math.radians(angle_deg)
    x_off = distance * math.sin(theta)
    h = distance * math.cos(theta)
    pos = (P0[0] + x_off, P0[1], P0[2] + h)
    quat = quat_pitch(theta)
    return pos, quat


class Calibrator(Node):
    def __init__(self):
        super().__init__('calibrate_noise')
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/cmd_pose', qos_pub)

        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(PointCloud2, '/drone/points', self._cb, qos_sub)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._collecting = False
        self._warmup_left = 0
        self._collected = []  # list of dict per scan-passing-filter summary
        self._target_scans = 0

    def _cb(self, msg):
        if not self._collecting:
            return
        if self._warmup_left > 0:
            self._warmup_left -= 1
            return
        if len(self._collected) >= self._target_scans:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException as ex:
            self.get_logger().warn(f'TF fail (스킵): {ex}')
            return

        raw = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if raw.shape[0] == 0:
            self._collected.append({'n': 0})
            return
        finite = raw[np.isfinite(raw).all(axis=1)]
        if finite.shape[0] == 0:
            self._collected.append({'n': 0})
            return
        ranges_local = np.linalg.norm(finite, axis=1)
        keep = ranges_local >= MIN_RANGE
        finite = finite[keep]
        ranges_local = ranges_local[keep]
        if finite.shape[0] == 0:
            self._collected.append({'n': 0})
            return

        mapped = transform_points(finite, transform.transform)
        d_xy = np.linalg.norm(mapped[:, :2] - P0[:2], axis=1)
        near = d_xy <= TARGET_RADIUS
        if not near.any():
            self._collected.append({'n': 0})
            return

        pts = mapped[near]
        rng = ranges_local[near]
        # 센서 위치(map frame)는 transform.transform.translation으로 알 수 있다.
        sx = transform.transform.translation.x
        sy = transform.transform.translation.y
        sz = transform.transform.translation.z
        dz = np.abs(pts[:, 2] - sz)
        # 입사각(평탄면=수직법선 가정): 광선과 수직 사이 각 = arccos(|dz|/range)
        with np.errstate(invalid='ignore'):
            inc = np.degrees(np.arccos(np.clip(dz / np.maximum(rng, 1e-6), -1, 1)))

        self._collected.append({
            'n': int(pts.shape[0]),
            'z': pts[:, 2].tolist(),
            'range': rng.tolist(),
            'incidence_deg': inc.tolist(),
            'sensor_pos': [sx, sy, sz],
        })

    def teleport(self, pos, quat):
        """cmd_pose 발행 후 TF로 실제 이동을 확인한다.

        진단(diag_probe.py, PROGRESS.md 기록)으로 확인된 버그: 노드 시작
        직후 첫 teleport 호출은 /drone/cmd_pose 구독자(drone_pose_controller)와의
        DDS 디스커버리가 아직 안 끝난 상태라 publish가 그냥 유실될 수 있다(드론이
        이전 pose에 그대로 남음 -- 실제로 84m 지점에 멈춰있는 것을 확인했었다).
        그래서 발행 후 TF(map->drone/base_link)로 실제 도착 여부를 확인하고,
        도착 못 했으면 재발행한다.
        """
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat

        target = np.array(pos)
        t_deadline = time.time() + 10.0
        confirmed = False
        while time.time() < t_deadline:
            for _ in range(5):
                msg.header.stamp = self.get_clock().now().to_msg()
                self.pose_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.1)
            t_settle = time.time() + 1.0
            while time.time() < t_settle:
                rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform('map', 'drone/base_link', Time())
                actual = np.array([tf.transform.translation.x,
                                    tf.transform.translation.y,
                                    tf.transform.translation.z])
                if np.linalg.norm(actual - target) < 0.1:
                    confirmed = True
                    break
                self.get_logger().warn(
                    f'teleport 미도착(재시도): target={target.tolist()} actual={actual.tolist()}')
            except TransformException as ex:
                self.get_logger().warn(f'teleport 확인용 TF 조회 실패(재시도): {ex}')
        if not confirmed:
            raise RuntimeError(f'teleport 확인 실패(10초 타임아웃): target={target.tolist()}')
        # 물리/렌더 안정화 추가 대기.
        t_end = time.time() + 1.0
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def collect(self, n_scans, warmup=WARMUP_SCANS):
        self._collected = []
        self._target_scans = n_scans
        self._warmup_left = warmup
        self._collecting = True
        t_start = time.time()
        timeout = (n_scans + warmup) * 0.3 + 10.0  # 10Hz 기준 여유
        while len(self._collected) < n_scans and (time.time() - t_start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        self._collecting = False
        got = len(self._collected)
        if got < n_scans:
            self.get_logger().warn(f'스캔 부족: {got}/{n_scans} (timeout)')
        return list(self._collected)


RADIUS_LADDER = [0.5, 1.0, 2.0, 3.0]
# TARGET_RADIUS=0.5m는 거리·입사각이 커질수록 지면에 닿는 빔 간격(footprint)이
# 넓어져 해당 반경 안에 빔이 하나도 안 들어올 수 있다(결정론적 gpu_lidar라 스캔을
# 늘려도 안 잡힘 -- 다른 스캔도 같은 지오메트리를 반복할 뿐). 0점이면 반경을
# 점차 넓혀 재시도한다. P0 주변 평탄성은 R=2m까지는 실측 확인됐다(스크립트 상단
# docstring 참고, std<0.0023m) -- 그 이상(3m)은 미검증이라 사용 시 결과에 함께 표기.


def run_condition(node, distance, angle_deg, n_scans):
    pos, quat = sensor_pose_for(distance, angle_deg)
    node.get_logger().info(
        f'--- 조건 d={distance}m angle={angle_deg}deg -> sensor_pos={pos} ---')
    node.teleport(pos, quat)

    for radius in RADIUS_LADDER:
        global TARGET_RADIUS
        TARGET_RADIUS = radius
        scans = node.collect(n_scans)

        all_z, all_range, all_inc = [], [], []
        for s in scans:
            if s.get('n', 0) == 0:
                continue
            all_z.extend(s['z'])
            all_range.extend(s['range'])
            all_inc.extend(s['incidence_deg'])

        if all_z:
            all_z = np.array(all_z)
            all_range = np.array(all_range)
            all_inc = np.array(all_inc)
            err = all_z - P0[2]
            return {
                'distance_nominal': distance,
                'angle_nominal_deg': angle_deg,
                'n_scans_ok': sum(1 for s in scans if s.get('n', 0) > 0),
                'n_points': int(all_z.shape[0]),
                'target_radius_used_m': radius,
                'range_mean': float(all_range.mean()),
                'range_std': float(all_range.std()),
                'incidence_mean_deg': float(all_inc.mean()),
                'incidence_std_deg': float(all_inc.std()),
                'height_error_mean_m': float(err.mean()),
                'height_error_std_m': float(err.std()),
                'height_error_min_m': float(err.min()),
                'height_error_max_m': float(err.max()),
            }
        node.get_logger().warn(
            f'반경 {radius}m에서도 0점 -- 다음 단계로 확대 시도')

    return {
        'distance_nominal': distance, 'angle_nominal_deg': angle_deg,
        'n_scans_ok': 0, 'n_points': 0, 'error': 'NO_POINTS_NEAR_TARGET',
        'target_radius_tried_m': RADIUS_LADDER,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pilot', action='store_true')
    ap.add_argument('--out', default='run_results/calib_summary.json')
    ap.add_argument('--merge', action='store_true',
                     help='--out에 이미 있는(에러 없는) 조건은 건너뛰고 나머지만 채워 병합')
    args = ap.parse_args()

    rclpy.init()
    node = Calibrator()
    # 초기 dummy waypoint 완주 + drone_tf 브리지가 뜰 시간을 넉넉히 대기.
    t_end = time.time() + 5.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)

    if args.pilot:
        conds = [(5.0, 0.0)]
        n_scans = 10
    else:
        conds = [(d, a) for d in DISTANCES for a in ANGLES_DEG]
        n_scans = SCANS_PER_COND

    existing = {}
    if args.merge:
        try:
            with open(args.out) as f:
                for r in json.load(f):
                    if 'error' not in r:
                        existing[(r['distance_nominal'], r['angle_nominal_deg'])] = r
        except FileNotFoundError:
            pass

    results = []
    for d, a in conds:
        if (d, a) in existing:
            print(f'--- 조건 d={d}m angle={a}deg: 기존 결과 재사용(스킵) ---')
            results.append(existing[(d, a)])
            continue
        r = run_condition(node, d, a, n_scans)
        print(json.dumps(r, indent=2, ensure_ascii=False))
        results.append(r)
        with open(args.out, 'w') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)

    with open(args.out, 'w') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f'저장: {args.out}')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
