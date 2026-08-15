#!/usr/bin/env python3
"""84m velocity bag -> baseline(GT pose만) / 방법B(GT pose 초기값 + GICP 정합) 누적 cloud.

GT pose는 bag의 /tf(map -> drone/base_link, 물리엔진이 실제로 계산한 진짜 위치 --
velocity_path_follower가 명령한 경로가 아니라 시뮬레이터가 아는 실제 물리 위치)를
+ /tf_static(base_link -> os1_lidar)을 tf2 Buffer에 그대로 먹여서 오프라인으로
map -> os1_lidar를 매 스캔 시각마다 조회한다(ros2 노드 실행 없이).

baseline: 그 GT pose로 스캔 포인트를 map 프레임으로 그대로 옮긴다(정합 없음) --
          RESULTS.md의 "84m GT 기준선"과 같은 방법론(GT pose만으로 누적).
방법B  : 그 GT pose를 초기 정렬(init_T_target_source)로 GICP(small_gicp)에 주고,
          직전 N개 스캔(월드 프레임, 이미 방법B로 정렬된 것)을 타겟으로 미세정합한
          결과 pose로 스캔을 누적한다. GICP가 수렴 실패하면 GT pose 그대로 씀(안전
          쪽으로 fallback -- 실패를 정합 오류로 왜곡하지 않기 위함).

출력: baseline .npy, 방법B .npy (둘 다 Nx3 float32, map 프레임 -- feed_cloud.py가
바로 먹을 수 있는 포맷).

    python3 build_methodB_cloud.py <bag_dir> <baseline.npy> <methodb.npy>
"""
import sys

import numpy as np
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer

import small_gicp

TARGET_FRAME = 'map'
WINDOW_SCANS = 6           # 방법B GICP 타겟으로 쓰는 직전 스캔 개수(시공간적으로 겹치는 스캔)
GICP_DOWNSAMPLE = 0.3      # m -- config_preprocess.json의 downsample_resolution과 동일
GICP_MAX_CORR_DIST = 1.0   # m
GICP_THREADS = 4


def transform_to_matrix(tf: TransformStamped) -> np.ndarray:
    t = tf.transform.translation
    q = tf.transform.rotation
    R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def main():
    bag_path, out_baseline, out_methodb = sys.argv[1], sys.argv[2], sys.argv[3]

    reader = open_reader(bag_path)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/drone/points' not in type_map:
        print('bag에 /drone/points 없음 -- 경로 확인', file=sys.stderr)
        sys.exit(1)

    buffer = Buffer()
    baseline_chunks, methodb_chunks = [], []
    window = []  # 직전 스캔들의 월드프레임 점(방법B 궤적으로 정렬된 것)

    n_scans = n_tf_miss = n_gicp_ok = n_gicp_fail = n_gicp_skip_first = 0

    while reader.has_next():
        topic, data, _t = reader.read_next()
        msg = deserialize_message(data, get_message(type_map[topic]))

        if topic == '/tf_static':
            for tr in msg.transforms:
                buffer.set_transform_static(tr, 'default_authority')
            continue
        if topic == '/tf':
            for tr in msg.transforms:
                buffer.set_transform(tr, 'default_authority')
            continue
        if topic != '/drone/points':
            continue

        stamp = msg.header.stamp
        lidar_frame = msg.header.frame_id
        try:
            tf = buffer.lookup_transform(TARGET_FRAME, lidar_frame, Time.from_msg(stamp))
        except Exception:
            n_tf_miss += 1
            continue

        T_gt = transform_to_matrix(tf)

        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            continue
        n_scans += 1

        pts_h = np.hstack([pts, np.ones((len(pts), 1))])

        # --- baseline: GT pose 그대로 ---
        world_gt = (T_gt @ pts_h.T).T[:, :3].astype(np.float32)
        baseline_chunks.append(world_gt)

        # --- 방법B: GT를 초기값으로 GICP 정합 ---
        if window:
            target = np.concatenate(window, axis=0)
            try:
                result = small_gicp.align(
                    target, pts, init_T_target_source=T_gt,
                    registration_type='GICP',
                    downsampling_resolution=GICP_DOWNSAMPLE,
                    max_correspondence_distance=GICP_MAX_CORR_DIST,
                    num_threads=GICP_THREADS)
                if result.converged:
                    T_refined = result.T_target_source
                    n_gicp_ok += 1
                else:
                    T_refined = T_gt
                    n_gicp_fail += 1
            except Exception as e:
                T_refined = T_gt
                n_gicp_fail += 1
        else:
            T_refined = T_gt
            n_gicp_skip_first += 1

        world_refined = (T_refined @ pts_h.T).T[:, :3]
        methodb_chunks.append(world_refined.astype(np.float32))

        window.append(world_refined)
        if len(window) > WINDOW_SCANS:
            window.pop(0)

        if n_scans % 50 == 0:
            print(f'  스캔 {n_scans}개 처리, GICP 성공 {n_gicp_ok} 실패 {n_gicp_fail}',
                  file=sys.stderr)

    baseline_arr = (np.concatenate(baseline_chunks, axis=0)
                     if baseline_chunks else np.zeros((0, 3), dtype=np.float32))
    methodb_arr = (np.concatenate(methodb_chunks, axis=0)
                    if methodb_chunks else np.zeros((0, 3), dtype=np.float32))

    np.save(out_baseline, baseline_arr)
    np.save(out_methodb, methodb_arr)

    print(f'스캔 {n_scans}개 (TF 조회 실패로 스킵 {n_tf_miss}개)')
    print(f'GICP: 성공 {n_gicp_ok}, 실패(GT로 대체) {n_gicp_fail}, 첫 스캔이라 스킵 {n_gicp_skip_first}')
    print(f'baseline 점 {len(baseline_arr)}개 -> {out_baseline}')
    print(f'방법B    점 {len(methodb_arr)}개 -> {out_methodb}')


if __name__ == '__main__':
    main()
