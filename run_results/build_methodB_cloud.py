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

**메모리 설계(2026-08-16 5m AGL 4m/5mps 실험에서 추가)**: 이 VM은 5.8GB RAM
뿐이라, 84m 실험(스캔~1,670개)과 달리 이번처럼 스캔이 훨씬 많은 비행
(21,771개)에서는 원래 방식(파이썬 리스트에 전체 비행 분량 점을 다 들고
있다가 마지막에 한 번에 np.concatenate)이 **실제로 OOM-kill됨을 확인**
(dmesg: anon-rss 4.7GB, `Out of memory: Killed process ... python3`,
스캔 21750/21771까지 가고서 마지막 concatenate 직전에 죽음). 그래서 스캔마다
바로 디스크에 스트리밍으로 append하고, 끝에서 raw -> .npy 변환도 청크
단위로 memmap에 복사(전체를 한 번에 메모리에 올리지 않음)하도록 재작성함.
GICP 타겟용 `window`(최근 6스캔만)는 원래도 작아서 그대로 메모리에 유지.
"""
import os
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
# 2026-08-16 5m AGL 4m/5mps 실험에서 추가: `result.converged`가 True인데도
# GICP가 물리적으로 말이 안 되는 해(예: z가 84m로 튐)로 수렴하는 사례가 실제로
# 나왔다(GT통과가능 셀 44,864개/2.67억 중 0.017%가 z>15m 또는 |y|>250m 등
# 명백한 이상치 -- lawnmower 경로의 코너(180도 yaw 반전)마다 스캔이 아주
# 작아지는 구간(build 로그에 "point cloud is too small" 다수)에서 타겟/소스가
# 몇 점 안 남아 GICP가 잘못된 국소해로 "수렴"한 것으로 판단). max_correspondence_
# distance=1.0m로 탐색폭 자체가 좁으므로, GT 대비 정상 보정량은 원래 수 cm~
# 수십 cm 수준이어야 한다 -- 그래서 "GT 대비 이동량이 물리적으로 말이 안 되게
# 큰 결과는 신뢰 안 함" 이라는, 원래 코드에 이미 있던 "GICP 실패 시 GT로 안전
# 폴백" 설계를 완성하는 차원에서 이동량 상한을 추가한다.
MAX_GICP_TRANSLATION_DEVIATION_M = 2.0


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


def raw_to_npy(raw_path, npy_path, n_pts, chunk_pts=2_000_000):
    """raw_path(연속 float32 x,y,z 바이너리, 헤더 없음)를 청크 단위로 복사해
    npy_path(.npy, shape=(n_pts,3))로 변환. 전체를 한 번에 메모리에 올리지 않는다."""
    mm = np.lib.format.open_memmap(npy_path, mode='w+', dtype=np.float32, shape=(n_pts, 3))
    with open(raw_path, 'rb') as f:
        written = 0
        while written < n_pts:
            take = min(chunk_pts, n_pts - written)
            buf = f.read(take * 12)
            if not buf:
                break
            arr = np.frombuffer(buf, dtype=np.float32).reshape(-1, 3)
            mm[written:written + len(arr)] = arr
            written += len(arr)
    mm.flush()
    del mm
    os.remove(raw_path)


def main():
    bag_path, out_baseline, out_methodb = sys.argv[1], sys.argv[2], sys.argv[3]
    raw_baseline, raw_methodb = out_baseline + '.raw', out_methodb + '.raw'

    reader = open_reader(bag_path)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/drone/points' not in type_map:
        print('bag에 /drone/points 없음 -- 경로 확인', file=sys.stderr)
        sys.exit(1)

    buffer = Buffer()
    window = []  # 직전 스캔들의 월드프레임 점(방법B 궤적으로 정렬된 것) -- 최대 WINDOW_SCANS개만 유지
    n_pts_baseline = n_pts_methodb = 0
    fb = open(raw_baseline, 'wb')
    fm = open(raw_methodb, 'wb')

    n_scans = n_tf_miss = n_gicp_ok = n_gicp_fail = n_gicp_skip_first = n_gicp_implausible = 0

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
        fb.write(world_gt.tobytes())
        n_pts_baseline += len(world_gt)

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
                deviation = float(np.linalg.norm(
                    result.T_target_source[:3, 3] - T_gt[:3, 3])) if result.converged else None
                if result.converged and deviation <= MAX_GICP_TRANSLATION_DEVIATION_M:
                    T_refined = result.T_target_source
                    n_gicp_ok += 1
                else:
                    T_refined = T_gt
                    n_gicp_fail += 1
                    if result.converged:
                        n_gicp_implausible += 1
            except Exception as e:
                T_refined = T_gt
                n_gicp_fail += 1
        else:
            T_refined = T_gt
            n_gicp_skip_first += 1

        world_refined = (T_refined @ pts_h.T).T[:, :3]
        world_refined_f32 = world_refined.astype(np.float32)
        fm.write(world_refined_f32.tobytes())
        n_pts_methodb += len(world_refined_f32)

        window.append(world_refined)
        if len(window) > WINDOW_SCANS:
            window.pop(0)

        if n_scans % 50 == 0:
            print(f'  스캔 {n_scans}개 처리, GICP 성공 {n_gicp_ok} 실패 {n_gicp_fail}',
                  file=sys.stderr)
            fb.flush()
            fm.flush()

    fb.close()
    fm.close()

    print(f'스캔 {n_scans}개 (TF 조회 실패로 스킵 {n_tf_miss}개)')
    print(f'GICP: 성공 {n_gicp_ok}, 실패(GT로 대체) {n_gicp_fail} '
          f'(그 중 수렴했지만 이동량 상한 {MAX_GICP_TRANSLATION_DEVIATION_M}m 초과로 기각 '
          f'{n_gicp_implausible}), 첫 스캔이라 스킵 {n_gicp_skip_first}')

    if n_pts_baseline:
        raw_to_npy(raw_baseline, out_baseline, n_pts_baseline)
    else:
        np.save(out_baseline, np.zeros((0, 3), dtype=np.float32))
        os.remove(raw_baseline)
    if n_pts_methodb:
        raw_to_npy(raw_methodb, out_methodb, n_pts_methodb)
    else:
        np.save(out_methodb, np.zeros((0, 3), dtype=np.float32))
        os.remove(raw_methodb)

    print(f'baseline 점 {n_pts_baseline}개 -> {out_baseline}')
    print(f'방법B    점 {n_pts_methodb}개 -> {out_methodb}')


if __name__ == '__main__':
    main()
