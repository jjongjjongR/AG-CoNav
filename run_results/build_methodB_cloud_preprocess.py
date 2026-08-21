#!/usr/bin/env python3
"""build_methodB_cloud.py의 전처리 실험용 변형.

방법B(GT pose 초기값 + GICP 정합, 지도 누적은 단순평균 -- 즉 정합된 pose로
스캔을 그대로 쌓기만 함, 칼만필터 없음)에 "GICP 실행 직전" 전처리를 하나씩
끼워 넣어 테스트한다.

**설계 결정(원본과 다른 부분)**: 전처리 필터는 매 스캔의 원본 점군(pts)에
"딱 한 번" 적용되고, 그 결과(pts_proc)가 이후 전부(GICP source, 최종 맵
누적, 다음 스캔들의 GICP 타겟이 되는 window)에 그대로 쓰인다 -- 즉 필터링은
"GICP 대응점 탐색용으로만 잠깐 쓰고 버리는" 것이 아니라 그 스캔 자체를
치환한다. 이렇게 한 이유:
  1. 과제 설명 자체가 "원본 점군에 적용하는 필터 함수"라고 명시함 (GICP
     호출에만 쓰이는 임시 사본이 아니라 점군 자체를 거른다는 뜻으로 읽음).
  2. window(GICP 타겟)가 이전 반복에서 이미 필터링된 점들로 채워지므로,
     매 반복마다 window를 다시 필터링할 필요가 없다 -- 성능상 결정적으로
     유리하다(target은 최대 6스캔 누적으로 소스보다 훨씬 크다).
  3. 다운샘플링(A) 변형만 예외: `small_gicp.align()`이 어차피 내부적으로
     `downsampling_resolution` 만큼 voxel 다운샘플을 하고 나서 대응점을
     찾으므로, A는 별도 사전 필터 없이 그 파라미터를 0.3(원본 기본값)
     대신 0.10/0.20으로 바꿔서 호출하는 것으로 구현한다(중복 다운샘플링을
     피하기 위함). 이 경우 최종 맵에 쌓이는 점 밀도는 원본과 동일하다
     (다운샘플은 GICP 대응점 탐색에만 영향).
  4. B(SOR)/C(평탄면)는 필터링 후 점이 너무 적어지면(코너 구간 등 이미
     스캔 자체가 작은 경우) 원본 스캔으로 안전 폴백한다(원본 코드의
     "GICP 실패 시 GT로 폴백" 철학과 같은 방향).

baseline(.npy)은 전처리와 무관하게 항상 원본 GT pose 누적이라, 매번 다시
계산할 필요는 없지만(모든 변형에서 동일) 원본 스크립트와 동일하게 같이
생성한다 -- 코드 재사용/일관성 우선, 필요 없으면 호출부에서 지우면 된다.

    python3 build_methodB_cloud_preprocess.py <bag_dir> <baseline.npy> <methodb.npy> <preprocess>

<preprocess>: none | voxel0.10 | voxel0.20 | sor | flatness
"""
import os
import sys

import numpy as np
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer

import small_gicp

TARGET_FRAME = 'map'
WINDOW_SCANS = 6
DEFAULT_GICP_DOWNSAMPLE = 0.3   # 원본 build_methodB_cloud.py의 GICP_DOWNSAMPLE과 동일
GICP_MAX_CORR_DIST = 1.0
GICP_THREADS = 4
MAX_GICP_TRANSLATION_DEVIATION_M = 2.0

# --- 전처리 파라미터 기본값 ---
# B: SOR(통계적 이상치 제거). K=10~20, 임계값 2~3표준편차 권장 범위의 중간값.
SOR_K = 15
SOR_STD_RATIO = 2.5

# C: 평탄면 필터링. 근거(run_results/surface_model.py 조사 결과):
#   - 건물 지붕은 바닥+3.0m로 "균일"(평평함) -- 노이즈 외에는 국소 높이분산이
#     거의 0이어야 정상.
#   - 지형(height_map)은 100m 폭에 걸쳐 총 고도차 5.25m -- 완만한 경사라
#     반경 0.3~0.5m 이내 국소 분산은 원래도 작다(수 cm 이하가 정상 범위).
#   - 반면 벽/모서리/단차 근처는 반경 0.3~0.5m 안에서 수십cm~수m 급 높이차가
#     난다(주행성 판정 기준 자체가 wheel 0.08m/leg 0.15m 단차인 것과 대비됨).
#   => "평평함"과 "단차"를 가르는 임계값을 wheel 통과기준(0.08m)의 절반
#      수준으로 잡으면, 지붕/완만지형의 노이즈는 대부분 걸러지고 실제
#      단차/모서리는 남을 것으로 기대 -- 첫 시도 값이라 결과 문서에서
#      실측으로 재검토한다.
FLATNESS_RADIUS = 0.4        # m
FLATNESS_STD_THRESHOLD = 0.04  # m (z 표준편차 기준)
FLATNESS_MIN_NEIGHBORS = 4     # 이보다 이웃이 적으면 분산 추정을 신뢰 못 함 -- 안전하게 유지


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


def sor_filter(pts, k=SOR_K, std_ratio=SOR_STD_RATIO):
    """표준 SOR: 각 점의 k-최근접 평균거리가 전체 평균 대비 std_ratio*sigma를
    넘으면 이상치로 제거. 완전 벡터화(scipy cKDTree.query가 배치 처리)."""
    n = len(pts)
    if n <= k + 1:
        return pts
    tree = cKDTree(pts)
    dists, _ = tree.query(pts, k=k + 1, workers=-1)
    mean_d = dists[:, 1:].mean(axis=1)  # [:,0]은 자기 자신(거리 0)이라 제외
    mu, sigma = mean_d.mean(), mean_d.std()
    if sigma == 0:
        return pts
    keep = mean_d <= mu + std_ratio * sigma
    if keep.sum() < 4:
        return pts
    return pts[keep]


def flatness_filter(pts, radius=FLATNESS_RADIUS, std_threshold=FLATNESS_STD_THRESHOLD,
                     min_neighbors=FLATNESS_MIN_NEIGHBORS):
    """각 점 주변 반경 내 이웃들의 z 표준편차를 계산해, 표준편차가 임계값
    이하(평평함 -- 지붕이든 완만한 지형이든)면 제거. scatter-reduce로
    이웃별 평균/분산을 벡터화 계산(점마다 파이썬 루프 없음)."""
    n = len(pts)
    if n < min_neighbors + 1:
        return pts
    tree = cKDTree(pts)
    neighbor_lists = tree.query_ball_point(pts, r=radius, workers=-1)
    lengths = np.fromiter((len(nb) for nb in neighbor_lists), dtype=np.int64, count=n)
    total = int(lengths.sum())
    if total == 0:
        return pts
    src_idx = np.repeat(np.arange(n), lengths)
    nbr_idx = np.fromiter((i for nb in neighbor_lists for i in nb), dtype=np.int64, count=total)
    z = pts[:, 2]
    z_nbr = z[nbr_idx]
    sum_z = np.zeros(n)
    sum_z2 = np.zeros(n)
    np.add.at(sum_z, src_idx, z_nbr)
    np.add.at(sum_z2, src_idx, z_nbr * z_nbr)
    mean_z = sum_z / np.maximum(lengths, 1)
    var_z = np.clip(sum_z2 / np.maximum(lengths, 1) - mean_z * mean_z, 0, None)
    std_z = np.sqrt(var_z)

    keep = (lengths < min_neighbors) | (std_z > std_threshold)
    if keep.sum() < 4:
        return pts
    return pts[keep]


def resolve_preprocess(name):
    """(filter_fn, gicp_downsample_resolution) 반환. filter_fn=None이면
    스캔 자체는 그대로 두고 align()의 다운샘플 해상도만 바꾼다(A)."""
    if name == 'none':
        return None, DEFAULT_GICP_DOWNSAMPLE
    if name == 'voxel0.10':
        return None, 0.10
    if name == 'voxel0.20':
        return None, 0.20
    if name == 'sor':
        return sor_filter, DEFAULT_GICP_DOWNSAMPLE
    if name == 'flatness':
        return flatness_filter, DEFAULT_GICP_DOWNSAMPLE
    if name == 'combo':
        # 4단계에서 개별로 효과 있었던 것만 골라 조합 -- main()에서 실제 체인 구성
        return 'COMBO', DEFAULT_GICP_DOWNSAMPLE
    raise ValueError(f'알 수 없는 전처리: {name}')


def combo_filter(pts):
    """4단계 개별 검증 결과 반영해서 채운다 (결과 문서 5절 참고)."""
    pts = sor_filter(pts)
    pts = flatness_filter(pts)
    return pts


def main():
    bag_path, out_baseline, out_methodb, preprocess = (
        sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    raw_baseline, raw_methodb = out_baseline + '.raw', out_methodb + '.raw'

    filter_fn, gicp_downsample = resolve_preprocess(preprocess)
    if filter_fn == 'COMBO':
        filter_fn = combo_filter

    reader = open_reader(bag_path)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/drone/points' not in type_map:
        print('bag에 /drone/points 없음 -- 경로 확인', file=sys.stderr)
        sys.exit(1)

    buffer = Buffer()
    window = []
    n_pts_baseline = n_pts_methodb = 0
    fb = open(raw_baseline, 'wb')
    fm = open(raw_methodb, 'wb')

    n_scans = n_tf_miss = n_gicp_ok = n_gicp_fail = n_gicp_skip_first = n_gicp_implausible = 0
    n_filter_fallback = 0
    n_pts_before_filter = n_pts_after_filter = 0

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

        # --- baseline: 항상 원본(전처리 미적용) GT pose 누적 ---
        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        world_gt = (T_gt @ pts_h.T).T[:, :3].astype(np.float32)
        fb.write(world_gt.tobytes())
        n_pts_baseline += len(world_gt)

        # --- 방법B(+전처리): 필터를 스캔 자체에 적용 ---
        if filter_fn is not None:
            n_pts_before_filter += len(pts)
            pts_proc = filter_fn(pts)
            if len(pts_proc) < 10:
                pts_proc = pts
                n_filter_fallback += 1
            n_pts_after_filter += len(pts_proc)
        else:
            pts_proc = pts

        pts_proc_h = np.hstack([pts_proc, np.ones((len(pts_proc), 1))])

        if window:
            target = np.concatenate(window, axis=0)
            try:
                result = small_gicp.align(
                    target, pts_proc, init_T_target_source=T_gt,
                    registration_type='GICP',
                    downsampling_resolution=gicp_downsample,
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
            except Exception:
                T_refined = T_gt
                n_gicp_fail += 1
        else:
            T_refined = T_gt
            n_gicp_skip_first += 1

        world_refined = (T_refined @ pts_proc_h.T).T[:, :3]
        world_refined_f32 = world_refined.astype(np.float32)
        fm.write(world_refined_f32.tobytes())
        n_pts_methodb += len(world_refined_f32)

        window.append(world_refined)
        if len(window) > WINDOW_SCANS:
            window.pop(0)

        if n_scans % 200 == 0:
            print(f'  스캔 {n_scans}개 처리, GICP 성공 {n_gicp_ok} 실패 {n_gicp_fail} '
                  f'(필터폴백 {n_filter_fallback})', file=sys.stderr)
            fb.flush()
            fm.flush()

    fb.close()
    fm.close()

    print(f'전처리={preprocess} (gicp_downsample={gicp_downsample})')
    print(f'스캔 {n_scans}개 (TF 조회 실패로 스킵 {n_tf_miss}개)')
    print(f'GICP: 성공 {n_gicp_ok}, 실패(GT로 대체) {n_gicp_fail} '
          f'(그 중 수렴했지만 이동량 상한 {MAX_GICP_TRANSLATION_DEVIATION_M}m 초과로 기각 '
          f'{n_gicp_implausible}), 첫 스캔이라 스킵 {n_gicp_skip_first}')
    if filter_fn is not None:
        reduction = (1 - n_pts_after_filter / n_pts_before_filter) * 100 if n_pts_before_filter else 0
        print(f'필터링: 스캔당 폴백(점 10개 미만으로 필터 후 원본 사용) {n_filter_fallback}회, '
              f'필터 전 점 {n_pts_before_filter}개 -> 필터 후 {n_pts_after_filter}개 '
              f'({reduction:.1f}% 감소)')

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
    print(f'방법B({preprocess}) 점 {n_pts_methodb}개 -> {out_methodb}')


if __name__ == '__main__':
    main()
