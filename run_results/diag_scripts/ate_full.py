#!/usr/bin/env python3
"""Phase 2, 3단계 -- GLIM TUM 궤적(txt)을 bag의 GT(/tf map->base_link)에
Umeyama(스케일 고정, 전체 매칭쌍 사용)로 정합해 ATE를 계산한다.
run_results/ate_align.py와 완전히 동일한 방법론(RESULTS.md 10절 기준,
84m 실험 GLIM ATE~23m 비교 대상)이지만, EST 궤적을 bag 토픽이 아니라
GLIM dump TUM txt 파일에서 읽는다는 점만 다르다.

    python3 ate_full.py <bag_dir> <glim_traj_txt>
"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_gt(bag_dir, child_frame='drone/base_link'):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    times, trans = [], []
    while reader.has_next():
        t, data, _ = reader.read_next()
        if t != '/tf':
            continue
        m = deserialize_message(data, TFMessage)
        for tr in m.transforms:
            if tr.child_frame_id != child_frame:
                continue
            p = tr.transform.translation
            times.append(stamp_of(tr))
            trans.append((p.x, p.y, p.z))
    order = np.argsort(times)
    return np.array(times)[order], np.array(trans)[order]


def read_glim_txt(path):
    data = np.loadtxt(path)
    return data[:, 0], data[:, 1:4]


def umeyama(src, dst, fix_scale=True):
    mu_src, mu_dst = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_src, dst - mu_dst
    cov = dst_c.T @ src_c / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1
    R = U @ S @ Vt
    if fix_scale:
        s = 1.0
    else:
        var_src = (src_c ** 2).sum() / len(src)
        s = np.trace(np.diag(D) @ S) / var_src
    t = mu_dst - s * R @ mu_src
    return R, t, s


def main():
    bag_dir, traj_txt = sys.argv[1], sys.argv[2]
    t_max = float(sys.argv[3]) if len(sys.argv) > 3 else None

    gt_t, gt_p = read_gt(bag_dir)
    est_t, est_p = read_glim_txt(traj_txt)

    if t_max is not None:
        gt_mask = gt_t <= t_max
        est_mask = est_t <= t_max
        gt_t, gt_p = gt_t[gt_mask], gt_p[gt_mask]
        est_t, est_p = est_t[est_mask], est_p[est_mask]
        print(f'(t<={t_max}s로 절단)')
    print(f'GT {len(gt_t)}개, EST {len(est_t)}개')
    print(f'GT t range: [{gt_t[0]:.2f}, {gt_t[-1]:.2f}]')
    print(f'EST t range: [{est_t[0]:.2f}, {est_t[-1]:.2f}]')

    matched_gt, matched_est = [], []
    for t, p in zip(est_t, est_p):
        i = np.searchsorted(gt_t, t)
        if i == 0 or i >= len(gt_t):
            continue
        u = (t - gt_t[i - 1]) / (gt_t[i] - gt_t[i - 1]) if gt_t[i] > gt_t[i - 1] else 0.0
        gp = gt_p[i - 1] * (1 - u) + gt_p[i] * u
        matched_gt.append(gp)
        matched_est.append(p)
    matched_gt = np.array(matched_gt)
    matched_est = np.array(matched_est)
    print(f'시간 매칭된 쌍: {len(matched_gt)}')
    if len(matched_gt) < 10:
        raise SystemExit('매칭 쌍이 너무 적습니다')

    R, t, s = umeyama(matched_est, matched_gt, fix_scale=True)
    aligned = (s * (R @ matched_est.T).T + t)
    err = np.linalg.norm(aligned - matched_gt, axis=1)
    print(f'ATE  평균 {err.mean():.3f}  중앙값 {np.median(err):.3f}  '
          f'95%% {np.percentile(err, 95):.3f}  최대 {err.max():.3f}')
    rmse = np.sqrt(((aligned - matched_gt) ** 2).mean(0))
    print(f'축별 RMSE  X {rmse[0]:.3f}  Y {rmse[1]:.3f}  Z {rmse[2]:.3f}')

    # 커버리지 비율(EST가 GT bag 전체 시간의 몇 %를 실제로 커버했는지)
    span_covered = (est_t[-1] - est_t[0]) / (gt_t[-1] - gt_t[0]) * 100
    print(f'EST가 커버한 시간 비율: {span_covered:.1f}% '
          f'(EST span {est_t[-1]-est_t[0]:.1f}s / GT span {gt_t[-1]-gt_t[0]:.1f}s)')


if __name__ == '__main__':
    main()
