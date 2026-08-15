#!/usr/bin/env python3
"""GLIM 추정 궤적을 ground-truth 궤적(같은 bag의 /tf, map->drone/base_link)에
Umeyama(스케일 고정) 정합하고 ATE를 계산한다. RESULTS.md 10절과 동일한 방법론.

    python3 ate_align.py <bag_dir> --est-topic /glim/... --est-frame ...
"""
from __future__ import annotations

import argparse
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_tf_trajectory(bag_dir, topic, child_frame):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    times, trans = [], []
    while reader.has_next():
        t, data, _ = reader.read_next()
        if t != topic:
            continue
        m = deserialize_message(data, TFMessage)
        for tr in m.transforms:
            if child_frame and tr.child_frame_id != child_frame:
                continue
            p = tr.transform.translation
            times.append(stamp_of(tr))
            trans.append((p.x, p.y, p.z))
    return np.array(times), np.array(trans)


def umeyama(src, dst, fix_scale=True):
    """src -> dst 로 맞추는 (R, t, s). Umeyama 1991, 스케일 고정 옵션."""
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
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--gt-topic', default='/tf')
    ap.add_argument('--gt-frame', default='drone/base_link')
    ap.add_argument('--est-topic', required=True)
    ap.add_argument('--est-frame', default='')
    args = ap.parse_args()

    gt_t, gt_p = read_tf_trajectory(args.bag_dir, args.gt_topic, args.gt_frame)
    est_t, est_p = read_tf_trajectory(args.bag_dir, args.est_topic, args.est_frame)
    print(f'GT {len(gt_t)}개, EST {len(est_t)}개')
    if not len(gt_t) or not len(est_t):
        raise SystemExit('궤적을 찾지 못했습니다')

    order = np.argsort(gt_t)
    gt_t, gt_p = gt_t[order], gt_p[order]
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

    R, t, s = umeyama(matched_est, matched_gt, fix_scale=True)
    aligned = (s * (R @ matched_est.T).T + t)
    err = np.linalg.norm(aligned - matched_gt, axis=1)
    print(f'ATE  평균 {err.mean():.3f}  중앙값 {np.median(err):.3f}  '
         f'95%% {np.percentile(err, 95):.3f}  최대 {err.max():.3f}')
    rmse = np.sqrt(((aligned - matched_gt) ** 2).mean(0))
    print(f'축별 RMSE  X {rmse[0]:.3f}  Y {rmse[1]:.3f}  Z {rmse[2]:.3f}')

    np.savez('/tmp/ate_align.npz', R=R, t=t, s=s,
            ate_mean=err.mean(), ate_median=np.median(err),
            ate_p95=np.percentile(err, 95), ate_max=err.max())
    return R, t, s


if __name__ == '__main__':
    main()
