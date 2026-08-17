#!/usr/bin/env python3
"""ate_full.py와 같은 정합(전체 매칭쌍 Umeyama, fix_scale)을 쓰되 궤적 그림과
시간별 오차 그림을 저장한다. t_max로 GT의 미션종료 후 자유낙하 구간을 제외.

    python3 ate_full_plot.py <bag_dir> <glim_traj_txt> <out_prefix> [t_max]
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
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


def umeyama(src, dst, fix_scale=True):
    mu_src, mu_dst = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_src, dst - mu_dst
    cov = dst_c.T @ src_c / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1
    R = U @ S @ Vt
    t = mu_dst - R @ mu_src
    return R, t


def main():
    bag_dir, traj_txt, out_prefix = sys.argv[1], sys.argv[2], sys.argv[3]
    t_max = float(sys.argv[4]) if len(sys.argv) > 4 else None

    gt_t, gt_p = read_gt(bag_dir)
    data = np.loadtxt(traj_txt)
    est_t, est_p = data[:, 0], data[:, 1:4]

    if t_max is not None:
        gt_t, gt_p = gt_t[gt_t <= t_max], gt_p[gt_t <= t_max]
        est_t, est_p = est_t[est_t <= t_max], est_p[est_t <= t_max]

    matched_gt, matched_est, matched_t = [], [], []
    for t, p in zip(est_t, est_p):
        i = np.searchsorted(gt_t, t)
        if i == 0 or i >= len(gt_t):
            continue
        u = (t - gt_t[i - 1]) / (gt_t[i] - gt_t[i - 1]) if gt_t[i] > gt_t[i - 1] else 0.0
        gp = gt_p[i - 1] * (1 - u) + gt_p[i] * u
        matched_gt.append(gp)
        matched_est.append(p)
        matched_t.append(t)
    matched_gt = np.array(matched_gt)
    matched_est = np.array(matched_est)
    matched_t = np.array(matched_t)

    R, t = umeyama(matched_est, matched_gt)
    aligned = (R @ matched_est.T).T + t
    err = np.linalg.norm(aligned - matched_gt, axis=1)

    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    ax = axes[0]
    ax.plot(matched_gt[:, 0], matched_gt[:, 1], label='GT', color='tab:green', lw=2)
    ax.plot(aligned[:, 0], aligned[:, 1], label='GLIM (cv1e3, aligned)', color='tab:red', lw=1, alpha=0.7)
    ax.scatter([matched_gt[0, 0]], [matched_gt[0, 1]], color='k', marker='o', zorder=5, label='start')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_title('Full-bag trajectory: GT vs GLIM (cv1e3)')
    ax.legend(); ax.axis('equal'); ax.grid(alpha=0.3)

    ax2 = axes[1]
    ax2.plot(matched_t, err, color='tab:blue', lw=0.8)
    ax2.set_xlabel('t [s]'); ax2.set_ylabel('ATE [m]')
    ax2.set_title(f'ATE over time (mean={err.mean():.1f}m, median={np.median(err):.1f}m, max={err.max():.1f}m)')
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(f'{out_prefix}.png', dpi=130)
    print(f'saved {out_prefix}.png')
    print(f'ATE mean={err.mean():.3f} median={np.median(err):.3f} '
          f'p95={np.percentile(err,95):.3f} max={err.max():.3f}')


if __name__ == '__main__':
    main()
