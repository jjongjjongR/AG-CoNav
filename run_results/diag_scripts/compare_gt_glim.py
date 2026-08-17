#!/usr/bin/env python3
"""1단계 ①② -- bag의 GT(/tf map->drone/base_link)와 GLIM odom_imu.txt를
시간 정렬해 궤적을 겹쳐 그리고, 유클리드 오차의 시간에 따른 변화를 그린다.
발산이 "완만"이 아니라 "특정 시점에서 급격히 꺾이는" 지점을 자동 탐지한다.

    python3 compare_gt_glim.py <bag_dir> <glim_dump_dir> <out_prefix> [t0_header_stamp]
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def stamp_to_sec(s):
    return s.sec + s.nanosec * 1e-9


def load_gt(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    ts, xs, ys, zs = [], [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/tf':
            continue
        msg = deserialize_message(data, TFMessage)
        for tr in msg.transforms:
            if tr.child_frame_id != 'drone/base_link':
                continue
            ts.append(stamp_to_sec(tr.header.stamp))
            xs.append(tr.transform.translation.x)
            ys.append(tr.transform.translation.y)
            zs.append(tr.transform.translation.z)
    order = np.argsort(ts)
    return np.array(ts)[order], np.array(xs)[order], np.array(ys)[order], np.array(zs)[order]


def load_glim(path):
    data = np.loadtxt(path)
    return data[:, 0], data[:, 1], data[:, 2], data[:, 3]


def umeyama(src, dst, fix_scale=True):
    """src -> dst 로 맞추는 (R, t, s). Umeyama 1991 (run_results/ate_align.py와 동일)."""
    mu_src, mu_dst = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_src, dst - mu_dst
    cov = dst_c.T @ src_c / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1
    R = U @ S @ Vt
    s = 1.0 if fix_scale else np.trace(np.diag(D) @ S) / ((src_c ** 2).sum() / len(src))
    t = mu_dst - s * R @ mu_src
    return R, t, s


def main():
    bag_dir, dump_dir, out_prefix = sys.argv[1], sys.argv[2], sys.argv[3]
    align_window_s = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0

    gt_t, gt_x, gt_y, gt_z = load_gt(bag_dir)
    g_t, g_x, g_y, g_z = load_glim(f'{dump_dir}/odom_imu.txt')

    # GT/GLIM 둘 다 같은 header.stamp(sim time) 축을 쓴다 (bag 원본 그대로).
    gi_x = np.interp(g_t, gt_t, gt_x)
    gi_y = np.interp(g_t, gt_t, gt_y)
    gi_z = np.interp(g_t, gt_t, gt_z)

    # GLIM odom/map 프레임은 첫 스캔을 원점(0,0,0)으로 잡는 자체 좌표계라
    # GT(월드 절대좌표)와 상수 오프셋+회전이 있다 -- 전체 궤적이 아니라
    # "발산이 아직 없다고 볼 수 있는" 초반 구간(align_window_s)만으로
    # Umeyama(스케일 고정) 정합해, 그 정합을 전체 궤적에 적용한다.
    # 전체 구간으로 정합하면 발산한 뒷부분이 정합 자체를 오염시킨다.
    mask = g_t <= (g_t[0] + align_window_s)
    src = np.stack([g_x[mask], g_y[mask], g_z[mask]], axis=1)
    dst = np.stack([gi_x[mask], gi_y[mask], gi_z[mask]], axis=1)
    R, t, s = umeyama(src, dst, fix_scale=True)
    est = np.stack([g_x, g_y, g_z], axis=1)
    aligned = (s * (R @ est.T).T + t)
    g_x, g_y, g_z = aligned[:, 0], aligned[:, 1], aligned[:, 2]
    print(f'정합에 사용한 초반 구간: {mask.sum()}개 점 (t<={g_t[0]+align_window_s:.1f}s)')

    err = np.sqrt((g_x - gi_x) ** 2 + (g_y - gi_y) ** 2 + (g_z - gi_z) ** 2)

    # 발산 시작점 자동 탐지: 오차의 1차 미분(변화율)이 처음으로 임계값을
    # 넘어서는 시점 -- "완만한 증가"와 "급격한 꺾임"을 구분하기 위해
    # 미분값 자체가 이전 구간 중앙값의 5배를 넘는 첫 지점을 채택.
    derr = np.gradient(err, g_t)
    window = 10
    onset_idx = None
    for i in range(window, len(derr)):
        baseline = np.median(np.abs(derr[max(0, i - window):i]))
        if baseline > 1e-6 and abs(derr[i]) > 5 * baseline and err[i] > 2.0:
            onset_idx = i
            break
    onset_t = g_t[onset_idx] if onset_idx is not None else None

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    ax = axes[0]
    ax.plot(gt_x, gt_y, label='GT (/tf map->base_link)', color='tab:green', lw=2)
    ax.plot(g_x, g_y, label='GLIM odom_imu', color='tab:red', lw=1.5, alpha=0.8)
    ax.scatter([gt_x[0]], [gt_y[0]], color='k', marker='o', zorder=5, label='start')
    if onset_idx is not None:
        ax.scatter([g_x[onset_idx]], [g_y[onset_idx]], color='orange', marker='x', s=120,
                   zorder=6, label=f'divergence onset t={onset_t:.1f}s')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_title('Trajectory: GT vs GLIM')
    ax.legend(); ax.axis('equal'); ax.grid(alpha=0.3)

    ax2 = axes[1]
    ax2.plot(g_t, err, color='tab:blue')
    if onset_idx is not None:
        ax2.axvline(onset_t, color='orange', linestyle='--', label=f'onset t={onset_t:.1f}s')
    ax2.set_xlabel('t [s] (bag header.stamp)'); ax2.set_ylabel('Euclidean error [m]')
    ax2.set_title('GT-GLIM position error over time'); ax2.legend(); ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(f'{out_prefix}.png', dpi=130)
    print(f'saved {out_prefix}.png')

    print(f'GT points: {len(gt_t)}, GLIM points: {len(g_t)}')
    print(f'GLIM t range: [{g_t[0]:.2f}, {g_t[-1]:.2f}]')
    print(f'max error: {err.max():.2f}m at t={g_t[np.argmax(err)]:.2f}s')
    print(f'divergence onset (auto-detected): t={onset_t}')
    # 발산 전후 표
    for i in range(0, len(g_t), max(1, len(g_t)//40)):
        print(f'  t={g_t[i]:7.2f}  glim=({g_x[i]:8.2f},{g_y[i]:8.2f},{g_z[i]:7.2f})  '
              f'gt=({gi_x[i]:7.2f},{gi_y[i]:7.2f},{gi_z[i]:6.2f})  err={err[i]:7.2f}')


if __name__ == '__main__':
    main()
