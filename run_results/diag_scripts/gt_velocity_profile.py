#!/usr/bin/env python3
"""Phase 2, 1-1 -- bag의 GT(/tf map->drone/base_link)에서 초반(t=0~window_s)
속도 프로파일을 계산해 그린다. 1단계에서 찾은 "정지->전진 전환이
t~15~17s 근처"라는 가설을 다시 한번 시각으로 검증한다.

    python3 gt_velocity_profile.py <bag_dir> <out_prefix> [window_s]
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


def load_gt(bag_dir, t_max=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    ts, xs, ys, zs = [], [], [], []
    t0 = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/tf':
            continue
        msg = deserialize_message(data, TFMessage)
        for tr in msg.transforms:
            if tr.child_frame_id != 'drone/base_link':
                continue
            t = stamp_to_sec(tr.header.stamp)
            if t0 is None:
                t0 = t
            ts.append(t)
            xs.append(tr.transform.translation.x)
            ys.append(tr.transform.translation.y)
            zs.append(tr.transform.translation.z)
        if t_max is not None and t0 is not None and (t - t0) > t_max:
            break
    order = np.argsort(ts)
    return np.array(ts)[order], np.array(xs)[order], np.array(ys)[order], np.array(zs)[order]


def main():
    bag_dir, out_prefix = sys.argv[1], sys.argv[2]
    window_s = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0

    t, x, y, z = load_gt(bag_dir, t_max=window_s + 5)
    t0 = t[0]
    rel = t - t0
    mask = rel <= window_s
    t, x, y, z, rel = t[mask], x[mask], y[mask], z[mask], rel[mask]

    vx = np.gradient(x, t)
    vy = np.gradient(y, t)
    vz = np.gradient(z, t)
    speed_xy = np.sqrt(vx ** 2 + vy ** 2)
    speed_3d = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

    # 정지->전진 전환 탐지: xy 평면 속도가 처음으로 1.0 m/s를 넘는 시점
    # (등속비행 목표 5m/s의 20% -- 노이즈 문턱을 넉넉히 두고 "확실히
    # 움직이기 시작한" 시점을 잡기 위함).
    THRESH = 1.0
    above = np.where(speed_xy > THRESH)[0]
    onset_rel = rel[above[0]] if len(above) else None

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax = axes[0]
    ax.plot(rel, speed_xy, label='|v_xy| (GT)', color='tab:blue')
    ax.plot(rel, speed_3d, label='|v_3d| (GT)', color='tab:purple', alpha=0.6, linestyle='--')
    ax.axhline(5.0, color='gray', linestyle=':', label='cruise target 5 m/s')
    if onset_rel is not None:
        ax.axvline(onset_rel, color='orange', linestyle='--',
                   label=f'v_xy>{THRESH}m/s at t={onset_rel:.2f}s')
    ax.set_ylabel('speed [m/s]')
    ax.set_title(f'GT velocity profile, first {window_s}s (bag: {bag_dir})')
    ax.legend(); ax.grid(alpha=0.3)

    ax2 = axes[1]
    ax2.plot(rel, z, label='z (altitude)', color='tab:green')
    ax2.axhline(5.0, color='gray', linestyle=':', label='target 5m AGL')
    if onset_rel is not None:
        ax2.axvline(onset_rel, color='orange', linestyle='--')
    ax2.set_xlabel('t [s] (since first /tf sample)')
    ax2.set_ylabel('z [m]')
    ax2.legend(); ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(f'{out_prefix}.png', dpi=130)
    print(f'saved {out_prefix}.png')

    print(f'GT samples in window: {len(t)}')
    print(f'onset (v_xy > {THRESH} m/s): t={onset_rel}')
    print(f'z at t=0: {z[0]:.3f}, z at onset: '
          f'{z[above[0]] if len(above) else float("nan"):.3f}')
    for i in range(0, len(rel), max(1, len(rel)//30)):
        print(f'  t={rel[i]:6.2f}  x={x[i]:8.3f} y={y[i]:8.3f} z={z[i]:6.3f}  '
              f'v_xy={speed_xy[i]:6.3f} v_z={vz[i]:6.3f}')


if __name__ == '__main__':
    main()
