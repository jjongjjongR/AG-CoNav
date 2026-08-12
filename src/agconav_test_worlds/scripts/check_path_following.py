#!/usr/bin/env python3
"""Compare the drone's actual flown track against the ㄹ (lawnmower) path spec.

'경로대로 안 움직인다'는 눈으로 본 인상이라, 숫자로 확인해야 고칠 대상이
정해진다. 여기서 재는 것은 딱 네 가지다.

  1) 선 이탈(cross-track): 각 시점의 위치가 설계된 왕복선에서 옆으로 얼마나
     벗어났나. ㄹ 모양이 뭉개졌다면 여기가 커진다.
  2) 줄 간격(strip spacing): 실제로 날아간 이웃한 왕복선 사이의 거리.
     설계값(기존 코드 산출)과 어긋나면 지도에 빈 줄이 생긴다.
  3) 고도: 설계 고도를 유지했나.
  4) 속도: 순항 속도대로 날았나.

입력은 /tf(map -> drone/base_link)를 담은 rosbag2. 그게 드론이 '실제로 있던'
위치다(명령 pose가 아니라).
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import yaml


def read_track(bag, frame='drone/base_link'):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/tf':
            continue
        for tr in deserialize_message(data, TFMessage).transforms:
            if tr.child_frame_id != frame:
                continue
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            p = tr.transform.translation
            rows.append((t, p.x, p.y, p.z))
    a = np.array(sorted(rows), dtype=float)
    if len(a):                       # 같은 stamp 중복 제거
        keep = np.concatenate(([True], np.diff(a[:, 0]) > 1e-9))
        a = a[keep]
    return a


def segment_stats(track, wps):
    """Assign each sample to its nearest path segment and measure cross-track."""
    P = track[:, 1:3]
    W = np.array([[w['position']['x'], w['position']['y']] for w in wps])
    best_d = np.full(len(P), np.inf)
    best_s = np.zeros(len(P), dtype=int)
    for i in range(len(W) - 1):
        a, b = W[i], W[i + 1]
        e = b - a
        L = np.hypot(*e)
        if L < 1e-9:
            continue
        u = e / L
        r = P - a
        along = np.clip(r @ u, 0.0, L)
        proj = a + along[:, None] * u
        d = np.linalg.norm(P - proj, axis=1)
        m = d < best_d
        best_d[m], best_s[m] = d[m], i
    return best_d, best_s


def strip_spacing(W):
    """Distance between neighbouring long (scan) legs of the lawnmower."""
    legs = []
    for i in range(len(W) - 1):
        a, b = W[i], W[i + 1]
        if np.hypot(*(b - a)) > 5.0:
            legs.append((a, b))
    out = []
    for i in range(len(legs) - 1):
        (a1, b1), (a2, b2) = legs[i], legs[i + 1]
        d1, d2 = b1 - a1, b2 - a2
        n1 = np.hypot(*d1)
        # 평행한 두 다리 사이 거리만 센다(꺾는 짧은 구간 제외)
        if n1 < 1e-9:
            continue
        cosang = abs(d1 @ d2 / (n1 * np.hypot(*d2) + 1e-12))
        if cosang < 0.95:
            continue
        u = d1 / n1
        out.append(abs(np.cross(u, a2 - a1)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('path_yaml')
    ap.add_argument('--frame', default='drone/base_link')
    args = ap.parse_args()

    with open(args.path_yaml) as f:
        spec = yaml.safe_load(f)
    wps = spec['waypoints']
    W = np.array([[w['position']['x'], w['position']['y']] for w in wps])
    design_alt = float(spec.get('altitude', wps[0]['position']['z']))

    track = read_track(args.bag, args.frame)
    if len(track) < 10:
        print(f'{args.frame} 샘플이 {len(track)}개뿐입니다 - 기록 실패.')
        return 1

    # 이륙/착륙 구간을 빼고 순항만 본다(설계 고도의 ±5 m 안).
    cruise = track[np.abs(track[:, 3] - design_alt) < 5.0]
    print(f'샘플 {len(track)}개 (순항 {len(cruise)}개), '
          f'기록 {track[-1,0]-track[0,0]:.1f} 초')

    d, _ = segment_stats(cruise if len(cruise) > 10 else track, wps)
    print('\n[1] 설계 경로에서 옆으로 벗어난 거리 (작을수록 ㄹ 모양이 정확)')
    print('    평균 %.3f m | 중앙 %.3f m | 90%% %.3f m | 최대 %.3f m'
          % (d.mean(), np.median(d), np.percentile(d, 90), d.max()))

    sp = strip_spacing(W)
    print('\n[2] 설계 줄 간격')
    if sp:
        print('    ' + ', '.join('%.3f m' % s for s in sp))
        print('    평균 %.3f m (모든 줄이 같아야 지도에 빈 줄이 없다)' % np.mean(sp))
    else:
        print('    평행한 왕복선을 찾지 못했습니다.')

    z = cruise[:, 3] if len(cruise) > 10 else track[:, 3]
    print('\n[3] 고도 (설계 %.2f m)' % design_alt)
    print('    평균 %.2f m | 표준편차 %.3f m | 최소 %.2f | 최대 %.2f'
          % (z.mean(), z.std(), z.min(), z.max()))

    t = track[:, 0]
    step = np.linalg.norm(np.diff(track[:, 1:3], axis=0), axis=1)
    dt = np.diff(t)
    ok = dt > 1e-6
    v = step[ok] / dt[ok]
    v = v[v < 50.0]                      # 순간이동 점프 제외
    print('\n[4] 수평 속도')
    print('    평균 %.2f m/s | 중앙 %.2f m/s | 90%% %.2f m/s'
          % (v.mean(), np.median(v), np.percentile(v, 90)))

    total = np.linalg.norm(np.diff(track[:, 1:3], axis=0), axis=1).sum()
    design_len = sum(np.hypot(*(W[i + 1] - W[i])) for i in range(len(W) - 1))
    print('\n[5] 이동 거리  실제 %.2f m / 설계 %.2f m (%+.2f%%)'
          % (total, design_len, 100 * (total - design_len) / design_len))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
