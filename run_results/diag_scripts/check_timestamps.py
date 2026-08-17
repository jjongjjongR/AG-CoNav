#!/usr/bin/env python3
"""1단계 ③ -- /drone/points, /drone/imu의 header.stamp 동기화 진단.
bag 전체에 대해 단조증가 여부, 간격 통계를 내고, 발산 시작 추정 구간
(bag 시작 후 t=90~150s)을 집중적으로 덤프한다.

    python3 check_timestamps.py <bag_dir>
"""
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu

FOCUS_LO, FOCUS_HI = 90.0, 150.0


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    bag_dir = sys.argv[1]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))

    topics_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    assert '/drone/points' in topics_types and '/drone/imu' in topics_types

    t0 = None
    last = {'/drone/points': None, '/drone/imu': None}
    stats = {'/drone/points': {'n': 0, 'reversals': 0, 'gaps_gt2x': 0, 'deltas': []},
              '/drone/imu': {'n': 0, 'reversals': 0, 'gaps_gt2x': 0, 'deltas': []}}
    expected_dt = {'/drone/points': 0.1, '/drone/imu': 0.01}
    focus_events = []

    # bag recv time (rosbag2 stores this separately from header.stamp)
    recv_vs_header_diffs = {'/drone/points': [], '/drone/imu': []}

    while reader.has_next():
        topic, data, recv_t_ns = reader.read_next()
        if topic not in ('/drone/points', '/drone/imu'):
            continue
        msg = deserialize_message(data, PointCloud2 if topic == '/drone/points' else Imu)
        hstamp = stamp_to_sec(msg.header.stamp)
        if t0 is None:
            t0 = hstamp
        rel = hstamp - t0

        recv_t = recv_t_ns * 1e-9
        recv_vs_header_diffs[topic].append(recv_t - hstamp)

        s = stats[topic]
        s['n'] += 1
        if last[topic] is not None:
            dt = hstamp - last[topic]
            s['deltas'].append(dt)
            if dt < 0:
                s['reversals'] += 1
                if FOCUS_LO <= rel <= FOCUS_HI or True:
                    focus_events.append((topic, rel, 'REVERSAL', dt))
            elif dt > expected_dt[topic] * 2:
                s['gaps_gt2x'] += 1
                focus_events.append((topic, rel, 'GAP', dt))
        last[topic] = hstamp

    print(f'=== bag: {bag_dir} ===')
    print(f't0 (first header.stamp, epoch) = {t0:.6f}')
    for topic in ('/drone/points', '/drone/imu'):
        s = stats[topic]
        deltas = s['deltas']
        if deltas:
            import statistics
            print(f'\n--- {topic} ---')
            print(f'  n={s["n"]}, mean_dt={statistics.mean(deltas):.6f}, '
                  f'median_dt={statistics.median(deltas):.6f}, '
                  f'min_dt={min(deltas):.6f}, max_dt={max(deltas):.6f}')
            print(f'  reversals={s["reversals"]}, gaps(>2x expected)={s["gaps_gt2x"]}')
        rv = recv_vs_header_diffs[topic]
        if rv:
            import statistics
            print(f'  recv_time - header.stamp: mean={statistics.mean(rv):.6f}, '
                  f'min={min(rv):.6f}, max={max(rv):.6f}, stdev={statistics.pstdev(rv):.6f}')

    print(f'\n=== anomaly events (reversal/gap>2x), all bag, count={len(focus_events)} ===')
    for topic, rel, kind, dt in focus_events[:200]:
        print(f'  t+{rel:8.3f}s  {topic:14s} {kind:9s} dt={dt:+.6f}')
    if len(focus_events) > 200:
        print(f'  ... ({len(focus_events)-200} more, truncated)')

    print(f'\n=== focus window t+[{FOCUS_LO},{FOCUS_HI}]s anomaly count ===')
    focus_only = [e for e in focus_events if FOCUS_LO <= e[1] <= FOCUS_HI]
    print(f'  {len(focus_only)} anomalies in this window (out of {len(focus_events)} total)')
    for e in focus_only:
        print(f'  t+{e[1]:8.3f}s  {e[0]:14s} {e[2]:9s} dt={e[3]:+.6f}')


if __name__ == '__main__':
    main()
