#!/usr/bin/env python3
"""Phase 2 파라미터 스윕용 -- 원본 mcap에서 앞 window_s초만 잘라내고,
동시에 /drone/points에 point-level t 필드를 추가한 작은 bag을 만든다.
(add_point_times_single.py + 시간 슬라이스를 합친 버전 -- 반복 스윕 시
190s 전체를 매번 재생할 필요 없이 30~40s만으로 빠르게 확인하기 위함)

    python3 make_slice_with_t.py <원본.mcap> <신규bag디렉토리> <window_s>
"""
import os
import shutil
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField

SCAN_PERIOD = 0.1


def add_times(m: PointCloud2) -> PointCloud2:
    raw = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, m.point_step)
    col = (np.arange(m.width, dtype=np.float32) / m.width * SCAN_PERIOD)
    tcol = np.repeat(col[None, :], m.height, axis=0).copy().view(np.uint8)
    out = np.concatenate([raw, tcol.reshape(m.height, m.width, 4)], axis=2)

    new = PointCloud2()
    new.header = m.header
    new.height, new.width = m.height, m.width
    new.fields = list(m.fields) + [PointField(
        name='t', offset=m.point_step, datatype=PointField.FLOAT32, count=1)]
    new.is_bigendian = m.is_bigendian
    new.point_step = m.point_step + 4
    new.row_step = new.point_step * m.width
    new.data = out.tobytes()
    new.is_dense = m.is_dense
    return new


def main():
    src_mcap, dst_dir, window_s = sys.argv[1], sys.argv[2], float(sys.argv[3])
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src_mcap, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for t in reader.get_all_topics_and_types():
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=t.name, type=t.type, serialization_format='cdr'))

    t0_ns = None
    n_pts, n_msgs = 0, 0
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if t0_ns is None:
            t0_ns = stamp_ns
        rel_s = (stamp_ns - t0_ns) * 1e-9
        if rel_s > window_s:
            break
        if topic == '/drone/points':
            data = serialize_message(add_times(deserialize_message(data, PointCloud2)))
            n_pts += 1
        writer.write(topic, data, stamp_ns)
        n_msgs += 1
    del writer
    print(f'슬라이스 [0,{window_s}s], 메시지 {n_msgs}건(스캔 {n_pts}장) -> {dst_dir}')


if __name__ == '__main__':
    main()
