#!/usr/bin/env python3
"""add_point_times.py를 mcap 청크 파일 하나에만 적용하는 버전 -- 디스크가
빠듯해(원본+t필드본 동시 보유 불가) bag 전체를 한 번에 변환할 수 없어서,
청크 하나씩만 변환->GLIM 재생->삭제->다음 청크로 순환하기 위함
(run_results/PROGRESS.md 5단계 참고).

    python3 add_point_times_single.py <원본.mcap> <신규bag디렉토리>
"""
import os
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
    src_mcap = sys.argv[1]
    dst_dir = sys.argv[2]
    if os.path.exists(dst_dir):
        import shutil
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

    n = 0
    n_pts = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/drone/points':
            data = serialize_message(add_times(deserialize_message(data, PointCloud2)))
            n += 1
        writer.write(topic, data, stamp)
    del writer
    print(f'스캔 {n}장에 t 필드 추가 -> {dst_dir}')


if __name__ == '__main__':
    main()
