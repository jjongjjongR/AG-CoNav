#!/usr/bin/env python3
"""스캔의 각 점에 찍힌 시각(`t` 필드)을 계산해 넣는다.

Gazebo의 gpu_lidar 는 점별 타임스탬프를 주지 않는다. GLIM 은 그 경우 "받은 순서가
곧 시간 순서"라고 가정하고 0.1초를 점들에 균등 배분하는데(로그의
`use pseudo per-point timestamps based on the order of points`), 우리 스캔은
유효 반사가 방위 일부 구간에 몰려 있어 그 가정이 어긋난다.

실제 시각은 배열에서 정확히 알 수 있다. 32 x 1024 배열의 열 번호가 곧 방위 순서이고
스캔 주기는 0.1초이므로 `t = 열번호 / 1024 * 0.1` 이다.

8 m/s 로 날면 스캔 한 장을 찍는 동안 0.8 m 를 이동하므로, 이 보정 없이는 지도
칸(0.10 m) 여덟 칸어치가 뭉개진 채로 들어간다.

    python3 add_point_times.py bags/exp_velocity bags/exp_velocity_t
"""

import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField

SCAN_PERIOD = 0.1          # 10 Hz


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
    src = sys.argv[1]
    dst = sys.argv[2]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for t in reader.get_all_topics_and_types():
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=t.name, type=t.type, serialization_format='cdr'))

    n = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/drone/points':
            data = serialize_message(add_times(deserialize_message(data, PointCloud2)))
            n += 1
        writer.write(topic, data, stamp)
    del writer
    print('스캔 %d장에 per-point t 필드 추가 → %s' % (n, dst))


if __name__ == '__main__':
    main()
