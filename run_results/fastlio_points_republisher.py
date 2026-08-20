#!/usr/bin/env python3
"""FAST-LIO2(spark-fast-lio) 실험 0-3단계 -- /drone/points(x,y,z,intensity,ring
만 있음, per-point 타임스탬프 없음, GLIM 실험 때 이미 확인된 사실)를
FAST-LIO2의 ouster_ros::Point 레이아웃(x,y,z,intensity,t,reflectivity,ring,
ambient,range)에 맞춰 재발행한다.

- t(나노초, uint32): organized cloud(height=32 채널 x width=1024 방위각,
  row-major)의 column index를 발사 순서 근사로 써서
  t(col) = (col/width) * scan_period_ns 로 근사한다 -- GLIM 실험(디스큐)/
  drone_elevation_mapper.py의 _accumulate_deskewed와 정확히 동일한 근사
  방식, 이번에 새로 설계한 게 아니다.
- reflectivity/ambient/range: 우리 데이터에 없고 FAST-LIO의 OUST64
  핸들러가 실제로 읽지도 않아(x/y/z/intensity/t/ring만 사용) 0으로 채운다.
- QoS: 원본(/drone/points)은 best_effort인데 FAST-LIO의 lidar 구독은
  reliable을 요구해(spark_fast_lio.cpp 실측 확인) 그대로 remap하면 아예
  연결이 안 된다 -- 이 브리지가 QoS를 reliable/volatile로 바꿔 발행해
  문제를 해결한다(추가로 하는 일, 근거는 PROGRESS.md 참고).

    python3 fastlio_points_republisher.py [in_topic] [out_topic]
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

SCAN_PERIOD_NS = 1.0e8  # update_rate=10Hz (model.sdf) -> 0.1s -> 1e8 ns

# 출력 포인트 레이아웃(패킹, ouster_ros::Point와 필드명/타입만 일치하면 됨 --
# PCL fromROSMsg는 메시지 자체의 fields[].offset을 읽어 이름으로 매칭하므로
# C++ 구조체의 실제 메모리 정렬을 그대로 복제할 필요가 없다).
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='t', offset=16, datatype=PointField.UINT32, count=1),
    PointField(name='reflectivity', offset=20, datatype=PointField.UINT16, count=1),
    PointField(name='ring', offset=22, datatype=PointField.UINT8, count=1),
    PointField(name='ambient', offset=23, datatype=PointField.UINT16, count=1),
    PointField(name='range', offset=25, datatype=PointField.UINT32, count=1),
]
POINT_STEP = 32  # 25+4=29를 32로 여유 있게 반올림(패딩 -- 문제 없음)

OUT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring', 'ambient', 'range'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u4', '<u2', 'u1', '<u2', '<u4'],
    'offsets': [0, 4, 8, 12, 16, 20, 22, 23, 25],
    'itemsize': POINT_STEP,
})


class FastlioPointsRepublisher(Node):
    def __init__(self, in_topic, out_topic):
        super().__init__('fastlio_points_republisher')
        in_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # FAST-LIO2의 lidar 구독이 reliable을 요구(실측 확인, spark_fast_lio.cpp)
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(PointCloud2, out_topic, out_qos)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.cb, in_qos)
        self.count = 0
        self.get_logger().info(f'republishing "{in_topic}" -> "{out_topic}" '
                                f'(ouster_ros::Point 레이아웃 + 근사 t필드 추가)')

    def cb(self, msg: PointCloud2):
        width = int(msg.width)
        height = int(msg.height)
        # x/y/z/intensity(float32)와 ring(대개 uint16)은 datatype이 달라
        # read_points_numpy(단일 dtype 배열만 반환)를 못 쓴다 -- 구조화
        # 배열을 반환하는 read_points()로 필드별 이질적 타입을 그대로 읽는다.
        struct_pts = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z', 'intensity', 'ring'), reshape_organized_cloud=False)
        n = struct_pts.shape[0]
        if n == 0:
            return
        xs = struct_pts['x'].astype(np.float32)
        ys = struct_pts['y'].astype(np.float32)
        zs = struct_pts['z'].astype(np.float32)
        intens = struct_pts['intensity'].astype(np.float32)
        rings = struct_pts['ring']

        if width > 1 and height > 0 and n == width * height:
            col_idx = np.tile(np.arange(width, dtype=np.float64), height)
        else:
            # organized cloud가 아니면(비정상 메시지) 점 순서를 그대로 근사 시간으로 사용.
            col_idx = np.arange(n, dtype=np.float64) % max(width, 1)
            width = max(width, n)

        # gz의 gpu_lidar는 최대 사거리 밖(반사 없음)인 점을 NaN이 아니라
        # Inf로 채운다(이 프로젝트 전체에서 일관되게 확인/필터링해온 사실 --
        # drone_elevation_mapper.py의 _accumulate와 동일한 처리). 이걸 안
        # 거르면 FAST-LIO의 ikd-tree/ESKF가 Inf가 섞인 스캔을 받아
        # "No Effective Points"로 실패하거나 궤적이 발산할 수 있다.
        finite = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        if not np.all(finite):
            xs, ys, zs, intens, rings, col_idx = (
                xs[finite], ys[finite], zs[finite], intens[finite], rings[finite],
                col_idx[finite])
            n = xs.shape[0]
            if n == 0:
                return

        t_ns = (col_idx / width * SCAN_PERIOD_NS).astype(np.uint32)

        out = np.zeros(n, dtype=OUT_DTYPE)
        out['x'] = xs
        out['y'] = ys
        out['z'] = zs
        out['intensity'] = intens
        out['ring'] = np.clip(rings, 0, 255).astype(np.uint8)
        out['t'] = t_ns
        # reflectivity/ambient/range: 데이터 없음, FAST-LIO OUST64 핸들러가
        # 읽지 않으므로 0으로 둔다(위 docstring 참고).

        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height = 1
        out_msg.width = n
        out_msg.fields = FIELDS
        out_msg.is_bigendian = False
        out_msg.point_step = POINT_STEP
        out_msg.row_step = POINT_STEP * n
        out_msg.is_dense = False
        out_msg.data = out.tobytes()
        self.pub.publish(out_msg)

        self.count += 1
        if self.count % 2000 == 0:
            self.get_logger().info(f'republished {self.count} scans')


def main():
    in_topic = sys.argv[1] if len(sys.argv) > 1 else '/drone/points'
    out_topic = sys.argv[2] if len(sys.argv) > 2 else '/drone/points_ouster'
    rclpy.init()
    node = FastlioPointsRepublisher(in_topic, out_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
