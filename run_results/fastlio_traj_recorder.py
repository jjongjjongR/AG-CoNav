#!/usr/bin/env python3
"""FAST-LIO2 실험 -- FAST-LIO2는 GLIM과 달리 자체적으로 궤적을 파일에
저장하지 않는다(실측 확인, spark_fast_lio.cpp에 pos_log 관련 실제
파일쓰기 코드 없음, save_dir_/sequence_name_ 파라미터는 선언만 되고 안 씀).
대신 매 프레임 `odometry`(nav_msgs/Odometry) 토픽을 발행하므로, 이 노드가
그걸 받아 TUM 포맷(t x y z qx qy qz qw)으로 저장한다.
`common.visualization_frame: "lidar"`로 설정해뒀으므로 이 궤적은 LiDAR
프레임 pose(=T_map_lidar)다 -- pure_glim_build_map.py가 쓰던 traj_lidar.txt
와 동일한 의미, 우리 맵빌더가 그대로 재사용 가능.

    python3 fastlio_traj_recorder.py <out.txt> [odom_topic]
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class FastlioTrajRecorder(Node):
    def __init__(self, out_path, odom_topic):
        super().__init__('fastlio_traj_recorder')
        self.out_path = out_path
        self.f = open(out_path, 'w')
        self.count = 0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(Odometry, odom_topic, self.cb, qos)
        self.get_logger().info(f'recording "{odom_topic}" -> {out_path} (TUM format)')

    def cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.f.write(f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
                      f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
        self.count += 1
        if self.count % 2000 == 0:
            self.f.flush()
            self.get_logger().info(f'recorded {self.count} poses')

    def destroy_node(self):
        self.f.flush()
        self.f.close()
        self.get_logger().info(f'total {self.count} poses saved to {self.out_path}')
        super().destroy_node()


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/fastlio_traj.txt'
    odom_topic = sys.argv[2] if len(sys.argv) > 2 else '/odometry'
    rclpy.init()
    node = FastlioTrajRecorder(out_path, odom_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
