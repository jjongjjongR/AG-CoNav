#!/usr/bin/env python3
"""Phase 4 -- /drone/gps(sensor_msgs/NavSatFix, WGS84)를 glim_ext의
gnss_global 모듈이 기대하는 geometry_msgs/PoseWithCovarianceStamped(고정
데카르트 ENU 프레임)로 변환해 /gnss로 재발행한다.

world 원점(Seongdong_gu_100x100_dynamic.world의 <spherical_coordinates>,
run_results/PROGRESS.md 3-2절에서 검증됨): lat=37.54233814881853,
lon=127.06050643805561, elevation=15.4m. pymap3d.geodetic2enu로 변환하면
world 좌표계(x=East, y=North, z=Up)와 그대로 일치한다(1단계 진단에서 이미
검증된 gz-sim 표준 관례).

    python3 navsat_to_gnss_pose.py
"""
import pymap3d as pm
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped

LAT0 = 37.54233814881853
LON0 = 127.06050643805561
ALT0 = 15.4


class NavSatToGnssPose(Node):
    def __init__(self):
        super().__init__('navsat_to_gnss_pose')
        qos_in = QoSProfile(depth=100)
        qos_in.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_in.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/gnss', 100)
        self.sub = self.create_subscription(NavSatFix, '/drone/gps', self.cb, qos_in)
        self.count = 0

    def cb(self, msg: NavSatFix):
        e, n, u = pm.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, LAT0, LON0, ALT0)
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose.pose.position.x = e
        out.pose.pose.position.y = n
        out.pose.pose.position.z = u
        out.pose.pose.orientation.w = 1.0
        out.pose.covariance[0] = 1.0
        out.pose.covariance[7] = 1.0
        out.pose.covariance[14] = 1.0
        self.pub.publish(out)
        self.count += 1
        if self.count % 500 == 0:
            self.get_logger().info(f'republished {self.count} GNSS poses (last enu=({e:.2f},{n:.2f},{u:.2f}))')


def main():
    rclpy.init()
    node = NavSatToGnssPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
