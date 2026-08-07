import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class MockPublisher(Node):
    def __init__(self):
        super().__init__('mock_publisher')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.map_pub = self.create_publisher(OccupancyGrid, '/wheel/nav_map', 10)
        self.points_pub = self.create_publisher(PointCloud2, '/wheel/points', 10)

        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        
        self.get_logger().info('Mock Publisher Started: generating TF, Map, and Points (flat + wall + slope)')

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        # 1. Publish TF (map -> odom -> base_link -> os1_lidar)
        # map -> odom
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.translation.x = 0.0
        t_map_odom.transform.translation.y = 0.0
        t_map_odom.transform.translation.z = 0.0
        t_map_odom.transform.rotation.w = 1.0

        # odom -> base_link
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = now
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_link'
        t_odom_base.transform.translation.x = 0.0
        t_odom_base.transform.translation.y = 0.0
        t_odom_base.transform.translation.z = 0.0
        t_odom_base.transform.rotation.w = 1.0

        # base_link -> os1_lidar (lidar is 0.5m above base_link)
        t_base_lidar = TransformStamped()
        t_base_lidar.header.stamp = now
        t_base_lidar.header.frame_id = 'base_link'
        t_base_lidar.child_frame_id = 'os1_lidar'
        t_base_lidar.transform.translation.x = 0.0
        t_base_lidar.transform.translation.y = 0.0
        t_base_lidar.transform.translation.z = 0.5
        t_base_lidar.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform([t_map_odom, t_odom_base, t_base_lidar])

        # 2. Publish OccupancyGrid (nav_map)
        grid = OccupancyGrid()
        grid.header.stamp = now
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.1
        grid.info.width = 100
        grid.info.height = 100
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        # Fill with 0 (free space)
        grid.data = [0] * (100 * 100)
        self.map_pub.publish(grid)

        # 3. Publish PointCloud2 (flat ground, wall, slope)
        # All points are relative to os1_lidar, which is at z = 0.5 relative to ground
        points = []

        # (a) Flat ground: x in [0, 5], y in [-3, 3], z = -0.5
        for x in np.arange(0.0, 5.0, 0.2):
            for y in np.arange(-3.0, 3.0, 0.2):
                points.append([float(x), float(y), -0.5])

        # (b) Wall obstacle: x = 3.0, y in [-1, 1], z in [-0.5, 1.0]
        for y in np.arange(-1.0, 1.0, 0.1):
            for z in np.arange(-0.5, 1.0, 0.1):
                points.append([3.0, float(y), float(z)])

        # (c) Slope: x in [0, 5], y in [3, 5] (to the left)
        # Slope rises 0.3m for every 1m in X direction. z_base_link = 0.3 * x
        # Since lidar is at z_base_link = 0.5, z_lidar = z_base_link - 0.5
        # z_lidar = 0.3 * x - 0.5
        for x in np.arange(0.0, 5.0, 0.2):
            for y in np.arange(3.0, 5.0, 0.2):
                z_lidar = (0.3 * x) - 0.5
                points.append([float(x), float(y), float(z_lidar)])

        # Create PointCloud2 message
        import std_msgs.msg
        header = std_msgs.msg.Header()
        header.stamp = now
        header.frame_id = 'os1_lidar'
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.points_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
