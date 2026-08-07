import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import math

class GroundSegmentationNode(Node):
    def __init__(self):
        super().__init__('ground_segmentation_node')
        
        self.declare_parameter('grid_size', 0.5)
        self.declare_parameter('z_threshold', 0.15)
        self.declare_parameter('max_height', 2.0)
        self.declare_parameter('odom_frame', 'odom')

        self.grid_size = self.get_parameter('grid_size').value
        self.z_threshold = self.get_parameter('z_threshold').value
        self.max_height = self.get_parameter('max_height').value
        odom_frame_param = self.get_parameter('odom_frame').value
        
        if odom_frame_param == 'odom':
            ns = self.get_namespace().strip('/')
            self.odom_frame = f"{ns}/odom" if ns else "odom"
        else:
            self.odom_frame = odom_frame_param

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.sub = self.create_subscription(
            PointCloud2,
            'points',
            self.pointcloud_callback,
            qos_profile
        )
        self.pub = self.create_publisher(PointCloud2, 'points_filtered', 10)

    def pointcloud_callback(self, msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.5)
            )
            # Transform cloud to odom frame
            cloud_odom = do_transform_cloud(msg, transform)
        except Exception as e:
            self.get_logger().warn(f"Could not transform {msg.header.frame_id} to {self.odom_frame}: {str(e)}")
            return

        # Extract points to numpy array
        # This gives a structured array, we need x, y, z
        cloud_data = pc2.read_points_numpy(cloud_odom, field_names=("x", "y", "z"), skip_nans=True)
        
        if len(cloud_data) == 0:
            return

        # Voxel downsampling (simple approximation by taking unique coordinates divided by voxel size)
        voxel_size = 0.1
        coords = np.floor(cloud_data / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(coords, axis=0, return_index=True)
        downsampled_points = cloud_data[unique_indices]

        if len(downsampled_points) == 0:
            return

        x = downsampled_points[:, 0]
        y = downsampled_points[:, 1]
        z = downsampled_points[:, 2]

        # Grid-based minimum Z finding
        idx_x = np.floor(x / self.grid_size).astype(np.int64)
        idx_y = np.floor(y / self.grid_size).astype(np.int64)
        
        # Create a unique key for each grid cell
        keys = idx_x * 1000000000 + idx_y

        # Sort points by key to find min z for each unique key
        sort_idx = np.argsort(keys)
        keys_sorted = keys[sort_idx]
        z_sorted = z[sort_idx]

        unique_keys, first_occurrence_indices = np.unique(keys_sorted, return_index=True)
        # Using minimum.reduceat is extremely fast in numpy
        min_z_per_key = np.minimum.reduceat(z_sorted, first_occurrence_indices)
        
        # Map min_z back to the downsampled points
        key_to_min_z = dict(zip(unique_keys, min_z_per_key))
        
        # Vectorized mapping (using searchsorted since unique_keys is sorted)
        mapped_min_z = min_z_per_key[np.searchsorted(unique_keys, keys)]

        # Filter out ground points
        mask = (z > mapped_min_z + self.z_threshold) & (z < mapped_min_z + self.max_height)
        obstacle_points = downsampled_points[mask]

        if len(obstacle_points) == 0:
            # Publish empty cloud
            empty_msg = pc2.create_cloud_xyz32(cloud_odom.header, [])
            self.pub.publish(empty_msg)
            return

        # Create output PointCloud2 message
        output_msg = pc2.create_cloud_xyz32(cloud_odom.header, obstacle_points.tolist())
        self.pub.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GroundSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
