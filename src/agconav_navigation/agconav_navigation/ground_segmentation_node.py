import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points
from std_msgs.msg import Header
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
        # TF 수신을 이 노드가 아닌 전용 노드+스레드에 맡긴다(node=None, spin_thread=True).
        #
        # lookup_transform의 timeout은 tf2_ros 내부에서 time.sleep 루프로 대기한다.
        # TF 구독이 이 노드의 단일 스레드 실행기에 얹혀 있으면 대기하는 동안 /tf가
        # 처리되지 않아 timeout이 항상 만료되고, 10 Hz로 들어오는 다음 점군이 또
        # 0.5초를 잡아먹으며 TF가 점점 더 뒤처지는 악순환이 된다
        # (실측: leg의 TF 버퍼가 6초까지 밀려 points_filtered가 한 건도 안 나갔다).
        # 전용 스레드를 주면 대기 중에도 TF가 계속 들어와 실제로 기다릴 수 있다.
        # node=self로 주면 rclpy.spin(node)가 같은 노드를 자기 실행기로 뺏어가
        # 전용 스레드가 무력화되므로 반드시 None으로 준다.
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)

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
        except Exception as e:
            self.get_logger().warn(f"Could not transform {msg.header.frame_id} to {self.odom_frame}: {str(e)}")
            return

        # PointCloud2 전체를 do_transform_cloud로 재조립하지 않는다. 그 함수는
        # create_cloud()에 point_step을 넘기지 않아, 필드 뒤에 패딩이 있는
        # 클라우드에서 AssertionError를 낸다(gz gpu_lidar: 필드 합 26바이트 /
        # point_step 32 → 6바이트 패딩). 지면 분할에는 x,y,z만 필요하므로
        # 좌표만 직접 변환한다. 모듈 A·D도 같은 이유로 같은 방식을 쓴다.
        cloud_data = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)

        if len(cloud_data) == 0:
            return

        # skip_nans는 NaN만 거른다. gz gpu_lidar는 최대 사거리 밖 점을 NaN이 아닌
        # Inf로 채우므로 따로 걸러야 한다. 남겨두면 아래 정수 격자 인덱스
        # (idx_x * 1e9 + idx_y) 계산이 오버플로해 엉뚱한 셀로 뭉친다.
        cloud_data = cloud_data[np.isfinite(cloud_data).all(axis=1)]

        if len(cloud_data) == 0:
            return

        cloud_data = transform_points(cloud_data, transform.transform)
        cloud_header = Header(stamp=msg.header.stamp, frame_id=self.odom_frame)

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
            empty_msg = pc2.create_cloud_xyz32(cloud_header, [])
            self.pub.publish(empty_msg)
            return

        # Create output PointCloud2 message
        output_msg = pc2.create_cloud_xyz32(cloud_header, obstacle_points.tolist())
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
