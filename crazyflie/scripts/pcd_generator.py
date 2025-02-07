import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import struct
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__("pointcloud_saver")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10  # Default depth value
        )

        self.subscription = self.create_subscription(
            PointCloud2, "/cf231/pointcloud", self.callback, qos_profile
        )

    def callback(self, msg):
        # Convert ROS2 PointCloud2 to numpy array
        cloud_data = np.array(list(self.read_points(msg, field_names=("x", "y", "z"))))

        # Convert to Open3D PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud_data)

        # Save as PCD or PLY
        o3d.io.write_point_cloud("output.pcd", pcd)
        o3d.io.write_point_cloud("output.ply", pcd)
        self.get_logger().info("Point cloud saved as output.pcd and output.ply")

    def read_points(self, cloud, field_names=None):
        """Helper function to convert PointCloud2 to numpy array"""
        fmt = "<fff"  # 'x', 'y', 'z' float32
        for i in range(cloud.height * cloud.width):
            yield struct.unpack_from(fmt, cloud.data, i * cloud.point_step)

rclpy.init()
node = PointCloudSaver()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
