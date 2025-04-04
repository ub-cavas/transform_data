import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import pcl
import os

class PointCloudAccumulator(Node):
    def __init__(self):
        super().__init__('pointcloud_accumulator')

        # Create an empty list to store all point cloud data
        self.all_points = []

        # Create a subscription to the /cloud_registered topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.point_cloud_callback,
            10
        )

        # Set up a timer to periodically save the accumulated point clouds
        self.timer = self.create_timer(1.0, self.save_all_points_to_pcd)  # Save every second

    def point_cloud_callback(self, msg):
        # Convert ROS PointCloud2 message to numpy array
        pc_data = pc2.read_points_numpy(msg)
        pc_data = np.array(pc_data.tolist(), dtype=np.float32)
        
        # Append new data to the list of all points
        self.all_points.append(pc_data)
        self.get_logger().info(f"Received point cloud data, current accumulation size: {len(self.all_points)}")

    def save_all_points_to_pcd(self):
        # Combine all accumulated point clouds into one array
        if self.all_points:
            combined_points = np.vstack(self.all_points)
            cloud = pcl.PointCloud()
            cloud.from_array(combined_points)
            
            # Set the output directory and filename
            output_directory = '/path/to/output_directory'
            filename = os.path.join(output_directory, 'accumulated_scan.pcd')
            
            # Create the output directory if it doesn't exist
            if not os.path.exists(output_directory):
                os.makedirs(output_directory)
            
            # Save the combined point cloud to a PCD file
            pcl.save(cloud, filename)
            self.get_logger().info(f"Saved accumulated point cloud to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
