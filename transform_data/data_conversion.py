import rclpy
import struct
import numpy as np
from geodesy import utm
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from velodyne_msgs.msg import VelodyneScan
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

class TopicRepublisher(Node):
    def __init__(self):
        super().__init__('topic_republisher')
        # print("Hello")
        # Subscribers
        self.sub_fix = self.create_subscription(
            NavSatFix,
            '/novatel/oem7/fix',
            self.listener_callback_fix,
            10
        )
        self.sub_imu_raw = self.create_subscription(
            Imu,
            '/novatel/oem7/imu/data_raw',
            self.listener_callback_imu,
            10
        )
        # self.sub_odom = self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     '/novatel/oem7/odom',
        #     self.listener_callback_odom,
        #     10
        # )

        self.subscriber_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
)
        self.sub_lidar = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.listener_callback_lidar,
            self.subscriber_qos
        )
        

        # self.origin_lat = 42.99263738471643
        # self.origin_long = -78.79321921677531
        # self.origin_alt = 142
        # Publishers
        self.pub_pose = self.create_publisher(
            NavSatFix,
            '/sensing/gnss/ublox/nav_sat_fix',
            10
        )
        # self.pub_pose = self.create_publisher(
        #     PoseStamped,
        #     '/sensing/gnss/pose',
        #     10
        # )
        # self.pub_pose_with_cov = self.create_publisher(
        #     PoseWithCovarianceStamped,
        #     '/sensing/gnss/pose_with_covariance',
        #     10
        # )
        # self.pub_imu_data = self.create_publisher(
        #     Imu,
        #     '/sensing/imu/imu_data',
        #     10
        # )
        self.pub_imu_raw = self.create_publisher(
            Imu,
            '/sensing/imu/tamagawa/imu_raw',
            10
        )
        # self.pub_pointcloud_raw = self.create_publisher(
        #     PointCloud2,
        #     '/sensing/lidar/top/pointcloud_raw',
        #     10
        # )
        self.pub_pointcloud_raw_ex = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw_ex',
            10
        )

        # self.pub_pointcloud_filtered = self.create_publisher(
        #     PointCloud2,
        #     '/sensing/lidar/top/outlier_filtered/pointcloud',
        #     10
        # )

        # self.subscription = self.create_subscription(
        #     VelodyneScan,
        #     '/velodyne_packets',
        #     self.listener_callback_packets,
        #     10)
        
        # self.publisher_packets = self.create_publisher(
        #     VelodyneScan,
        #     '/sensing/lidar/top/velodyne_packets',
        #     10)
        # print("bye")

    # def conversion(self, lat, long, alt):
    #     ref = utm.fromLatLong(self.origin_lat, self.origin_long)
    #     point = utm.fromLatLong(lat, long)
    #     x = (point.easting - ref.easting)
    #     y = (point.northing - ref.northing)
    #     z = alt - self.origin_alt
    #     # print( x, y)
    #     # 79897.390625
    #     return x+79897.390625, y+62360.640625, z

    def listener_callback_fix(self, msg):
        # self.get_logger().info('Received /novatel/oem7/fix')
        # lat = msg.latitude
        # long = msg.longitude
        # alt = msg.altitude
        # x, y, z = self.conversion(lat, long, alt)
        pose_msg = msg
        # pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'gnss_link'  # Adjust frame_id as needed
        #0 0 -0.99896 0.0144513
        # Populate the pose message with local coordinates
        # pose_msg.pose.position.x = x
        # pose_msg.pose.position.y = y
        # pose_msg.pose.position.z = z    

        # pose_msg.pose.orientation.z = 1.0
        # self.pub_pose.publish(pose_msg)
        
        # pose_cov_msg = PoseWithCovarianceStamped()
        # pose_cov_msg.header.stamp = self.get_clock().now().to_msg()
        # pose_cov_msg.header.frame_id = 'map'
        # pose_cov_msg.pose.pose.position.x = x
        # pose_cov_msg.pose.pose.position.y = y
        # pose_cov_msg.pose.pose.position.z = z
        # pose_cov_msg.pose.pose.orientation.z = 1.0
        # # Optionally set covariance (assuming 6x6 covariance matrix with diagonal values)
        # pose_cov_msg.pose.covariance = [
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        # ]
        
        self.pub_pose.publish(pose_msg)
        

    def listener_callback_imu(self, msg):
        # self.get_logger().info('Received /novatel/oem7/imu/data_raw')
        
        imu_msg = msg
        #msg1.header.stamp = 0
        #msg.header.stamp.sec = 0
        #msg.header.stamp.nanosec = 0
        imu_msg.header.frame_id ='tamagawa/imu_link'
        # self.pub_imu_data.publish(msg1)
        self.pub_imu_raw.publish(msg)


    # def listener_callback_odom(self, msg):
    #     self.get_logger().info('Received /novatel/oem7/odom')
    #     self.pub_pose_with_cov.publish(msg)

    def listener_callback_lidar(self, msg):
        # self.get_logger().info('Received /velodyne_points')
        # print("here")
        # msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'velodyne_top'
        # msg.header.frame_id = 'sensor_kit_base_link'
        # self.pub_pointcloud_raw.publish(msg)
        # self.pub_pointcloud_filtered.publish(msg)
        self.pub_pointcloud_raw_ex.publish(msg)

    # def listener_callback_lidar(self, msg):
    #     # self.get_logger().info('Received /velodyne_points')

    #     # Update header timestamp and frame_id
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.header.frame_id = "sensor_kit_base_link"

    #     # Extract points from PointCloud2
    #     points = []
    #     for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
    #         points.append([point[0], point[1], point[2]])

    #     points = np.array(points)

    #     # Filter out points within 1.5 meters
    #     distances = np.linalg.norm(points, axis=1)  # Compute distance from origin
    #     filtered_points = points[distances >= 3.0]  # Keep points beyond 1.5 meters

    #     # Create a new PointCloud2 message with filtered points
    #     filtered_msg = pc2.create_cloud_xyz32(msg.header, filtered_points)

    #     # Publish the filtered point cloud
    #     self.pub_pointcloud_filtered.publish(filtered_msg)

    # # def listener_callback_packets(self, msg):
    # #     # self.get_logger().info('Received Velodyne packet, republishing...')
    # #     # self.publisher_packets.publish(msg)


def main(args=None):
    # print("Here")
    rclpy.init(args=args)
    node = TopicRepublisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
