import rclpy
from geodesy import utm
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from velodyne_msgs.msg import VelodyneScan
from sensor_msgs.msg import Imu, PointCloud2

class TopicRepublisher(Node):
    def __init__(self):
        super().__init__('topic_republisher')

        # Subscribers
        self.sub_fix = self.create_subscription(
            NavSatFix,
            '/novatel/oem7/fix',
            self.listener_callback_fix,
            100
        )
        self.sub_imu_raw = self.create_subscription(
            Imu,
            '/novatel/oem7/imu/data_raw',
            self.listener_callback_imu,
            100
        )
        # self.sub_odom = self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     '/novatel/oem7/odom',
        #     self.listener_callback_odom,
        #     10
        # )
        self.sub_lidar = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.listener_callback_lidar,
            100
        )
        

        self.origin_lat = 42.99263738471643
        self.origin_long = -78.79321921677531
        self.origin_alt = 142
        # Publishers
        self.pub_pose = self.create_publisher(
            PoseStamped,
            '/sensing/gnss/pose',
            100
        )
        self.pub_pose_with_cov = self.create_publisher(
            PoseWithCovarianceStamped,
            '/sensing/gnss/pose_with_covariance',
            100
        )
        self.pub_imu_data = self.create_publisher(
            Imu,
            '/sensing/imu/imu_data',
            100
        )
        self.pub_imu_raw = self.create_publisher(
            Imu,
            '/sensing/imu/tamagawa/imu_raw',
            100
        )
        self.pub_pointcloud_raw = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw',
            100
        )
        self.pub_pointcloud_raw_ex = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw_ex',
            100
        )

        self.pub_pointcloud_filtered = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/top/outlier_filtered/pointcloud',
            100
        )

        self.subscription = self.create_subscription(
            VelodyneScan,
            '/velodyne_packets',
            self.listener_callback_packets,
            100)
        
        self.publisher_packets = self.create_publisher(
            VelodyneScan,
            '/sensing/lidar/top/velodyne_packets',
            100)
        

    def conversion(self, lat, long, alt):
        ref = utm.fromLatLong(self.origin_lat, self.origin_long)
        point = utm.fromLatLong(lat, long)
        x = (point.easting - ref.easting)
        y = (point.northing - ref.northing)
        z = alt - self.origin_alt
        # print( x, y)
        # 79897.390625
        return x+79897.390625, y+62360.640625, z

    def listener_callback_fix(self, msg):
        self.get_logger().info('Received /novatel/oem7/fix')
        lat = msg.latitude
        long = msg.longitude
        alt = msg.altitude
        x, y, z = self.conversion(lat, long, alt)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # Adjust frame_id as needed
        #0 0 -0.99896 0.0144513
        # Populate the pose message with local coordinates
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z    

        pose_msg.pose.orientation.z = 1.0
        self.pub_pose.publish(pose_msg)
        
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header.stamp = self.get_clock().now().to_msg()
        pose_cov_msg.header.frame_id = 'map'
        pose_cov_msg.pose.pose.position.x = x
        pose_cov_msg.pose.pose.position.y = y
        pose_cov_msg.pose.pose.position.z = z
        pose_cov_msg.pose.pose.orientation.z = 1.0
        # Optionally set covariance (assuming 6x6 covariance matrix with diagonal values)
        pose_cov_msg.pose.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]
        
        self.pub_pose_with_cov.publish(pose_cov_msg)
        

    def listener_callback_imu(self, msg):
        self.get_logger().info('Received /novatel/oem7/imu/data_raw')
        
        msg1 =msg
        msg1.header.stamp = self.get_clock().now().to_msg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg1.header.frame_id ='base_link'
        self.pub_imu_data.publish(msg1)
        self.pub_imu_raw.publish(msg)

    # def listener_callback_odom(self, msg):
    #     self.get_logger().info('Received /novatel/oem7/odom')
    #     self.pub_pose_with_cov.publish(msg)

    def listener_callback_lidar(self, msg):
        self.get_logger().info('Received /velodyne_points')
        # print("here")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg1 = msg
        msg1.header.frame_id = "sensor_kit_base_link"
        msg.header.frame_id = 'sensor_kit_base_link'
        # self.pub_pointcloud_raw.publish(msg)
        self.pub_pointcloud_filtered.publish(msg1)
        # self.pub_pointcloud_raw_ex.publish(msg)

    def listener_callback_packets(self, msg):
        self.get_logger().info('Received Velodyne packet, republishing...')
        self.publisher_packets.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopicRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
