import rclpy
from rclpy.node import Node
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Header
import geometry_msgs.msg

class TransformBroadcaster(Node):
    def __init__(self):
        super().__init__('transform_broadcaster')

        # Create a tf2 broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)

        # Define the transforms
        self.create_timer(0.1, self.publish_transforms)
        
        self.velodyne_subscriber = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.velodyne_callback,
            100
        )

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/novatel/oem7/imu/data_raw',
            self.imu_callback,
            100
        )

        # Create publishers
        self.new_points_publisher = self.create_publisher(
            PointCloud2,
            '/velodyne_points',
            100
        )

        self.imu_data_publisher = self.create_publisher(
            Imu,
            '/imu/data',
            100
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/novatel/oem7/odom',
            self.odom_callback,
            100
        )

        self.fix_subscriber = self.create_subscription(
            NavSatFix,
            '/novatel/oem7/fix',
            self.fix_callback,
            100
        )

        # self.odom_publisher = self.create_publisher(Odometry, '/novatel/odom', 10)
        # self.fix_publisher = self.create_publisher(NavSatFix, '/novatel/fix', 10)



    def publish_transforms(self):
        # Define and broadcast the transform from odom to base_link
        t_odom_base_link = geometry_msgs.msg.TransformStamped()
        t_odom_base_link.header.stamp = self.get_clock().now().to_msg()
        t_odom_base_link.header.frame_id = 'odom'
        t_odom_base_link.child_frame_id = 'base_link'
        t_odom_base_link.transform.translation.x = 0.0
        t_odom_base_link.transform.translation.y = 0.0
        t_odom_base_link.transform.translation.z = 0.0
        t_odom_base_link.transform.rotation.x = 0.0
        t_odom_base_link.transform.rotation.y = 0.0
        t_odom_base_link.transform.rotation.z = 0.0
        t_odom_base_link.transform.rotation.w = 1.0
        self.br.sendTransform(t_odom_base_link)

        # Define and broadcast the transform from base_link to camera_init
        t_base_link_camera_init = geometry_msgs.msg.TransformStamped()
        t_base_link_camera_init.header.stamp = self.get_clock().now().to_msg()
        t_base_link_camera_init.header.frame_id = 'base_link'
        t_base_link_camera_init.child_frame_id = 'camera_init'
        t_base_link_camera_init.transform.translation.x = 1.82
        t_base_link_camera_init.transform.translation.y = 0.02
        t_base_link_camera_init.transform.translation.z = 0.91  # Example value
        t_base_link_camera_init.transform.rotation.x = 0.0
        t_base_link_camera_init.transform.rotation.y = 0.0
        t_base_link_camera_init.transform.rotation.z = 0.0
        t_base_link_camera_init.transform.rotation.w = 1.0
        self.br.sendTransform(t_base_link_camera_init)

        # Define and broadcast the transform from base_link to velodyne_base_link
        t_base_link_velodyne_base_link = geometry_msgs.msg.TransformStamped()
        t_base_link_velodyne_base_link.header.stamp = self.get_clock().now().to_msg()
        t_base_link_velodyne_base_link.header.frame_id = 'base_link'
        t_base_link_velodyne_base_link.child_frame_id = 'velodyne_base_link'
        t_base_link_velodyne_base_link.transform.translation.x = 1.14
        t_base_link_velodyne_base_link.transform.translation.y = 0.0
        t_base_link_velodyne_base_link.transform.translation.z = 1.81
        t_base_link_velodyne_base_link.transform.rotation.x = 0.0
        t_base_link_velodyne_base_link.transform.rotation.y = 0.0
        t_base_link_velodyne_base_link.transform.rotation.z = 0.0
        t_base_link_velodyne_base_link.transform.rotation.w = 1.0
        self.br.sendTransform(t_base_link_velodyne_base_link)

        # Define and broadcast the transform from velodyne_base_link to velodyne
        t_velodyne_base_link_velodyne = geometry_msgs.msg.TransformStamped()
        t_velodyne_base_link_velodyne.header.stamp = self.get_clock().now().to_msg()
        t_velodyne_base_link_velodyne.header.frame_id = 'velodyne_base_link'
        t_velodyne_base_link_velodyne.child_frame_id = 'velodyne'
        t_velodyne_base_link_velodyne.transform.translation.x = 0.0
        t_velodyne_base_link_velodyne.transform.translation.y = 0.0
        t_velodyne_base_link_velodyne.transform.translation.z = 0.0
        t_velodyne_base_link_velodyne.transform.rotation.x = 0.0
        t_velodyne_base_link_velodyne.transform.rotation.y = 0.0
        t_velodyne_base_link_velodyne.transform.rotation.z = 0.0
        t_velodyne_base_link_velodyne.transform.rotation.w = 1.0
        self.br.sendTransform(t_velodyne_base_link_velodyne)

        # Define and broadcast the transform from base_link to imu_link
        t_base_link_imu_link = geometry_msgs.msg.TransformStamped()
        t_base_link_imu_link.header.stamp = self.get_clock().now().to_msg()
        t_base_link_imu_link.header.frame_id = 'base_link'
        t_base_link_imu_link.child_frame_id = 'imu_link'
        t_base_link_imu_link.transform.translation.x = 0.49
        t_base_link_imu_link.transform.translation.y = 0.52
        t_base_link_imu_link.transform.translation.z = 1.36
        t_base_link_imu_link.transform.rotation.x = 0.0
        t_base_link_imu_link.transform.rotation.y = 0.0
        t_base_link_imu_link.transform.rotation.z = 0.0
        t_base_link_imu_link.transform.rotation.w = 1.0
        self.br.sendTransform(t_base_link_imu_link)

    def velodyne_callback(self, msg):
        self.get_logger().info('Received /velodyne_points')
        # Process the PointCloud2 message and publish it on /new_points
        # new_msg = PointCloud2()
        # new_msg.header = Header()
        # new_msg.header.stamp = self.get_clock().now().to_msg()
        # new_msg.header.frame_id = 'velodyne'
        # new_msg.height = msg.height
        # new_msg.width = msg.width
        # new_msg.fields = msg.fields
        # new_msg.is_bigendian = msg.is_bigendian
        # new_msg.point_step = msg.point_step
        # new_msg.row_step = msg.row_step
        # new_msg.data = msg.data
        # new_msg.is_dense = msg.is_dense
        self.new_points_publisher.publish(msg)

    def imu_callback(self, msg):
        self.get_logger().info('Received /novatel/oem7/imu/data_raw')
        # Process the Imu message and publish it on /imu/data
        # new_msg = Imu()
        # new_msg.header = Header()
        # new_msg.header.stamp = self.get_clock().now().to_msg()
        # new_msg.header.frame_id = 'imu_link'
        # new_msg.orientation = msg.orientation
        # new_msg.orientation_covariance = msg.orientation_covariance
        # new_msg.angular_velocity = msg.angular_velocity
        # new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        # new_msg.linear_acceleration = msg.linear_acceleration
        # new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.imu_data_publisher.publish(msg)

    def odom_callback(self, msg: Odometry):
        self.get_logger().info('Publishing Odometry data')
        self.odom_publisher.publish(msg)

    def fix_callback(self, msg: NavSatFix):
        self.get_logger().info('Publishing NavSatFix data')
        self.fix_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TransformBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()