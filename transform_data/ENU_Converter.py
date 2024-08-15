import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import pymap3d as pm

class GNSSConverter(Node):
    def __init__(self):
        super().__init__('gnss_converter')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/novatel/oem7/fix',
            self.listener_callback,
            10)
        # self.publisher_ = self.create_publisher(PoseStamped, '/sensing/gnss/pose', 10)
        # self.reference_lat = 42.992637860893  
        # self.reference_lon = -78.79334000189796  
        # self.reference_alt = 142.0336737120524 
        self.long = []
        self.lat = []
        self.local_x = []
        self.local_y = []
        self.x = 1000
        self.y = 1000
        self.ref_x = 42.99263738482443
        self.ref_y = -78.79321921644043

        plt.figure(figsize=(12, 6))

        plt.subplot(1, 2, 1)
        plt.scatter(self.long, self.lat, color='blue')
        plt.title('Latitude vs Longitude')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')

        # Plotting Local X vs Local Y
        plt.subplot(1, 2, 2)
        plt.scatter(self.local_x, self.local_y, color='red')
        plt.title('Local X vs Local Y')
        plt.xlabel('Local X')
        plt.ylabel('Local Y')

        plt.tight_layout()
        plt.show()


    def listener_callback(self, msg):
        # enu_x, enu_y, enu_z = pm.geodetic2enu(
        #     msg.longitude,
        #     msg.latitude,
        #     msg.altitude,
        #     self.reference_lon,
        #     self.reference_lat,
        #     self.reference_alt
        # )
        
        self.local_x.append(self.x + (msg.longitude-self.ref_x))
        self.local_y.append(self.y + (msg.latitude-self.ref_y))

        self.long.append(msg.longitude)
        self.lat.append(msg.latitude)
        print(f"local_x:{(msg.longitude-self.ref_x)}, local_y:{(msg.latitude-self.ref_y)}")
        # self
        # pose = PoseStamped()
        # pose.header.stamp = msg.header.stamp
        # pose.header.frame_id = "map"
        # pose.pose.position.x = enu_x
        # pose.pose.position.y = enu_y
        # pose.pose.position.z = enu_z
        # pose.pose.orientation.x = 0.0
        # pose.pose.orientation.y = 0.0
        # pose.pose.orientation.z = 0.0
        # pose.pose.orientation.w = 0.0
        plt.figure(figsize=(12, 6))

        plt.subplot(1, 2, 1)
        plt.scatter(self.long, self.lat, color='blue')
        plt.title('Latitude vs Longitude')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')

        # Plotting Local X vs Local Y
        plt.subplot(1, 2, 2)
        plt.scatter(self.local_x, self.local_y, color='red')
        plt.title('Local X vs Local Y')
        plt.xlabel('Local X')
        plt.ylabel('Local Y')

        plt.tight_layout()
        plt.show()
        # self.publisher_.publish(pose)
        # self.get_logger().info(f'Published ENU coordinates: x={enu_x}, y={enu_y}, z={enu_z}')

def main(args=None):
    rclpy.init(args=args)
    gnss_converter = GNSSConverter()
    rclpy.spin(gnss_converter)
    gnss_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
