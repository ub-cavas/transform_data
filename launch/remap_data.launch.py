from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='transform_data',
            executable='data_forward',
            name='converter',
            output="screen"
        )
    ])