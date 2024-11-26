from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_communication',  # Replace with your package name
            executable='gps_node.py',         # Replace with your node executable name
            name='my_gps_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB1'},
                {'baudrate': 115200},
            ]
        ),
    ])
