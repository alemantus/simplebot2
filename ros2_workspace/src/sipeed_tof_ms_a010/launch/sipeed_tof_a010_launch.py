#!/home/alexander/venv/bin/python

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sipeed_tof_ms_a010',
            executable='publisher',
            name='tof_publisher',
            output='screen',
            parameters=[{'device': '/dev/ttyUSB1'}]
        )
    ])
