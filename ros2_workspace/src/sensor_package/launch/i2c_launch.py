#!/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_package',
            executable='i2c_devices.py',
            output='screen'
        )
    ])