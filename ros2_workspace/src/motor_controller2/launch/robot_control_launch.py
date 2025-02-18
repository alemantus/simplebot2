from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_controller2',
            executable='odom.py',
            name='odom_node',
            output='screen'
        ),
        Node(
            package='motor_controller2',
            executable='kinematics2serial.py',
            name='motor_controller',
            output='screen'
        ),
        Node(
            package='motor_controller2',
            executable='joy2vel',
            name='joy2vel',
            output='screen'
        )
    ])
