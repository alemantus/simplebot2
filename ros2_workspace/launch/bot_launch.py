from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start joy_node with device_id parameter set to 0
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device_id': 0}],
            output='screen'
        ),

        # Start motor_controller
        Node(
            package='motor_controller',
            executable='kinematics2serial',
            name='motor_controller',
            output='screen'
        )
    ])
