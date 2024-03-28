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
        ),

        # Start odom node
        Node(
            package='motor_controller2',
            executable='odom.py',
            name='odom_node',
            output='screen'
        ),

        # Add static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0104', '0', '0.099', '0', '0', '0', '1', 'base_link', 'laser'],
            output='screen'
        ),

        # Add pointcloud_to_laserscan node with angle filtering
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            parameters=[{'target_frame': 'base_link', 'angle_min': -0.5, 'angle_max': 0.5}],
            remappings=[('cloud_in', 'scan')],
            output='screen'
        )
    ])
