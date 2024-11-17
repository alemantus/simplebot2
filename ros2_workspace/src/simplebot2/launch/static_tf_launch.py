from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0104', '0', '0.099', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
    ])