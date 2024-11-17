from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_package',  # Replace with the actual package name
            executable='camera_publisher',  # Replace with the actual executable name
            name='camera_publisher',  # Optional: Rename the node if needed
            output='screen',  # Prints the log to the terminal
            parameters=[{
                # Add parameters here if needed
                # 'param_name': param_value,
            }]
        )
    ])
