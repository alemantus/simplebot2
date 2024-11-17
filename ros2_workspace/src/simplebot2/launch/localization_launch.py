from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_localization_file_path = os.path.join('/home/alexander/simplebot2/ros2_workspace', 'config/ekf2.yaml')
    
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[robot_localization_file_path, {'use_sim_time': False}]
        )
    ])
