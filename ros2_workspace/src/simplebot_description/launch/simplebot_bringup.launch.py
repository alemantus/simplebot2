import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('simplebot_description')

    # Corrected filename as per your update
    # Note: Assumes the file is in 'urdf' folder; change to 'description' if that's where it lives
    default_model_path = os.path.join(pkg_share, 'urdf', 'simplebot_description.xacro')

    default_rviz_config = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    # Use sim time is critical for Gazebo + RViz synchronization
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Path to URDF/Xacro file'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Launch RViz'
        ),

        # 1. Robot State Publisher
        # Processes Xacro and publishes /robot_description and static TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
                'use_sim_time': use_sim_time
            }]
        ),

        # 2. Your Custom Mecanum Joint State Publisher
        # This will take your /encoder_data and publish /joint_states
        Node(
            package='simplebot_description',
            executable='mecanum_joint_state_publisher',
            name='mecanum_joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 3. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])