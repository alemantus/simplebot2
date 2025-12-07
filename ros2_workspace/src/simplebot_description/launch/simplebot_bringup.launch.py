import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch

def generate_launch_description():

    pkg_share = FindPackageShare(package='simplebot_description').find('simplebot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/simplebot_description.urdf')
    default_rviz_config = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Path to URDF file'
        ),
        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Launch RViz?'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),

        # Your mecanum joint state publisher
        Node(
            package='simplebot_description',
            executable='mecanum_joint_state_publisher',
            name='mecanum_joint_state_publisher',
            output='screen'
        ),

        # RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz_config],
            condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
        )
    ])
