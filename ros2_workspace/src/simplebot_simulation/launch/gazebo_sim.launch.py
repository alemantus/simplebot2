import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('simplebot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    install_dir = get_package_prefix('simplebot_description')

    # Fix Gazebo Resource Path
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + install_dir + '/share'
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = install_dir + '/share'

    xacro_file = os.path.join(pkg_description, 'urdf', 'simplebot_description.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'urdf_config.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # 1. Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
        ),

        # 2. Robot State Publisher (Crucial: Keep URDF available)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file]),
                'use_sim_time': use_sim_time
            }]
        ),

        # 3. Spawn Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'simplebot2', '-z', '5.0'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # 4. The Bridge (Directional fixed for Jazzy)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/encoder_data_gz@sensor_msgs/msg/JointState[gz.msgs.Model'
            ],
            output='screen'
        ),

        # 5. Your Custom Mecanum Node
        Node(
            package='simplebot_description',
            executable='mecanum_joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # 6. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])