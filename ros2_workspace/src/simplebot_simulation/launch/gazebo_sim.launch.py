import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Setup paths to your description package
    # Change 'simplebot_description' if your package name is different
    pkg_description = get_package_share_directory('simplebot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    

    install_dir = get_package_prefix('simplebot_description')

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + install_dir + '/share'
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = install_dir + '/share'

    # Path to your xacro file
    xacro_file = os.path.join(pkg_description, 'urdf', 'simplebot_description.xacro')

    # 2. Start Gazebo Sim (The Harmonic version used in Jazzy)
    # This replaces the 'gazebo_ros' launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    # 3. Robot State Publisher 
    # Converts Xacro to URDF and publishes /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )

    # 4. Spawn the robot into the Gazebo world
    # In Jazzy/Harmonic, the executable is 'create'
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'simplebot2',
            '-z', '0.1'
        ],
        output='screen'
    )

    # 5. The Bridge (CRITICAL)
    # Gazebo Harmonic and ROS 2 Jazzy speak different languages. 
    # This bridge translates /cmd_vel and /clock so the robot can move.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge
    ])