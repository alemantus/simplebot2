from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    # Declare the map argument
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('nav2_bringup'),
            'maps',
            'apartment_map/apartment.yaml'  # Replace with your map file name
        ),
        description='Full path to the map file to load'
    )

    # Paths to external packages
    cmd_vel_mux_path = os.path.join(
        get_package_share_directory('cmd_vel_mux'),
        'launch',
        'cmd_vel_mux-launch.py'
    )

    foxglove_bridge_launch_path = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    )

    # Paths to launch files within this package
    lidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'lidar_launch.py'
    )
    robot_control_launch_path = os.path.join(
        get_package_share_directory('motor_controller2'),
        'launch',
        'robot_control_launch.py'
    )
    localization_launch_path = os.path.join(
        get_package_share_directory('simplebot2'),
        'launch',
        'localization_launch.py'
    )
    joy_launch_path = os.path.join(
        get_package_share_directory('simplebot2'),
        'launch',
        'joy_launch.py'
    )

    static_tf_path = os.path.join(
        get_package_share_directory('simplebot2'),
        'launch',
        'static_tf_launch.py'
    )

    camera_launch_path = os.path.join(
        get_package_share_directory('camera_package'),
        'launch',
        'camera_launch.py'
    )
    
    
    slam_launch_path = os.path.join(
        get_package_share_directory('simplebot2'),
        'launch',
        'online_async_launch.py'
    )

    return LaunchDescription([
        # Declare map argument
        map_arg,

        # Include local package launch files
        IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_control_launch_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(localization_launch_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(joy_launch_path)),

        # Include external package launch files
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cmd_vel_mux_path)),
        IncludeLaunchDescription(XMLLaunchDescriptionSource(foxglove_bridge_launch_path)),
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch_path)),

        # Include the slam node
        IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch_path)),
    ])
