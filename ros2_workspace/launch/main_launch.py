from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
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

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), 'lidar_launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), 'robot_control_launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), 'localization_launch.py'))),
        #IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), 'nav2_bringup_launch.py'))),
        #IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), 'slam_launch.py'))),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(cmd_vel_mux_path)),
        IncludeLaunchDescription(XMLLaunchDescriptionSource(foxglove_bridge_launch_path))
    ])
