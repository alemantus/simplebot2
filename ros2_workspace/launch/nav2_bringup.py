from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # map_dir = LaunchConfiguration(
    #     'map',
    #     default=os.path.join(
    #         get_package_share_directory('your_package_name'),
    #         'maps',
    #         'your_map.yaml'))
    map_dir = os.path.join('/home/alexander/simplebot2/ros2_workspace', 'config/map.yaml') 
    
    #param_dir = LaunchConfiguration(
    #    'params_file',
    #    default=os.path.join(
    #        get_package_share_directory('your_package_name'),
    #        'config',
    #        'nav2_params.yaml'))
    param_dir = os.path.join('/home/alexander/simplebot2/ros2_workspace', 'config/nav2_params.yaml') 

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map yaml file to load'),
            
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),
            
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                       {'yaml_filename': map_dir}]),
                       
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                       {'autostart': True},
                       {'node_names': ['map_server',
                                     'amcl',
                                     'controller_server',
                                     'planner_server',
                                     'bt_navigator']}]),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[param_dir]),
            
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[param_dir]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[param_dir]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[param_dir]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[param_dir]),
    ])