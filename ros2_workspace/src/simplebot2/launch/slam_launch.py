from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default='/home/alexander/simplebot2/ros2_workspace/config/mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation time'),
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        )
    ])
