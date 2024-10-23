from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
import time 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource  # Corrected import for XML


def generate_launch_description():
    # LiDAR parameters
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    robot_localization_file_path = os.path.join('/home/alexander/simplebot2/ros2_workspace', 'config/ekf.yaml') 

    # SLAM parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default='/home/alexander/simplebot2/ros2_workspace/src/mapper_params_online_async.yaml'
    )


    # Path to the second launch file (same directory as current file)
    cmd_vel_mux_path = os.path.join(
        get_package_share_directory('cmd_vel_mux'),
        'launch',  
        'cmd_vel_mux-launch.py'
    )

    online_async_launch_path = os.path.join(
        os.path.dirname(__file__),  # Get the directory of the current launch file
        'online_async_launch.py'
    )

    # Path to the foxglove_bridge XML launch file
    foxglove_bridge_launch_path = os.path.join(
        get_package_share_directory('rosbridge_server'),  # Assuming foxglove_bridge is installed and findable
        'launch',
        'rosbridge_websocket_launch.xml'
    )
    # Lifecycle node parameters
    # use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager', default='true')

    return LaunchDescription([
        # LiDAR configuration arguments
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        # Start odom Node
        Node(
            package='motor_controller2',
            executable='odom.py',
            name='odom_node',
            output='screen'
        ),

        # Start joy_node with device_id parameter set to 0
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device_id': 0}],
            output='screen'
        ),

        Node(
            package='sensor_package',
            executable='lsm6dsm.py',
            name='imu',
            output='screen'
        ),


        # Start motor_controller
        Node(
            package='motor_controller2',
            executable='kinematics2serial.py',
            name='motor_controller',
            output='screen'
        ),

        # Start joy2vel Node
        Node(
            package='motor_controller2',
            executable='joy2vel',
            name='joy2vel',
            output='screen'
        ),

        # Start LiDAR Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate, 
                         'scan_mode': scan_mode,
                         'sample_rate': 1}],
            output='screen'),

          # Start robot localization using an Extended Kalman filter
         #Node(
         #   package='robot_localization',
         #   executable='ekf_node',
         #   name='ekf_filter_node',
         #   output='screen',
         #   parameters=[robot_localization_file_path, 
         #   {'use_sim_time': use_sim_time}]),
        # Start SLAM Toolbox Node
        #Node(
        #    parameters=[
        #      slam_params_file:='/home/alexander/simplebot2/ros2_workspace/src/mapper_params_online_async.yaml'
        #    ],
        #    package='slam_toolbox',
        #    executable='async_slam_toolbox_node',
        #    name='slam_toolbox',
        #    output='screen',
        #    namespace=''
        #),

        # Start static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0104', '0', '0.099', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # Include another launch file (online_async_launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(online_async_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cmd_vel_mux_path)
        ),
        
        # Include the foxglove_bridge XML launch file
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(foxglove_bridge_launch_path)
        ),
        
        # Start SLAM Toolbox Node with 5 second delay
        #TimerAction(
        #    period=10.0,  # 5 seconds delay
        #    actions=[
        #        Node(
        #            package='slam_toolbox',
        #            executable='async_slam_toolbox_node',
        #            name='slam_toolbox',
        #            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        #            output='screen'
        #        )
        #    ]
        #),
    ])