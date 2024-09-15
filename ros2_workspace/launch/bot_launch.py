from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # LiDAR parameters
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    # SLAM parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(get_package_share_directory("slam_toolbox"),
                             'config', 'mapper_params_online_async.yaml')
    )

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
        
        # Start SLAM Toolbox Node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0104', '0', '0.099', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate, 
                         'scan_mode': scan_mode,
                         'sample_rate': 1}],
            output='screen'),


    
        # Add static transform publisher
        #Node(
        #    package='sensor_package',
        #    executable='lsm6dsm.py',
        #    name='imu_node',
        #    output='screen'
        #),

        #Node(
        #   package='robot_localization',
        #   executable='ekf_node',
        #   name='ekf_filter_node',
        #   output='screen',
        #   parameters=['/home/alexander/simplebot2/ros2_workspace/src/ekf.yaml']
        #)
    ])
