import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    return LaunchDescription([
        # LIDAR Launch Arguments
        DeclareLaunchArgument('channel_type', default_value=channel_type, description='Lidar channel type'),
        DeclareLaunchArgument('serial_port', default_value=serial_port, description='Lidar serial port'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate, description='Lidar baudrate'),
        DeclareLaunchArgument('frame_id', default_value=frame_id, description='Lidar frame ID'),
        DeclareLaunchArgument('inverted', default_value=inverted, description='Invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate, description='Enable angle compensation'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode, description='Lidar scan mode'),

        # LIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate
            }],
            output='screen'
        ),
        
        # Keyboard Subscriber Node
        Node(
            package='serial_pubsub',
            executable='kSubscriber',
            name='keyboard_subscriber_node',
            output='screen'
        ),
        
        # Camera Publisher Node
        Node(
            package='ros2_opencv',
            executable='publisher_node',
            name='camera_publisher_node',
            output='screen'
        )
    ])
