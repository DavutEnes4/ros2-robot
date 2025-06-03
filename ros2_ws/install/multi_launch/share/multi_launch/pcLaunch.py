import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Serial Publisher (cPublisher) düğümünü başlat
        ExecuteProcess(
            cmd=['ros2', 'run', 'serial_pubsub', 'cPublisher'],
            output='screen'
        ),

        # Lidar Mapper (subscriber_lidar) düğümünü başlat
        ExecuteProcess(
            cmd=['ros2', 'run', 'lidar_mapper', 'subscriber_lidar'],
            output='screen'
        ),

        # Tello ROS2 node'u başlat
        Node(
            package='tello_ros2',
            executable='tello_node',
            name='tello_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        # Yolo Object Detection başlat
        ExecuteProcess(
            cmd=['ros2', 'run', 'tello_ros2', 'yolo_object_detection.py'],
            output='screen'
        ),
    ])
