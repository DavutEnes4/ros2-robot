from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_ros2',
            executable='tello_node',
            name='tello_node',
            output='screen',
            emulate_tty=True,
        ),
    ])