from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_driver',
            executable='action_command',
            name='serial_node',
            output='screen'
        ),
        Node(
            package='serial_driver',
            executable='send_move_command',
            name='send_move_command',
            output='screen'
        ),
    ])