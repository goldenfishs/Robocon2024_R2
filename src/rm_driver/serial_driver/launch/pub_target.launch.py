from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_serial_driver',
            executable='pub_target',
            name='pub_target_node'
        )
    ])
