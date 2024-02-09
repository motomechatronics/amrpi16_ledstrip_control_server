
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amrpi16_ledstrip_control_server',
            executable='amrpi16_ledstrip_control_server_node',
            output='screen'),
    ])
