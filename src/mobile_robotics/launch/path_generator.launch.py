from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_robotics',
            executable='path_generator',
            name='path_generator',
            output='screen',
        )
    ])
