from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='signal_processing',
            executable='signal_generator',
            name='signal_generator'
        ),
        Node(
            package='signal_processing',
            executable='signal_processor',
            name='signal_processor'
        ),
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            arguments=['/signal/data', '/proc_signal/data']
        )
    ])
