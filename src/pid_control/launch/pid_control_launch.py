import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_directory = get_package_share_directory('pid_control')
    
    # Params file
    config = os.path.join(
        package_directory,
        'config',
        'params.yaml'
    )
    
    # Set Point Node
    set_point_node = Node(
        name="sp_gen",
        package='pid_control',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        parameters=[config]
    )
    
    # Controller Node
    controller_node = Node(
        name="ctrl_signal",
        package='pid_control',
        executable='controller',
        emulate_tty=True,
        output='screen',
        parameters=[config]
    )
    
    # Motor System Node (for simulation)
    motor_node = Node(
        name="motor_sys",
        package='pid_control',
        executable='dc_motor',
        emulate_tty=True,
        output='screen',
        parameters=[config]
    )
    
    # Launch Description
    ld = LaunchDescription([
        set_point_node,
        controller_node,
        motor_node
    ])
    
    return ld