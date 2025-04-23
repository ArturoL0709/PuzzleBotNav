from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robotics')
    
    return LaunchDescription([
        Node(
            package='mobile_robotics',
            executable='odometry_node',
            name='odometry_node',
            parameters=[os.path.join(pkg_share, 'params', 'odometry_params.yaml')]
        ),
        Node(
            package='mobile_robotics',
            executable='controller_node',
            name='controller_node',
            parameters=[os.path.join(pkg_share, 'params', 'controller_params.yaml')]
        ),
        Node(
            package='mobile_robotics',
            executable='path_generator',
            name='path_generator',
            parameters=[os.path.join(pkg_share, 'params', 'path_params.yaml')],
        ),
        
        
    ])