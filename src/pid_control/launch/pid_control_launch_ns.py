# Imports required to set the package address (directories)
import os
from ament_index_python.packages import get_package_share_directory

# Imports required for calling other launch files (nesting)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Imports required to push a namespace (append) a namespace to a nested launch file
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

# Describe launch file
def generate_launch_description():
    package = 'pid_control'  # Package to be launched
    launch_file = 'pid_control_launch.py'  # Launch file to get a namespace
    group1_ns = 'group1'  # Namespaces to be used
    group2_ns = 'group2'
    group3_ns = 'group3'
    
    # Get the address of the package and launch file
    package_directory = get_package_share_directory(package)
    launch_file_path = os.path.join(package_directory, 'launch', launch_file)
    
    # Set the launch file source for groups
    launch_source1 = PythonLaunchDescriptionSource(launch_file_path)
    launch_source2 = PythonLaunchDescriptionSource(launch_file_path)
    launch_source3 = PythonLaunchDescriptionSource(launch_file_path)
    
    # Include the launch description for groups
    motor_control_launch_1 = IncludeLaunchDescription(launch_source1)
    motor_control_launch_2 = IncludeLaunchDescription(launch_source2)
    motor_control_launch_3 = IncludeLaunchDescription(launch_source3)
    
    # Set namespaces for groups
    motor_control_group1 = GroupAction(
        actions=[
            PushRosNamespace(group1_ns),
            motor_control_launch_1,
        ]
    )
    
    motor_control_group2 = GroupAction(
        actions=[
            PushRosNamespace(group2_ns),
            motor_control_launch_2,
        ]
    )
    
    motor_control_group3 = GroupAction(
        actions=[
            PushRosNamespace(group3_ns),
            motor_control_launch_3,
        ]
    )
    
    # Launch
    ld = LaunchDescription([
        motor_control_group1,
        motor_control_group2,
        motor_control_group3
    ])
    
    return ld