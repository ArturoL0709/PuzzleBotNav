from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_node = Node(
        package='motor_control',
        executable='dc_motor',
        name='motor_sys',
        parameters=[{
            'sample_time': 0.01,
            'sys_gain_K': 2.16,
            'sys_tau_T': 0.05,
            'initial_conditions': 0.0,
        }],
        output='screen'
    )

    sp_node = Node(
        package='motor_control',
        executable='set_point',
        name='sp_gen',
        output='screen'
    )

    ctrl_node = Node(
        package='motor_control',
        executable='controller',
        name='ctrl',
        parameters=[{
            'Kp': 1.0,
            'Ki': 0.0,
            'Kd': 0.0
        }],
        output='screen'
    )

    return LaunchDescription([motor_node, sp_node, ctrl_node])
