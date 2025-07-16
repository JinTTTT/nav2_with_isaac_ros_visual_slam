#!/usr/bin/env python3

import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    bearcar_prefix = get_package_share_directory('bearcar_launch')
    rc_param_file = os.path.join(bearcar_prefix, 'params/remote_control.param.yaml')

    # Remote Control 节点们
    rc_publisher = Node(
        package='remote_control',
        namespace='vehicle',
        executable='rc_publisher',
        name='rc_publisher',
        output='screen',
        parameters=[rc_param_file],
        remappings=[
            ('cmd_vel', 'rc_cmd_vel'),  # 发布到 /vehicle/rc_cmd_vel
        ]
    )

    rc_controller = Node(
        package='remote_control',
        namespace='vehicle',
        executable='rc_to_vehicle_command',
        name='rc_to_vehicle_command',
        output='screen',
        parameters=[rc_param_file],
        remappings=[
            ('cmd_vel', 'rc_cmd_vel'),           # 订阅 /vehicle/rc_cmd_vel
            ('vehicle_cmd_vel', 'vehicle_cmd_vel'),  # 发布到 /vehicle/vehicle_cmd_vel
        ]
    )

    simple_vesc_interface_node = Node(
        package='remote_control',
        executable='simple_vesc_interface',
        name='simple_vesc_interface',
        namespace='vehicle',
        output='screen',
        parameters=[{
            'speed_to_erpm_gain': 1900.0,
            'speed_to_erpm_offset': 0.0,
            'steering_angle_to_servo_gain': 0.567,
            'steering_angle_to_servo_offset': 0.5
        }],
        remappings=[
            ('cmd_vel', 'vehicle_cmd_vel'),  # 从rc_controller接收数据
            ('commands/motor/speed', '/commands/motor/speed'),
            ('commands/servo/position', '/commands/servo/position')
        ]
    )

    return launch.LaunchDescription([
        rc_publisher,
        rc_controller,
        simple_vesc_interface_node,
    ]) 