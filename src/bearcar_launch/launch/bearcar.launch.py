# This file is based on f1tenth_vehicle_vesc.launch.py
# Function: Drive Bearcar with remote joystick, it should localize
# itself in a known map.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Package paths
    package_name = 'bearcar_launch'
    package_dir = get_package_share_directory(package_name)
    
    # Parameter files
    rc_param_file = os.path.join(package_dir, 'params', 'remote_control.param.yaml')
    vesc_param_file = os.path.join(package_dir, 'params', 'vesc.param.yaml')

    # Core nodes for remote control system
    rc_publisher = Node(  # Remote controller signal receiver
        package='remote_control',
        namespace='vehicle',
        executable='rc_publisher',
        name='rc_publisher',
        output='screen',
        parameters=[rc_param_file],
        remappings=[
            ('cmd_vel', 'rc_cmd_vel'),  # Publish to /vehicle/rc_cmd_vel
        ]
    )

    rc_controller = Node(  # Control logic processor
        package='remote_control',
        namespace='vehicle',
        executable='rc_to_vehicle_command',
        name='rc_to_vehicle_command',
        output='screen',
        parameters=[rc_param_file],
        remappings=[
            ('cmd_vel', 'rc_cmd_vel'),           # Subscribe to /vehicle/rc_cmd_vel
            ('vehicle_cmd_vel', 'vehicle_cmd_vel'),  # Publish to /vehicle/vehicle_cmd_vel
        ]
    )

    simple_vesc_interface_node = Node(  # VESC protocol converter - replaces autoware's vesc_interface
        package='remote_control',
        executable='simple_vesc_interface',
        name='simple_vesc_interface',
        namespace='vehicle',
        output='screen',
        parameters=[{
            'speed_to_erpm_gain': 1900.0,
            'speed_to_erpm_offset': 0.0,
            'steering_angle_to_servo_gain': 0.567,    # Original parameters
            'steering_angle_to_servo_offset': 0.4875  #added offset to counter left drift sweet spot should be around 0.485 - 0.49
        }],
        remappings=[
            ('vehicle_cmd_vel', 'vehicle_cmd_vel'),  # Receive data from rc_controller
            ('commands/motor/speed', 'commands/motor/speed'),      # Publish to vehicle namespace
            ('commands/servo/position', 'commands/servo/position') # Publish to vehicle namespace
        ]
    )

    vesc_driver_node = Node(  # VESC hardware driver
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace='vehicle',
        output='screen',
        parameters=[vesc_param_file],
        remappings=[
            ('commands/motor/speed', 'commands/motor/speed'),
            ('commands/servo/position', 'commands/servo/position'),
        ]
    )

    return LaunchDescription([
        rc_publisher,
        rc_controller, 
        simple_vesc_interface_node,
        vesc_driver_node,
    ])
