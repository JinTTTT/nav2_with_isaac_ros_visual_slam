import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',  # 从Sensitivity改为Standard，降低频率
                'scan_frequency': 8.0     # 明确设置8Hz而不是默认10Hz
            }]
        )
    ])
