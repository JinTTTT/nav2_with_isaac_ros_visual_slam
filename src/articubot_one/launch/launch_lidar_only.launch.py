# Launch file for RPLidar only
# Provides RPLidar for mapping and obstacle detection (without odometry)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Include RPLidar launch
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('articubot_one'),'launch','rplidar.launch.py'
        )])
    )

    return LaunchDescription([
        # RPLidar for mapping and obstacle detection
        rplidar,
    ])