# Launch file for basic lidar and odometry components
# Provides RPLidar and RF2O laser odometry for SLAM and navigation

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Include RPLidar launch
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('bearcar_nav2_launch'),'launch','rplidar.launch.py'
        )])
    )

    # Include Robot State Publisher launch (Optional - usually started by main robot launch)
    # rsp = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('bearcar_nav2_launch'),'launch','rsp.launch.py'
    #     )])
    # )

    # Include RF2O Laser Odometry launch (publish_tf=False, only publishes /odom topic)
    rf2o_laser_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rf2o_laser_odometry'),'launch','rf2o_laser_odometry.launch.py'
        )])
    )

    return LaunchDescription([
        # Basic sensor and odometry components
        rplidar,
        # rsp,  # Commented out - usually started by main robot launch
        rf2o_laser_odometry,
    ])