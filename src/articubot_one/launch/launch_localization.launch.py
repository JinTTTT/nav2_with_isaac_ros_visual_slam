# Launch file for SLAM Toolbox localization functionality
# Uses existing map for robot localization

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Package directory
    bringup_dir = get_package_share_directory('articubot_one')

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'localization_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for localization')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Include SLAM Toolbox in localization mode
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('articubot_one'),'launch','online_async_launch.py'
        )]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    return LaunchDescription([
        # Declare the launch options
        declare_params_file_cmd,
        declare_use_sim_time_cmd,

        # Localization component
        slam_localization,
    ])