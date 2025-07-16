from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


import os


def generate_launch_description():   
    bearcar_prefix = get_package_share_directory('bearcar_launch')
    nav2_prefix = get_package_share_directory('nav2_bringup')

    nav2_param_file = os.path.join(
        bearcar_prefix, 'params/nav2.params.yaml')

    bearcar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bearcar_prefix,
                         'launch/bearcar.launch.py'),
        ),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bearcar_prefix,
                         'launch/localization_slam.launch.py'),
        ),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_prefix,
                         'launch/navigation_launch.py'),
        ),
        launch_arguments={
            'params_file': nav2_param_file,
        }.items()
    )

    return LaunchDescription([
        bearcar_launch,
        slam_launch,
        nav2_launch,
    ])
