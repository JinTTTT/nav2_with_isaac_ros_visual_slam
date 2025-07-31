# Launch file for AMCL localization functionality
# Uses existing map for robot localization with AMCL (Adaptive Monte Carlo Localization)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Input parameters declaration
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map_yaml_file')

    # Package directory
    bringup_dir = get_package_share_directory('articubot_one')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value='/home/jetsonagx/nav2_ws/my_map_name.yaml',
        description='Full path to map yaml file to load')

    # Map Server - loads and serves the map (mimicking your manual approach)
    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml_file,
                'use_sim_time': use_sim_time
            }])

    # AMCL - Adaptive Monte Carlo Localization (mimicking your manual approach)
    start_amcl_cmd = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'scan_topic': '/scan',
                # Basic AMCL parameters
                'max_particles': 2000,
                'min_particles': 500,
                'laser_model_type': 'likelihood_field',
                'alpha1': 0.2,
                'alpha2': 0.2,
                'alpha3': 0.2,
                'alpha4': 0.2,
                'update_min_d': 0.25,
                'update_min_a': 0.2
            }])

    # Lifecycle Manager to automatically manage both nodes
    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }])

    return LaunchDescription([
        # Declare the launch options
        declare_use_sim_time_cmd,
        declare_map_yaml_file_cmd,

        # Localization components
        start_map_server_cmd,
        start_amcl_cmd,
        start_lifecycle_manager_cmd,
    ])