# Map the environment using RPLiDAR and SLAM Toolbox

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    
    # Package paths
    bearcar_prefix = get_package_share_directory('bearcar_launch')
    slam_toolbox_pkg_prefix = get_package_share_directory('slam_toolbox')
    
    # Parameter files
    mapping_param_file = os.path.join(
        bearcar_prefix, "params", "mapper_params_online_async.param.yaml"
    )
    rplidar_param_file = os.path.join(
        bearcar_prefix, 'params', 'rplidar.param.yaml')
    laser_filter_param_file = os.path.join(
        bearcar_prefix, "params", "laser_filter.param.yaml")
    
    # URDF file
    bearcar_urdf = os.path.join(
        bearcar_prefix, 'config', 'urdf', 'bearcar.urdf')
    with open(bearcar_urdf, 'r') as infp:
        bearcar_urdf_file = infp.read()
    
    # RViz config
    rviz_config_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'rviz',
        'rplidar_ros.rviz'
    )
    
    # Launch arguments
    mapping_param = DeclareLaunchArgument(
        "mapping_param_file",
        default_value=mapping_param_file,
        description="Path to config file for mapping nodes",
    )
    
    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='false',
        description='Launch RViz2'
    )
    
    # Essential nodes for SLAM
    
    # Robot State Publisher
    robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='vehicle',
        output='screen',
        parameters=[{'robot_description': bearcar_urdf_file}],
    )
    
    # RPLiDAR Node (from bearcar.launch.py)
    rplidar_node = Node(
        package='rplidar_ros2',
        executable='rplidar_scan_publisher',
        name='rplidar_scan_publisher',
        namespace='lidar',
        parameters=[rplidar_param_file],
        output='screen'
    )
    
    # Laser Filter Node (from bearcar.launch.py)
    laser_filter_node = Node(
        package="laser_filters",
        namespace='lidar',
        executable="scan_to_scan_filter_chain",
        parameters=[laser_filter_param_file],
        remappings=[
            ('output', 'scan'),
            ('scan', '/lidar/scan')
        ]
    )
    
    # Static transform: vehicle/base_link -> laser (matching the namespace)
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'vehicle/base_link', 'laser'],
        output='screen'
    )
    
    # SLAM Toolbox launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg_prefix, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('mapping_param_file'),
            'use_sim_time': 'false'
        }.items()
    )
    
    # RViz2 (optional)
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('with_rviz')),
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        mapping_param,
        with_rviz_param,
        robot_publisher,
        rplidar_node,
        laser_filter_node,
        base_to_laser_tf,
        slam_launch,
        rviz2
    ])