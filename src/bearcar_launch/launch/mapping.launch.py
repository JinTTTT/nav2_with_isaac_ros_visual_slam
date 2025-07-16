# Map the environment.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

import os


def generate_launch_description():

    # package paths
    bearcar_prefix = get_package_share_directory('bearcar_launch')
    slam_toolbox_pkg_prefix = get_package_share_directory('slam_toolbox')

    # parameters
    mapping_param_file = os.path.join(
        bearcar_prefix, "params/mapper_params_online_async.param.yaml"
    )
    mapping_param = DeclareLaunchArgument(
        "mapping_param_file",
        default_value=mapping_param_file,
        description="Path to config file for mapping nodes",
    )

    # nodes
    bearcar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bearcar_prefix,
                         'launch/bearcar.launch.py'),
        ),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg_prefix,
                         'launch/online_async_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('mapping_param_file'),
        }.items()
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('with_rviz')),
        arguments=['-d', LaunchConfiguration("rviz_cfg_path_param")]
    )

    return LaunchDescription([
        # with_rviz_param,
        mapping_param,
        # rviz_cfg_path_param,
        bearcar_launch,
        slam_launch,
        # rviz2
    ])
