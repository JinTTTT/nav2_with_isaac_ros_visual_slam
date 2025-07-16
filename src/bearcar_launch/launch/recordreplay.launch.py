# Copyright 2020-2022, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

import os


def IfEqualsCondition(arg_name: str, value: str):
    return IfCondition(PythonExpression([
        '"', LaunchConfiguration(arg_name), '" == "', value, '"'
    ]))


def generate_launch_description():
    # package paths
    bearcar_prefix = get_package_share_directory('bearcar_launch')
    map_path = os.path.join(bearcar_prefix, 'map/map14_slam')
    map_amcl_file_path = os.path.join(
        bearcar_prefix, 'map/map14_amcl.yaml'
    )
    map_amcl_file = DeclareLaunchArgument(
        'map',
        default_value=map_amcl_file_path,
        description='Path to 2D map config file'
    )
    map_slam_file = DeclareLaunchArgument(
        'map',
        default_value=LaunchConfiguration('map_file_path'),
        description='Path to 2D map config file'
    )

    # params
    localization_mode_param = DeclareLaunchArgument(
        'localization',
        default_value='slam',
        description='Use \'amcl\' or \'slam\' for localization'
    )

    # Nodes
    bearcar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bearcar_prefix,
                         'launch/bearcar.launch.py'),
        ),
    )

    localization_amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bearcar_prefix,
                         'launch/localization_amcl.launch.py'),
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
        }.items(),
        condition=IfEqualsCondition("localization", "amcl")
    )

    localization_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bearcar_prefix,
                         'launch/localization_slam.launch.py'),
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
        }.items(),
        condition=IfEqualsCondition("localization", "slam")
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bearcar_prefix,
                         'launch/planning.launch.py'),
        )
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bearcar_prefix,
                         'launch/perception.launch.py'),
        )
    )

    ld = LaunchDescription()
    ld.add_entity(localization_mode_param)
    ld.add_entity(map_slam_file)

    ld.add_action(bearcar_launch)
    ld.add_action(localization_slam_launch)
    ld.add_action(planning_launch)
    ld.add_action(perception_launch)
    
    return ld
