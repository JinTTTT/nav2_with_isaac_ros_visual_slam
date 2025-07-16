from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch remote controller, ackermann_to_vesc_node and vesc_driver.
    """
    rc_pkg_prefix = get_package_share_directory('remote_control')
    vesc_ackermann_prefix = get_package_share_directory('vesc_ackermann')
    vesc_driver_prefix = get_package_share_directory('vesc_driver')
    
    # Params
    rc_param_file = os.path.join(
        rc_pkg_prefix, 'config', 'params.yaml')
    
    # Nodes
    rc_publisher = Node( # receive signals and publish Twist messages
        package='remote_control',
        namespace='',
        executable='rc_publisher',
        name='rc_publisher',
        parameters=[rc_param_file],
    )
    rc_to_ackermann = Node( # transform Twist to ackermann msg and publish
        package='remote_control',
        namespace='',
        executable='rc_to_ackermann',
        name='rc_to_ackermann',
    )

    # vesc launch files
    vesc_ackermann_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            vesc_ackermann_prefix, '/launch/ackermann_to_vesc_node.launch.xml']),
    )
    vesc_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            vesc_driver_prefix, '/launch/vesc_driver_node.launch.py']),
    )

    return LaunchDescription([
        rc_publisher,
        rc_to_ackermann,
        vesc_ackermann_launch,
        vesc_driver_launch
    ])