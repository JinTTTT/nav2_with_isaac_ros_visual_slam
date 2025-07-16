import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

def generate_launch_description():
    bearcar_prefix = get_package_share_directory('bearcar_launch')

    # Bearcar param file
    rc_param_file = os.path.join(
        bearcar_prefix, 'params/remote_control.param.yaml')
    vesc_to_odom_node_param_file = os.path.join(
        bearcar_prefix, 'params/vesc_to_odom_node.param.yaml')
    vesc_config = os.path.join(
        bearcar_prefix, 'params/vesc.param.yaml')
    vesc_interface_param_file = os.path.join(
        bearcar_prefix, 'params/vesc_config.param.yaml'
    )

    bearcar_urdf = os.path.join(
        bearcar_prefix, 'urdf/bearcar.urdf')
    with open(bearcar_urdf, 'r') as infp:
        bearcar_urdf_file = infp.read()

    # Nodes
    vesc_driver_launcher = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace='vehicle',
        output='screen',
        parameters=[vesc_config]
    )

    vesc_to_odom_launcher = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        namespace='vehicle',
        output='screen',
        parameters=[vesc_to_odom_node_param_file],
        remappings=[
            ("odom", "vesc_odom")
        ]
    )

    vesc_interface_node = Node(
        package='vesc_interface',
        executable='vesc_interface_node_exe',
        namespace='vehicle',
        output='screen',
        parameters=[vesc_interface_param_file],
    )

    
    rc_publisher = Node( # receive signals and publish Twist messages
        package='remote_control',
        namespace='vehicle',
        executable='rc_publisher',
        name='rc_publisher',
        output='screen',
        parameters=[rc_param_file],
    )

    rc_controller = Node( # transform Twist to ackermann msg and publish
        package='remote_control',
        namespace='vehicle',
        executable='rc_to_vehicle_command', # rc_to_ackermann
        name='rc_to_vehicle_command', # rc_to_ackermann
    )


    return launch.LaunchDescription([
        rc_publisher, # only if we are using joystick
        rc_controller, # cmd_vel to Ackermann
        
        vesc_driver_launcher,
        vesc_to_odom_launcher,
        vesc_interface_node,
    ])
