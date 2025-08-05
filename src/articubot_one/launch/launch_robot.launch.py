import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():


    # Package paths
    rsp_package = 'articubot_one'
    bearcar_package = 'bearcar_launch'
    bearcar_package_dir = get_package_share_directory(bearcar_package)

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(rsp_package),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'false'}.items()
    )
    
    # Parameter files
    rc_param_file = os.path.join(bearcar_package_dir, 'params', 'remote_control.param.yaml')
    vesc_param_file = os.path.join(bearcar_package_dir, 'params', 'vesc.param.yaml')
    twist_mux_params = os.path.join(get_package_share_directory(rsp_package), 'config', 'twist_mux.yaml')

    # Ackermann conversion node (converts Nav2 angular velocity to steering angle)
    cmd_vel_to_ackermann = Node(
        package='remote_control',
        namespace='vehicle',
        executable='cmd_vel_to_ackermann',
        name='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{
            'wheelbase': 0.3175,           # Bearcar wheelbase in meters
            'min_velocity': 0.01,          # Minimum velocity to avoid division by zero
            'max_steering_angle': 0.5,     # Max steering angle in radians (~28.6°)
            'debug': True,                 # Enable debug output
        }],
        remappings=[
            ('nav_cmd_vel', '/cmd_vel'),                # Subscribe to Nav2 cmd_vel directly
            ('ackermann_cmd_vel', 'ackermann_cmd_vel'),  # Publish to twist_mux navigation input
        ]
    )


    simple_vesc_interface_node = Node(  # VESC protocol converter - replaces autoware's vesc_interface
        package='remote_control',
        executable='simple_vesc_interface',
        name='simple_vesc_interface',
        namespace='vehicle',
        output='screen',
        parameters=[{
            'speed_to_erpm_gain': 1900.0,
            'speed_to_erpm_offset': 160.0,  # Added 200 offset for navigation mode to ensure minimum speed
            'steering_angle_to_servo_gain': 0.567,    # Original parameters
            'steering_angle_to_servo_offset': 0.4875  #added offset to counter left drift sweet spot should be around 0.485 - 0.49
        }],
        remappings=[
            ('vehicle_cmd_vel', '/cmd_vel_out'),  # Receive directly from twist_mux
            ('commands/motor/speed', 'commands/motor/speed'),      # Publish to vehicle namespace
            ('commands/servo/position', 'commands/servo/position') # Publish to vehicle namespace
        ]
    )

    # Twist Multiplexer - handles Nav2 and joystick cmd_vel inputs
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params],
        remappings=[
            ('cmd_vel_out', '/cmd_vel_out'),  # Output to ackermann converter
        ]
    )

    # Remote controller signal receiver
    rc_publisher = Node(
        package='remote_control',
        namespace='vehicle',
        executable='rc_publisher',
        name='rc_publisher',
        output='screen',
        parameters=[rc_param_file],
        remappings=[
            ('cmd_vel', 'rc_cmd_vel'),  # Publish to /vehicle/rc_cmd_vel for twist_mux
        ]
    )

    vesc_driver_node = Node(  # VESC hardware driver
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace='vehicle',
        output='screen',
        parameters=[vesc_param_file],
        remappings=[
            ('commands/motor/speed', 'commands/motor/speed'),
            ('commands/servo/position', 'commands/servo/position'),
        ]
    )

    # Launch them all!
    return LaunchDescription([
        rsp, # Robot State Publisher
        
        # Control input sources
        rc_publisher,              # Remote controller input
        twist_mux_node,            # Multiplexer for Nav2 and joystick inputs
        
        # Ackermann conversion pipeline
        cmd_vel_to_ackermann,      # Convert multiplexed cmd_vel to steering angle
        simple_vesc_interface_node, # Convert steering angle to VESC commands
        vesc_driver_node,          # VESC hardware driver
    ])


