#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class NavigationCommandPublisher(Node):
    """
    Convert navigation cmd_vel (real physical units) to RC format (normalized values)
    This replaces rc_publisher for autonomous navigation mode.
    """
    def __init__(self):
        super().__init__('nav_cmd_publisher')

        self.get_logger().info('Initiating navigation command converter...')

        # Handle parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_linear_vel', 1.0),   # Maximum linear velocity in m/s
                ('max_angular_vel', 1.0),  # Maximum angular velocity in rad/s
                ('debug', False),
            ]
        )
        
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.debug = self.get_parameter('debug').value

        # Create subscriber for navigation cmd_vel
        self.nav_subscriber = self.create_subscription(
            Twist,
            'nav_cmd_vel',  # This will be remapped to /cmd_vel in launch file
            self.nav_callback,
            10
        )

        # Create publisher for RC format cmd_vel
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)  # This will be remapped to rc_cmd_vel

        self.get_logger().info('Navigation command converter ready!')

    def nav_callback(self, msg):
        """
        Convert navigation cmd_vel to RC format and publish
        
        Navigation format:
        - linear.x: real velocity in m/s (e.g., 0.1 m/s)
        - angular.z: real angular velocity in rad/s (e.g., 0.074 rad/s)
        
        RC format (what rc_publisher would output):
        - linear.x: normalized value -1.0 to 1.0 (1.0 = max forward speed)
        - angular.z: normalized value -0.5 to 0.5 (0.5 = max left turn, -0.5 = max right turn)
        """
        rc_msg = Twist()
        
        # Convert linear velocity: m/s -> normalized -1.0 to 1.0
        rc_msg.linear.x = max(-1.0, min(1.0, msg.linear.x / self.max_linear_vel))
        
        # Convert angular velocity: rad/s -> normalized -0.5 to 0.5  
        # Note: RC format uses ±0.5 range for steering
        rc_msg.angular.z = max(-0.5, min(0.5, msg.angular.z / self.max_angular_vel * 0.5))
        
        # Publish converted command
        self.cmd_publisher.publish(rc_msg)
        
        if self.debug:
            self.get_logger().info(
                f'Nav->RC: linear {msg.linear.x:.3f}m/s -> {rc_msg.linear.x:.3f}, '
                f'angular {msg.angular.z:.3f}rad/s -> {rc_msg.angular.z:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    nav_cmd_publisher = NavigationCommandPublisher()
    rclpy.spin(nav_cmd_publisher)
    nav_cmd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()