#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class CmdVelToAckermann(Node):
    """
    Converts Nav2 cmd_vel (with angular velocity) to Ackermann cmd_vel (with steering angle)
    
    Input (Nav2 format):
    - linear.x: linear velocity in m/s
    - angular.z: angular velocity in rad/s (body-fixed frame)
    
    Output (Ackermann format):
    - linear.x: linear velocity in m/s (unchanged)
    - angular.z: steering angle in rad
    
    Conversion formula: steering_angle = atan(angular_velocity * wheelbase / linear_velocity)
    """
    
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        
        # Declare parameters
        self.declare_parameter('wheelbase', 0.3175)  # Default bearcar wheelbase
        self.declare_parameter('min_velocity', 0.01)  # Minimum velocity to avoid division by zero
        self.declare_parameter('max_steering_angle', 0.5)  # Max steering angle in rad (~28.6 degrees)
        self.declare_parameter('debug', False)
        
        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.min_velocity = self.get_parameter('min_velocity').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.debug = self.get_parameter('debug').value
        
        # Subscriber to Nav2 cmd_vel (angular velocity format)
        self.nav_cmd_sub = self.create_subscription(
            Twist,
            'nav_cmd_vel',  # Will be remapped to /cmd_vel
            self.nav_cmd_callback,
            10
        )
        
        # Publisher for Ackermann cmd_vel (steering angle format)
        self.ackermann_cmd_pub = self.create_publisher(
            Twist,
            'ackermann_cmd_vel',  # Will be remapped to simple_vesc_interface input
            10
        )
        
        self.get_logger().info(f'cmd_vel to Ackermann converter started')
        self.get_logger().info(f'Wheelbase: {self.wheelbase:.4f}m, Max steering: {self.max_steering_angle:.3f}rad')

    def nav_cmd_callback(self, nav_msg):
        """
        Convert Nav2 cmd_vel to Ackermann format
        """
        ackermann_msg = Twist()
        
        # Linear velocity remains unchanged
        ackermann_msg.linear.x = nav_msg.linear.x
        
        # Convert angular velocity to steering angle
        if abs(nav_msg.linear.x) < self.min_velocity:
            # For low/zero velocity, use angular velocity directly as steering angle
            # This allows for turning in place or at very low speeds
            angular_velocity = nav_msg.angular.z
            ackermann_msg.angular.z = max(-self.max_steering_angle, 
                                        min(self.max_steering_angle, angular_velocity))
        else:
            # Apply Ackermann steering formula
            # steering_angle = atan(angular_velocity * wheelbase / linear_velocity)
            steering_angle = math.atan(nav_msg.angular.z * self.wheelbase / nav_msg.linear.x)
            
            # Clamp steering angle to maximum allowed
            ackermann_msg.angular.z = max(-self.max_steering_angle, 
                                        min(self.max_steering_angle, steering_angle))
        
        # Publish converted command
        self.ackermann_cmd_pub.publish(ackermann_msg)
        
        if self.debug:
            self.get_logger().info(
                f'Nav2->Ackermann: linear {nav_msg.linear.x:.3f}m/s, '
                f'angular {nav_msg.angular.z:.3f}rad/s -> steering {ackermann_msg.angular.z:.3f}rad '
                f'({math.degrees(ackermann_msg.angular.z):.1f}°)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()