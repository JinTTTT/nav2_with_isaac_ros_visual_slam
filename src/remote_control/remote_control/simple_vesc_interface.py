#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class SimpleVescInterface(Node):
    def __init__(self):
        super().__init__('simple_vesc_interface')
        
        # Parameter declaration
        self.declare_parameter('speed_to_erpm_gain', 1900.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter('steering_angle_to_servo_gain', 0.567)
        self.declare_parameter('steering_angle_to_servo_offset', 0.5)
        
        # Get parameters
        self.speed_gain = self.get_parameter('speed_to_erpm_gain').value
        self.speed_offset = self.get_parameter('speed_to_erpm_offset').value
        self.steer_gain = self.get_parameter('steering_angle_to_servo_gain').value
        self.steer_offset = self.get_parameter('steering_angle_to_servo_offset').value
        
        # Publisher - output to vesc_driver
        self.speed_pub = self.create_publisher(Float64, 'commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(Float64, 'commands/servo/position', 10)
        
        # Subscriber - subscribe to processed vehicle commands
        self.cmd_sub = self.create_subscription(
            Twist, 'vehicle_cmd_vel', self.cmd_callback, 10)
            
        self.get_logger().info('Simple VESC Interface started')

    def cmd_callback(self, msg):
        # Speed conversion: m/s -> ERPM
        speed_msg = Float64()
        speed_msg.data = self.speed_gain * msg.linear.x + self.speed_offset
        
        # Steering conversion: rad -> servo position (0.0-1.0)
        servo_msg = Float64() 
        servo_msg.data = self.steer_gain * msg.angular.z + self.steer_offset
        
        # Publish commands
        self.speed_pub.publish(speed_msg)
        self.servo_pub.publish(servo_msg)
        
        self.get_logger().debug(f'Speed: {speed_msg.data:.2f} ERPM, Servo: {servo_msg.data:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleVescInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()