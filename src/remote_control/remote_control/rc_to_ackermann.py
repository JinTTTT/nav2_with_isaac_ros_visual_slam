import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class RCtoAckermann(Node):

    def __init__(self):
        super().__init__('control_movement')
        self.ack_publisher = self.create_publisher(
            AckermannDriveStamped, 'ackermann_cmd', 10)

        self.twist_subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)

    def listener_callback(self, msg):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = msg.angular.z
        ack_msg.drive.speed = msg.linear.x
        ack_msg.drive.acceleration = 5.0
        ack_msg.drive.jerk = 10.0
        ack_msg.drive.steering_angle_velocity = 2.0
        self.ack_publisher.publish(ack_msg)

        # self.get_logger().info(f'Publishing: {msg.linear}  {msg.angular}')


def main(args=None):
    rclpy.init(args=args)

    rc_to_ackermann = RCtoAckermann()

    rclpy.spin(rc_to_ackermann)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rc_to_ackermann.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
