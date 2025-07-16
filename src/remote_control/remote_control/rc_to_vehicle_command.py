import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class RCtoVehicleCommand(Node):

    def __init__(self):
        super().__init__('rc_to_vehicle_command')

        self.declare_parameter('cmd_topic_name', 'cmd_vel')
        self.declare_parameter('vehicle_cmd_topic_name', 'vehicle_cmd_vel')

        cmd_topic_name = self.get_parameter('cmd_topic_name').get_parameter_value().string_value
        vehicle_cmd_topic_name = self.get_parameter('vehicle_cmd_topic_name').get_parameter_value().string_value

        self.vehicle_cmd_publisher = self.create_publisher(
            Twist, vehicle_cmd_topic_name, 10)
        
        self.twist_subscription = self.create_subscription(
            Twist, cmd_topic_name, self.listener_callback, 10)

    def listener_callback(self, msg):
        # Simply pass through the command - basic safety relay
        self.vehicle_cmd_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    rc_to_vehicle_command = RCtoVehicleCommand()

    rclpy.spin(rc_to_vehicle_command)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rc_to_vehicle_command.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
