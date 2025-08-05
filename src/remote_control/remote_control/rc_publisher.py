#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class RemoteControlPublisher(Node):
    """
    There are three cmd variable lists:
    line_cmd: (list) pwm values splited by serial input line, always updated if there is data in serial port.
    self.split_cmd: (list) pwm values from line_cmd if data is valid and not freezed.
    self.drive_cmd: (list) metric values derived from self.split_cmd if data is within the range and ready to publish.
    """
    def __init__(self):
        super().__init__('rc_publisher')

        self.get_logger().info('Initiating remote controller...')

        # Handle parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('cmd_topic_name', None),
                ('read_period', None),
                ('port_name', None),
                ('baud_rate', None),
                ('speed_max', None),
                ('speed_min', None),
                ('steer_max', None),
                ('steer_min', None),
                ('init_time', None),
                ('speed_init', None),
                ('steer_init', None),
                ('speed_offset', None),
                ('speed_ratio', None),
                ('steer_offset', None),
                ('steer_ratio', None),
                ('speed_init_thres', None),
                ('steer_init_thres', None),
                ('freeze_check_times', None),
                ('speed_deadzone', None),
                ('steer_deadzone', None),
                ('debug', None),
            ]
        )
        cmd_topic_name = self.get_parameter('cmd_topic_name').value
        self.read_period = self.get_parameter('read_period').value  # seconds
        port_name = self.get_parameter('port_name').value
        baud_rate = self.get_parameter('baud_rate').value
        self.init_time = self.get_parameter('init_time').value
        self.freeze_check_times = self.get_parameter('freeze_check_times').value

        self.speed_max = self.get_parameter('speed_max').value
        self.speed_min = self.get_parameter('speed_min').value
        self.steer_max = self.get_parameter('steer_max').value
        self.steer_min = self.get_parameter('steer_min').value

        self.speed_offset = self.get_parameter('speed_offset').value
        self.speed_ratio = self.get_parameter('speed_ratio').value
        self.steer_offset = self.get_parameter('steer_offset').value
        self.steer_ratio = self.get_parameter('steer_ratio').value

        self.speed_init = self.get_parameter('speed_init').value
        self.steer_init = self.get_parameter('steer_init').value
        self.speed_init_thres = self.get_parameter('speed_init_thres').value
        self.steer_init_thres = self.get_parameter('steer_init_thres').value
        
        self.speed_deadzone = self.get_parameter('speed_deadzone').value
        self.steer_deadzone = self.get_parameter('steer_deadzone').value
        
        self.debug = self.get_parameter('debug').value

        self.is_data_accepted = False

        # Dictionary stores drive commands read from serial input
        self.drive_cmd = {"speed": self.speed_offset, "steer": self.steer_offset}

        # Set up serial communication with Arduino
        self.ser = serial.Serial(port_name, baud_rate, timeout=1)
        self.ser.reset_input_buffer()

        # Create vesc drive command publisher - use parameter configuration
        self.cmd_publisher = self.create_publisher(Twist, cmd_topic_name, 10)

        self.read_cnt = 0 # Count how many times of check is passed, used in init and freeze detection
        self.split_cmd = {"throttle": self.speed_init, "steering": self.steer_init} # detect freeze
        self.read_timer = self.create_timer(self.read_period, self.read_callback)
        self.initialize()
        
    def initialize(self):
        """
        check if the remote controller is connected and control value is reset to zero.
        """
        self.get_logger().info("Reset the car!")
        self.stop_car()
        try:
            self.destroy_timer(self.pub_timer)
        except:
            pass
        try:
            self.destroy_timer(self.init_timer)
        except:
            pass
        self.init_timer = self.create_timer(self.init_time, self.init_check_callback)
        
    
    def init_check_callback(self):
        """
        Checks the connection of remote controller when initializing.
        cmd value should be around offset for several seconds to proceed.
        If successful, it will destroy the init_timer and create the publish timer.
        """
        if self.is_data_accepted: # Serial input is valid drive command string
            rcv_speed, rcv_steer = self.convert_data(self.split_cmd["throttle"], self.split_cmd["steering"])
            if abs(rcv_speed) < self.speed_init_thres and abs(rcv_steer) < self.steer_init_thres:
                self.destroy_timer(self.init_timer)
                # Create timer to read serial inputs continuously
                self.pub_timer = self.create_timer(self.read_period, self.publish_callback)
            else:
                self.get_logger().info('Current drive command speed: %.3f, steer: %.3f' %(rcv_speed, rcv_steer))
                self.get_logger().info('Please set the value of remote controller to zero.')
        else:
            self.get_logger().info('Remote controller is not connected.', once=True)

    def read_callback(self):
        """
        Read the last line from Serial input and check its validity.
        Also checks if the data freezes.
        If the data is accepted, then it will update the split_cmd,
        otherwise to False.
        """
        try:
            if self.ser.in_waiting > 0:
                # read last line
                while self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').rstrip()
                    if self.debug:
                        self.get_logger().info('Reading lines: "%s"' %line)
                line_cmd = line.split(" ")

                # check if the string is valid, sometimes line break is missing
                if len(line_cmd) == 4 and line_cmd[0] == "throttle" and line_cmd[2] == "steering":
                    try:
                        # check if data changes
                        if int(line_cmd[1]) == self.split_cmd["throttle"] and \
                            int(line_cmd[3]) == self.split_cmd["steering"]:
                            self.read_cnt = self.read_cnt + 1
                            if self.read_cnt >= self.freeze_check_times: # data freezes
                                if self.read_cnt == self.freeze_check_times:
                                    self.get_logger().info('Remote controller is disconnected. Reset the car.', once=True)
                                    self.initialize()
                            else: # data freezes for a short time
                                return
                        else: # data changes
                            self.split_cmd["throttle"] = int(line_cmd[1])
                            self.split_cmd["steering"] = int(line_cmd[3])
                            self.read_cnt = 0 # reset cnt
                            self.is_data_accepted = True
                            return
                    except:
                        if self.debug:
                            self.get_logger().info('Receiving invalid messages: "%s"' %line)
                else:
                    if self.debug:
                        self.get_logger().info('Receiving invalid messages: "%s"' %line)
        except:
            self.get_logger().info('Arduino is not detected.')
            self.initialize()

        self.is_data_accepted = False

    def check_and_save_cmd_value(self):
        """
        Check if the command value is within safe range.
        If yes, update self.drive_cmd with new values
        """      
        rcv_speed, rcv_steer = self.convert_data(self.split_cmd["throttle"], self.split_cmd["steering"])

        if rcv_speed < self.speed_max and rcv_speed > self.speed_min and \
            rcv_steer < self.steer_max and rcv_steer > self.steer_min:
            self.drive_cmd["steer"] = rcv_steer
            self.drive_cmd["speed"] = rcv_speed
            return True

        else: # value out of limit, not assign to drive_cmd until it goes back to normal
            if self.debug:
                if rcv_speed >= self.speed_max:
                    self.get_logger().info('speed command value "%.3f" is higher than max' %rcv_speed)
                elif rcv_speed <= self.speed_min:
                    self.get_logger().info('speed command value "%.3f" is lower than min' %rcv_speed)
                if rcv_steer >= self.speed_max:
                    self.get_logger().info('Steering command value "%.3f" is higher than max' %rcv_steer)
                elif rcv_steer <= self.steer_min:
                    self.get_logger().info('Steering command value "%.3f" is lower than min' %rcv_steer)
            return False

    def publish_cmd(self):
        """
        Publish data to corresponding topics.
        """
        msg = Twist()
        msg.linear.x = self.drive_cmd["speed"]
        msg.angular.z = self.drive_cmd["steer"]
        self.cmd_publisher.publish(msg)
    
    def publish_callback(self):
        """
        Publish callback function to publish data as Twist if data is accepted and significant.
        """
        if self.is_data_accepted: # Serial input is valid drive command string
            if self.check_and_save_cmd_value():  # cmd is in safe range
                # Only publish if movement is above deadzone threshold
                if self.has_significant_movement(self.drive_cmd["speed"], self.drive_cmd["steer"]):
                    self.publish_cmd()
                    if self.debug:
                        self.get_logger().info('Publishing speed: %.3f, steer: %.3f' %(self.drive_cmd["speed"], self.drive_cmd["steer"]) )
                elif self.debug:
                    self.get_logger().info('Movement below deadzone - not publishing: speed: %.3f, steer: %.3f' %(self.drive_cmd["speed"], self.drive_cmd["steer"]) )
        # 如果没有有效数据或动作太小，不发布任何消息，让twist_mux超时切换到Nav2
            
    def stop_car(self):
        """
        Publish drive_cmd as 0 to make the car stop
        """
        self.drive_cmd["steer"] = self.steer_init
        self.drive_cmd["speed"] = self.speed_init
        self.publish_cmd()
    
    def has_significant_movement(self, speed, steer):
        """
        Check if the movement is significant enough to publish (above deadzone)
        :param speed: speed in m/s
        :param steer: steering in rad
        :return: True if movement is above deadzone thresholds
        """
        return abs(speed) > self.speed_deadzone or abs(steer) > self.steer_deadzone

    def convert_data(self, pwm_speed, pwm_steer):
        """
        convert the speed and steering from pwm value to (m/s) and (rad)
        :param pwm_speed: 
        :param pwm_steer:   
        :return speed: speed in (m/s)
        :return steer: steering in (rad)
        """
        speed = (float(pwm_speed) - self.speed_offset) / self.speed_ratio # convert to (m/s)
        steer = (float(pwm_steer) - self.steer_offset) / self.steer_ratio # convert to rad
        return speed, steer


def main(args=None):
    rclpy.init(args=args)
    rc_publisher = RemoteControlPublisher()
    rclpy.spin(rc_publisher)
    rc_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()