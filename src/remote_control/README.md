# Remote Controller
This package handles remote control commands and navigation command conversion for Ackermann steering vehicles.

Data Transmission through the serial port is a string line:

"throttle" pwm_value_throttle "steering" pwm_value_steering

The node extracts these two values from remote controller and publish them.
First it checks if the command string is valid and within the safe range,
finally publish them as Twist messages.

If it freezes, it will see it as the remote controller is disconnected and stop the car.

throttle refers to PWM value sent by Arduino
speed refers to velocity m/s

There are 3 functions for security:

1. After the remote controller is connected, it will not publish commands before the user set the command to zero for several seconds. The exact waiting time is defined as "init_time" in [params](config/params.yaml).

2. If the remote controller is disconnected, the receiver will still keep sending freezed values. Therefore, if the value freezes for some time, this node will assume the remote controller is disconnected. This time is defined as "freeze_check_times" in [params](config/params.yaml).

3. If the drive command is out of limit (this limit is defined in [params](config/params.yaml)). It will skip this value and not publish it.
