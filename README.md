# Nav2 BearCar with RealSense D455
This project implements complete autonomous navigation capabilities on BearCar platform using Nav2 navigation stack with dual sensor support for mapping, localization, and navigation applications.

## Platform Requirements

- **Hardware**: BearCar platform on NVIDIA Jetson Xavier AGX
- **JetPack**: 5.1
- **Host ROS Distribution**: ROS2 Foxy
- **Container ROS Distribution**: ROS2 Humble (for Isaac ROS Visual SLAM)
- **Isaac ROS**: 2.1 (see [Isaac ROS Visual SLAM workspace](https://github.com/JinTTTT/issac_ros_visual_slam_d455))

## Purpose
This workspace provides complete autonomous navigation functionality combining:

- **Mapping**: 2D LiDAR-based SLAM using SLAM Toolbox
- **Localization**: Dual-mode support for LiDAR-only or LiDAR + Visual SLAM
- **Navigation**: Nav2 navigation stack with obstacle avoidance
- **Control**: Priority-based manual override system

## Environment
The project operates in dual environments:

- **Host Environment**: ROS2 Foxy for main navigation stack
- **Isaac Container**: ROS2 Humble for Visual SLAM (when using RealSense)

## Hardware Components

- **LiDAR**: RPLidar (2D laser scanning)
- **Camera**: RealSense D455 (optional, for visual odometry)
- **Robot Platform**: BearCar with VESC motor controller
- **Control**: Remote controller with priority-based switching

## Control Logic

The system uses VESC motor controller as the primary drive unit, equivalent to ros2_control in standard Nav2 setups. The architecture supports simultaneous operation of autonomous navigation and manual control through priority-based switching.

### Key Features
- **Dual Control**: Nav2 autonomous navigation and manual joystick control operate simultaneously
- **Priority Override**: Joystick control has higher priority through twist_mux, allowing operator takeover at any time
- **Seamless Switching**: Automatic return to Nav2 control when joystick becomes idle

### Data Flow Architecture
```
Nav2 → /cmd_vel → cmd_vel_to_ackermann → /vehicle/ackermann_cmd_vel ↘
                                                                     ↘
                                                                     twist_mux → /cmd_vel_out → simple_vesc_interface → VESC
Remote Controller → /vehicle/rc_cmd_vel ─────────────────────────────↗
```

### Control Components

1. **Nav2 Path**: Autonomous navigation commands
   - Nav2 publishes angular velocity (`/cmd_vel`)
   - `cmd_vel_to_ackermann` converts angular velocity to steering angle using Ackermann geometry
   - Output: `/vehicle/ackermann_cmd_vel` with steering angle format

2. **Remote Control Path**: Manual joystick control
   - `rc_publisher` reads joystick PWM signals and converts to steering angles
   - Includes deadzone filtering to eliminate noise (±0.02 m/s, ±0.02 rad)
   - Output: `/vehicle/rc_cmd_vel` with steering angle format

3. **Twist Multiplexer**: Priority-based input selection
   - **Remote Control Priority**: 100 (highest)
   - **Navigation Priority**: 10 (lower)
   - **Timeout**: 0.5 seconds for automatic switching
   - **Behavior**: Remote control overrides Nav2 when active; switches back to Nav2 when joystick idle

4. **VESC Interface**: Hardware command conversion
   - Replaces standard ros2_control hardware interface
   - Converts unified cmd_vel to VESC motor commands
   - Speed: m/s → ERPM conversion
   - Steering: rad → servo position (0.0-1.0)

## Installation

Install required ROS2 Foxy packages:

```bash
sudo apt update
sudo apt install -y \
  ros-foxy-nav2-bringup \
  ros-foxy-slam-toolbox \
```

For visual odometry functionality, set up Isaac ROS Visual SLAM in the Isaac container. See the [Isaac ROS Visual SLAM workspace](https://git.tu-berlin.de/tj965737461/remote_control_ws) for detailed setup instructions.

## Usage

### Basic Launch Sequence

#### 1. Launch Robot Base System
```bash
ros2 launch bearcar_nav2_launch launch_robot.launch.py
```
Starts robot state publisher and motor control interface.

### 2. Launch LiDAR
```bash
ros2 launch bearcar_nav2_launch launch_lidar_only.launch.py 
```
Starts RPLidar node and RF2O laser odometry.

### 3. Launch Isaac Vslam for Odometry

- go to isaac container
```bash
docker start -ai isaac_ros_dev-aarch64-container 
```

- source
```bash
soure install/setup.bash
```

- start vslam
```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

## Mapping Mode *

### 3. Launch Mapping
```bash
ros2 launch bearcar_nav2_launch launch_mapping.launch.py
```
Starts SLAM Toolbox for real-time mapping. Drive the robot around to build the map.

### 4. Save Map

**Standard format map (for AMCL):**
```bash
ros2 run nav2_map_server map_saver_cli -f /home/jetsonagx/nav2_ws/src/map/lab_scan
```

**SLAM Toolbox serialized map (for SLAM localization):**
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/jetsonagx/nav2_ws/src/map/lab_scan'}"
```

## Navigation Mode

### 3. Launch Localization
Starts SLAM Toolbox localization with pre-built map. 

Purpose is to use scanned info compare with the pre-built map, calculate the transform of map - odom
```bash
ros2 launch bearcar_nav2_launch launch_localization.launch.py
```

**Alternative: AMCL Localization**
Uses standard format map with AMCL algorithm.
```bash
ros2 launch bearcar_nav2_launch launch_amcl_localization.launch.py
```

**⚠️ Warning**: AMCL has integration issues with Nav2 in this setup. While AMCL works fine as a standalone localization solution, it may cause problems when combined with Nav2 navigation. Use SLAM Toolbox localization for reliable Nav2 integration.

### 4. Launch Navigation
Starts Nav2 navigation stack.
```bash
ros2 launch bearcar_nav2_launch navigation_launch.py
```

### 5. Set Navigation Goal
- Open RViz2
- Select "2D Goal Pose" tool
- Click on the map to set destination
- Robot will automatically navigate to the target

### Performance Monitoring

Monitor Jetson system performance during navigation:
```bash
python3 jetson_monitor.py
```

Optional parameters:
```bash
# Monitor for 60 seconds with 2-second intervals
python3 jetson_monitor.py -t 60 -i 2.0 -o performance_data.json
```

This script monitors CPU, GPU, memory usage and temperature, providing real-time performance statistics and saving data for analysis.

## Navigation Features

- **Autonomous Navigation**: Robot navigates to goals avoiding obstacles
- **Path Planning**: Uses SmacPlanner for global path planning  
- **Obstacle Avoidance**: Real-time local costmap and dynamic re-planning
- **Motor Control**: Direct integration with VESC motor controller via cmd_vel

## Key Configuration Files

- `mapper_params_online_async.yaml`: SLAM Toolbox mapping parameters
- `localization_params_online_async.yaml`: SLAM Toolbox localization parameters
- `amcl_params.yaml`: AMCL localization parameters
- `nav2_params.yaml`: Navigation stack parameters

## Workflow

1. **Mapping Phase**: Robot moves around while SLAM Toolbox builds environment map
2. **Localization Phase**: Uses pre-built map to determine robot position via particle filter (AMCL) or graph optimization (SLAM Toolbox)
3. **Navigation Phase**: Nav2 plans paths and executes autonomous navigation based on map and current position