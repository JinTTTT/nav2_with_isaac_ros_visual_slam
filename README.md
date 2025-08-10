# ROS2 Navigation Robot Project

A complete 2D LiDAR mapping, Isaac Vslam localization, and navigation system based on ROS2 Nav2.

## Hardware Components

- **LiDAR**: RPLidar (2D laser scanning)
- **Robot Platform**: VESC motor controller
- **Odometry**: Visual slam, topic name: _/visual_slam/tracking/odometry_
- **Navigation**: Nav2

## Control Logic

The system supports dual control modes with automatic priority switching between autonomous navigation and manual remote control:

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
   - Converts unified cmd_vel to VESC motor commands
   - Speed: m/s → ERPM conversion
   - Steering: rad → servo position (0.0-1.0)

### Control Modes

- **Autonomous Mode**: Nav2 provides navigation commands, robot follows planned paths
- **Manual Override**: Joystick input immediately takes control, Nav2 commands ignored
- **Automatic Recovery**: System returns to Nav2 control 0.5s after joystick becomes idle

## Quick Start

### 1. Launch Robot Base System
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
ros2 run nav2_map_server map_saver_cli -f /home/jetsonagx/nav2_ws/lab_scan
```

**SLAM Toolbox serialized map (for SLAM localization):**
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/jetsonagx/nav2_ws/lab_scan'}"
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

## Navigation Features

- **Autonomous Navigation**: Robot navigates to goals avoiding obstacles
- **Path Planning**: Uses SmacPlanner for global path planning  
- **Obstacle Avoidance**: Real-time local costmap and dynamic re-planning
- **Motor Control**: Direct integration with VESC motor controller via cmd_vel

## Project Structure

```
src/
├── bearcar_nav2_launch/           # Main robot package
│   ├── launch/             # Launch files
│   ├── config/             # Configuration parameters
│   └── description/        # Robot URDF description
├── remote_control/         # VESC motor control interface
├── rf2o_laser_odometry/    # Laser odometry
└── rplidar_ros/           # RPLidar driver
```

## Key Configuration Files

- `mapper_params_online_async.yaml`: SLAM Toolbox mapping parameters
- `localization_params_online_async.yaml`: SLAM Toolbox localization parameters
- `amcl_params.yaml`: AMCL localization parameters
- `nav2_params.yaml`: Navigation stack parameters

## Workflow

1. **Mapping Phase**: Robot moves around while SLAM Toolbox builds environment map
2. **Localization Phase**: Uses pre-built map to determine robot position via particle filter (AMCL) or graph optimization (SLAM Toolbox)
3. **Navigation Phase**: Nav2 plans paths and executes autonomous navigation based on map and current position

## Notes

- Ensure LiDAR is properly connected and configured
- Drive robot around sufficiently during mapping for complete coverage
- Set initial pose estimate in RViz before navigation * _only for amcl localization *