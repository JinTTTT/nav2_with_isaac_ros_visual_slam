# ROS2 Navigation Robot Project

A complete 2D LiDAR mapping, localization, and navigation system based on ROS2 Nav2.

## Hardware Components

- **LiDAR**: RPLidar (2D laser scanning)
- **Odometry**: RF2O laser odometry
- **Robot Platform**: Articubot One with VESC motor controller

## Quick Start

### 1. Launch Robot Base System
```bash
ros2 launch articubot_one launch_robot.launch.py
```
Starts robot state publisher and motor control interface.

### 2. Launch LiDAR and Odometry
```bash
ros2 launch articubot_one launch_lidar_odom.launch.py
```
Starts RPLidar node and RF2O laser odometry.

## Mapping Mode

### 3. Launch Mapping
```bash
ros2 launch articubot_one launch_mapping.launch.py
```
Starts SLAM Toolbox for real-time mapping. Drive the robot around to build the map.

### 4. Save Map

**Standard format map (for AMCL):**
```bash
ros2 run nav2_map_server map_saver_cli -f /home/jetsonagx/nav2_ws/my_map_name
```

**SLAM Toolbox serialized map (for SLAM localization):**
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/jetsonagx/nav2_ws/my_map_name'}"
```

## Navigation Mode

### 3. Launch Localization
```bash
ros2 launch articubot_one launch_localization.launch.py
```
Starts SLAM Toolbox localization with pre-built map.

**Alternative: AMCL Localization**
```bash
ros2 launch articubot_one launch_amcl_localization.launch.py
```
Uses standard format map with AMCL algorithm.

### 4. Launch Navigation
```bash
ros2 launch articubot_one navigation_launch.py
```
Starts Nav2 navigation stack.

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
├── articubot_one/           # Main robot package
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
- Set initial pose estimate in RViz before navigation
- Choose localization method based on specific requirements