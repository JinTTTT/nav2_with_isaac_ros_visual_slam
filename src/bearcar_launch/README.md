This package is a collection of launch files running on Bearcar.
## Launch files explaination
**bearcar.launch.py**: It launches all the sensors, including D455, T265, rplidar, vesc and remote controller. It also provides a tf tree from odom frame to sensor frames, and a odometry topic.

**direct_rc.launch.py**: It launches VESC and remote controller. It is for VESC tuning.

**localization_amcl.launch.py**: localization using amcl.

**localization_slam.launch.py**: localization using slam_toolbox.

**mapping.launch.py**: Create a map of surroundings. For details please refer to [Create Map](#create-map).

**perception.launch.py**: nav2 lifecycle_manager and costmap.

**planning.launch.py**: record replay planner and pure pursuit.

**recordreplay.launch.py**: All in one launch file for recordreplay function.

## Pointcloud of D455
Since align_depth function of ROS2 wrapper of D455 is not working, we have to use ROS1 wrapper to get aligned pointcloud. Therefore, we have to run ROS1 and ROS2 at the same time. 
```{bash}
# Terminal 1
$ source /opt/ros/noetic/setup.bash
$ source ~/yuchen/bearcar_catkin_ws/devel/setup.bash
$ roslaunch ros1_d455 d455.launch
```
```{bash}
# Terminal 2
$ source /opt/ros/noetic/setup.bash
$ source /opt/ros/foxy/setup.bash
$ source ~/yuchen/bridge_ws/install/setup.bash
$ ros2 run ros1_bridge parameter_bridge
```
```{bash}
# Terminal 3
$ source /opt/ros/foxy/setup.bash
$ source ~/yuchen/bearcar_ws/install/setup.bash
$ ros2 launch bearcar_launch bearcar.launch.py # make sure D455 is not launched in ROS2 workspace.
```

## Create Map

```{bash}
# (Terminal 1)
$ ros2 launch bearcar_launch mapping.launch.py
```

Visualize the map building process in Rviz.

Save the map for SLAM by running:

```{bash}
# (Terminal 2)
$ ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: "/PATH/TO/MAP"}" # No Extension in file name
```

This will save a pose graph for `slam_toolbox`'s localization.

After saving maps, stop mapping nodes by pressing `Ctrl+C` in terminal 1.

## Record and replay trajectory
### Record a trajectory

```{bash}
# (Terminal 1)
$ . install/setup.bash
$ ros2 launch bearcar_launch recordreplay.launch.py
```

Turn on remote controller to start driving. 

**We have to give a good initial pose to localize. Set a intial pose with correct orientation in Rviz using `2D pose estimate`**

Now start recording:

```{bash}
# (Terminal 2)
$ ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/home/jetsonagx/yuchen/bearcar_ws/map/record_path"}" --feedback
```
Drive around the vehicle and stop recording with Ctrl + C in terminal 2.

### Replay a trajectory

```{bash}
# (Terminal 1)
$ . install/setup.bash
$ ros2 launch bearcar_launch recordreplay.launch.py
```

Now start replaying. Turn off the joystick so that Bearcar could follow the command given by the planner:

```{bash}
(Terminal 2)
$ ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/home/jetsonagx/yuchen/bearcar_ws/map/record_path"}" --feedback
```

### Running in a Loop

To let the vehicle run continuously in a loop, first record a trajectory finished with a stop position just behind the start position. Modify bearcar_launch/params/recordreplay_planner.param.yaml by setting `loop_trajectory` to `True`. Build and run the replay.

# TODO
1. The replayed velocity might not be exactly the same with the recorded one. This is due to limitation of `Pure Pursuit` algorithm. The implementation does not take delay and external force into consideration, which means that it assumes constant speed when acceleration is released even if break is not pressed. This causes velocity of the vehicle to be wrong. Improvements will be made in the future.
2. Setting the initial position of the car on rviz2 shouldn't be necessary. Implementation of initializing its pose will be made.
3. T265 doesn't fuse wheel odometry as expected.
In src/realsense-ros/realsense2_camera/src/realsense_node_factory.cpp
line 389: `_realSenseNode->publishTopics();`\
When the hardware is T265, `_realSenseNode` should be `std::unique_ptr<T265RealsenseNode>(new T265RealsenseNode(*this, _device, _parameters, this->get_node_options().use_intra_process_comms()));` as stated in line 382. \
But it still calls the function in class `BaseRealSenseNode` instead of `T265RealsenseNode` \
Need ROS2 C++ debug, find why is this object not initialized correctly.
4. Remote controller sometimes give random commands after going to stop.

# Troubleshoot

## No Map in Rviz; LiDAR Message Dropped

X button needs to be pressed once for the map to show in Rviz when building the map or recording/replaying the trajectory.

If you have custom LiDAR with its own ROS2 driver, make sure the `LaserScan`'s `frame_id` is `lidar`, topic name is `scan`, and the node namespace is `lidar`. If the driver outputs `PointCloud2`, use the conversion node to republish it as a `LaserScan`.

## Terrible Map; Odometry Drifting

Please make sure there are enough obstacles around the track for the LiDAR. For example, an outdoor playground with a few cones would usually not work, Whereas an indoor space with nearby walls would be preferable.

A map is often bad due to uncalibrated odometry, although slight difference is acceptable. You can visualize the vehicle odometry by adding a visualization in Rviz for `/vehicle/odom`. Check the trajectory of the vehicle, and look for flipped travel direction and/or flipped steering direction. Correct them in the following two files.

Your vehicle odometry may need to be recalibrated, and values put into `src/bearcar_launch/params/vesc_to_odom_node.param.yaml` and `src/bearcar_launch/params/vesc_config.param.yaml`. Refer to the F1Tenth documentation for calibrating odometry (Driving the F1TENTH Car -> Calibrating the Odometry).

## Vehicle not Moving When Replaying

Turn off the remote controller. Otherwise the vehicle would not move.

If the car starting position is too close to the final trajectory point and the looping feature is off, the replay goal would be reached instantly and the car will not drive.

