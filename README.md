# walker

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## Overview
Turtlebot3 Simulation using Gazebo in ROS2 for course ENPM808X ROS Assignment 4

## Author
Jerry Pittman, Jr.

## Dependencies
- Ubuntu 22.04 or above
- ROS2 Humble
- Gazebo Ignition (Fortress)
- g11_launch: turtlebot3 (burger)
- gz_launch: turtlebot4 (standard)


## Download/Install
- Create a workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
- Clone the repository into src folder
```
git clone https://github.com/jpittma1/walker.git
```
# Resolve Dependencies using rosdep:
```
rosdep install -i --from-path src --rosdistro humble -y
```

# Build Instructions
```
cd ~/ros2_ws/
colcon build --symlink-install --packages-select walker
source install/setup.bash
```

### Run Instructions 
without Rosbag
```
source install/setup.bash
ros2 launch walker gz_launch.py
```

with Rosbag
```
source install/setup.bash
ros2 launch walker gz_launch.py record:=True
```

To view ros_bag info
```
ros2 bag info walker_bag
```