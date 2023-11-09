# walker

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## Overview
Turtlebot3 Simulation using Gazebo in ROS2 for course ENPM808X ROS Assignment 4

## Author
Jerry Pittman, Jr.

## Dependencies
- Ubuntu 22.04 or above
- ROS2 Humble
- Gazebo 11 (Classic) or Gazebo Ignition (Fortress)
- turtlebot4_desktop (turtlebot4_viz, turtlebot_description)
- turtlebot4_ignition_bringup

<!-- - ros_gz_example_bringup -->

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
<!-- colcon build --packages-select walker -->

<!-- - Set turtlebot3 variable for model

```
echo  "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
``` -->

### Run Instructions
Terminal 1
```
source install/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```

Terminal 2
```
source install/setup.bash
ros2 launch walker gz_launch.py
ros2 launch walker g11_launch.py
ros2 run walker walker
```