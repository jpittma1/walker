# walker

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## Overview
Turtlebot3 Simulation using Gazebo in ROS2 for course ENPM808X ROS Assignment 4

## Author
Jerry Pittman, Jr.

## Dependencies
- Ubuntu 22.04 or above
- ROS2 Humble
- Gazebo Garden, Harmonic or 11?!?
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
rosdep install -i --from-path src --rosdistro humble -y

# Build Instructions
```
cd ~/ros2_ws/
colcon build --packages-select walker
source install/setup.bash
```

- Set turtlebot3 variable for model

```
echo  "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```

### Run Instructions

```
source install/setup.bash
ros2 launch ros2_turtlebot gazebo.py
```