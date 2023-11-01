# walker

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## Overview
Turtlebot3 Simulation using Gazebo in ROS2 for course ENPM808X ROS Assignment 4

## Author
Jerry Pittman, Jr.

## Dependencies
- Ubuntu 20.0 or above
- ROS2 Humble
- Gazebo Harmonic

## Build
- Create a workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
- Clone the repository
```
git clone https://github.com/jpittma1/walker.git
```
- Build the workspace
```
cd ~/ros2_ws/src
colcon build --packages-select walker
cd .. && . install/setup.bash
```

- Set turtlebot3 variable for model

```
echo  "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```