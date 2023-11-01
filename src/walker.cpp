/**
 * @file walker.cpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
 * @brief ENPM808X ROS Assignment 4
 * Waffle Pi Turtlebot3 Robot (camera sensor w/ depth), differential Drive, 
 * @version 0.1
 * @date 2023-11-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;


// Velocity control topic: geometry_msgs::msg::Twist>("cmd_vel",10)
// Sensor data topic: sensor_msgs::msg::LaserScan>("scan",10)
