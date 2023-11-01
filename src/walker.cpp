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

class Walker : public rclcpp::Node {
 public:
 Walker() : Node("Walker"), collision_distance_(0.5) {
    RCLCPP_INFO(this->get_logger(), "Setting up publisher and subcriber");
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Walker::scan_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Walker Node Initialized!");
  }

 private:
  
  void scan_callback(
      const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) const {
    int16_t start_idx = 45;
    int16_t end_idx = 315;
    geometry_msgs::msg::Twist cmd_vel_msg;
    double scan_max = scan_msg->range_max;
    double min_dist_to_obstacle = scan_max;

    for (int16_t i = 0; i < int16_t(scan_msg->ranges.size()); i++) {
      if (i <= start_idx || i >= end_idx) {
        if (!std::isnan(scan_msg->ranges[i])) {
          double scan_dist = scan_msg->ranges[i];
          if (scan_dist < min_dist_to_obstacle) {
            min_dist_to_obstacle = scan_dist;
          }
        }
      }
    }
    if (min_dist_to_obstacle <= collision_distance_) {
      RCLCPP_WARN(this->get_logger(), "Obstacle on path!");
      RCLCPP_INFO(this->get_logger(), "Turning to avoid obstacle!");
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = -0.5;
    } else {
      RCLCPP_INFO(this->get_logger(), "No Obstacles Found!");
      cmd_vel_msg.linear.x = -0.3;
      cmd_vel_msg.angular.z = 0.0;
    }
    vel_pub_->publish(cmd_vel_msg);
    }
    
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  double collision_distance_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::string cmd_vel_topic = "/cmd_vel";
  std::string laser_topic = "/scan";
};


// Velocity control topic: geometry_msgs::msg::Twist>("cmd_vel",10)
// Sensor data topic: sensor_msgs::msg::LaserScan>("scan",10)
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}