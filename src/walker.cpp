/**
 * @file walker.cpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
 * @brief ENPM808X ROS Assignment 4
 * Turtlebot4 Robot (camera sensor w/ depth), differential Drive
 * Velocity control topic: geometry_msgs::msg::Twist>("cmd_vel",10)
 *  Sensor data topic: sensor_msgs::msg::LaserScan>("scan",10)
 * @version 1.0
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

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

using IMAGE = sensor_msgs::msg::Image;
using TWIST = geometry_msgs::msg::Twist;

/**
 * @brief State of movement
 * 
 */
typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
} StateType;

/**
 * @brief Walker class
 * 
 */
class Walker : public rclcpp::Node {
public:
/**
 * @brief Construct a new Walker object
 * 
 */
Walker() : Node("walker"), state_(STOP) {
// Walker() : Node("Walker"), collision_distance_(0.5) {

    
    // /ir_intensity
    RCLCPP_INFO(this->get_logger(), "Setting up publisher and subcriber");
    auto pubTopicName = "cmd_vel";
    vel_pub_ = this->create_publisher<TWIST>(pubTopicName, 10);

    auto subTopicName = "/oakd/rgb/preview/depth";
    auto subCallback = std::bind(&Walker::subscribe_callback, this, _1);
    subscription_ =
        this->create_subscription<IMAGE>(subTopicName, 10, subCallback);
    
    // create a 10Hz timer for processing
    auto timerCallback = std::bind(&Walker::timer_callback, this);
    timer_ = this->create_wall_timer(100ms, timerCallback);
    // subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     subTopicName, 10, std::bind(&Walker::subscribe_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Walker Node Initialized!");
}

private:
    void subscribe_callback(const IMAGE &msg) { lastImg_ = msg; }
    /**
     * @brief callback for laser scan
     * 
     * @param scan_msg 
     */
    // void scan_callback(
    //         const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) const {
    //     int16_t start_idx = 45;
    //     int16_t end_idx = 315;
    //     TWIST cmd_vel_msg;
    //     double scan_max = scan_msg->range_max;
    //     double min_dist_to_obstacle = scan_max;

    //     for (int16_t i = 0; i < int16_t(scan_msg->ranges.size()); i++) {
    //         if (i <= start_idx || i >= end_idx) {
    //             if (!std::isnan(scan_msg->ranges[i])) {
    //                 double scan_dist = scan_msg->ranges[i];
    //                 if (scan_dist < min_dist_to_obstacle) {
    //                     min_dist_to_obstacle = scan_dist;
    //                 }
    //             }
    //         }
    //     }
    //     if (min_dist_to_obstacle <= collision_distance_) {
    //         RCLCPP_WARN(this->get_logger(), "Obstacle on path!");
    //         RCLCPP_INFO(this->get_logger(), "Turning to avoid obstacle!");
    //         cmd_vel_msg.linear.x = 0.0;
    //         cmd_vel_msg.angular.z = -0.5;
    //     } else {
    //         RCLCPP_INFO(this->get_logger(), "No Obstacles Found!");
    //         cmd_vel_msg.linear.x = 0.5;
    //         cmd_vel_msg.angular.z = 0.0;
    //     }
    //     vel_pub_->publish(cmd_vel_msg);
    // }

    void timer_callback() {
        // Do nothing until the first data read
        if (lastImg_.header.stamp.sec == 0) return;

        // Create the message to publish (initialized to all 0)
        auto message = TWIST();

        // state machine (Mealy -- output on transition)
        switch (state_) {
        case FORWARD:
            if (hasObstacle()) {  // check transition
            state_ = STOP;
            vel_pub_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
            }
            break;
        case STOP:
            if (hasObstacle()) {  // check transition
            state_ = TURN;
            message.angular.z = 0.1;
            vel_pub_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = TURN");
            } else {
            state_ = FORWARD;
            message.linear.x = -0.1;
            vel_pub_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
            }
            break;
        case TURN:
            if (!hasObstacle()) {  // check transition
            state_ = FORWARD;
            message.linear.x = -0.1;
            vel_pub_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
            }
            break;
        }
    }

    bool hasObstacle() {
        unsigned char *dataPtr = lastImg_.data.data();
        float *floatData = (float *)dataPtr;

        int idx;
        for (unsigned int row = 0; row < lastImg_.height - 40; row++)
        for (unsigned int col = 0; col < lastImg_.width; col++) {
            idx = (row * lastImg_.width) + col;
            if (floatData[idx] < 1.0) {
            RCLCPP_INFO(this->get_logger(),
                        "row=%d, col=%d, floatData[idx] = %.2f", row, col,
                        floatData[idx]);
            return true;
            }
        }

        return false;
    }
    
    rclcpp::Publisher<TWIST>::SharedPtr vel_pub_;
    double collision_distance_;
    rclcpp::Subscription<IMAGE>::SharedPtr subscription_;
    // std::string cmd_vel_topic = "/cmd_vel";
    rclcpp::TimerBase::SharedPtr timer_;
    IMAGE lastImg_;
    StateType state_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}