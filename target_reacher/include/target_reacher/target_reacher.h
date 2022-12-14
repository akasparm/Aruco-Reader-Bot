#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/transform_broadcaster.h"

// timer
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {

        m_bot_controller = bot_controller;
        auto aruco_target_x = this->declare_parameter<double>("aruco_target.x");
        auto aruco_target_y = this->declare_parameter<double>("aruco_target.y");
        auto aruco_0_x = this->declare_parameter<double>("final_destination.aruco_0.x");
        auto aruco_0_y = this->declare_parameter<double>("final_destination.aruco_0.y");
        auto aruco_1_x = this->declare_parameter<double>("final_destination.aruco_1.x");
        auto aruco_1_y = this->declare_parameter<double>("final_destination.aruco_1.y");
        auto aruco_2_x = this->declare_parameter<double>("final_destination.aruco_2.x");
        auto aruco_2_y = this->declare_parameter<double>("final_destination.aruco_2.y");
        auto aruco_3_x = this->declare_parameter<double>("final_destination.aruco_3.x");
        auto aruco_3_y = this->declare_parameter<double>("final_destination.aruco_3.y");

        m_bot_controller->set_goal(aruco_0_x, aruco_0_y);

    }
    
    void spin();
    void find_marker();
    
    

// TODO
// listen to output of set_goal (subscribe to goal_reached/data)
// spin at location (publish to cmd_vel/angular_velocity and cmd_vel/linear_velocity)
// find marker (if marker found read id (id is a vector use vector.begin()) and call set_goal())
// listen to output of set_goal (subscribe to goal_reached/data)
// find marker (if marker found read id (id is a vector use vector.begin()) and call set_goal())
// listen to output of set_goal (subscribe to goal_reached/data)

private:
    // attributes
    std::shared_ptr<BotController> m_bot_controller;
};