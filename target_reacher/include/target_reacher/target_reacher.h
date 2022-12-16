#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <std_msgs/msg/bool.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// timer
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {

        m_bot_controller = bot_controller;

        auto aruco_target_x = this->declare_parameter<double>("aruco_target.x");
        auto aruco_target_y = this->declare_parameter<double>("aruco_target.y");

        this->declare_parameter<std::string>("final_destination.frame_id");

        this->declare_parameter<double>("final_destination.aruco_0.x");
        this->declare_parameter<double>("final_destination.aruco_0.y");
        this->declare_parameter<double>("final_destination.aruco_1.x");
        this->declare_parameter<double>("final_destination.aruco_1.y");
        this->declare_parameter<double>("final_destination.aruco_2.x");
        this->declare_parameter<double>("final_destination.aruco_2.y");
        this->declare_parameter<double>("final_destination.aruco_3.x");
        this->declare_parameter<double>("final_destination.aruco_3.y");

        m_bot_controller->set_goal(aruco_target_x, aruco_target_y);

        // subscribes to topic /goal_reached
        goal_reached_subscriber = this->create_subscription<std_msgs::msg::Bool>("/goal_reached", 10, std::bind(&TargetReacher::goal_reached_callback, this, std::placeholders::_1));
        
        // publishes to topic /robot1/odom
        spin_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        // subscriber to topic /aruco_marker
        aruco_subscriber = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&TargetReacher::aruco_callback, this, std::placeholders::_1));


        // final transform
        final_destination_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
       
        broadcast_timer = this->create_wall_timer(std::chrono::milliseconds((int)(100.0)), std::bind(&TargetReacher::broadcast_timer_callback, this));
        
        final_destination_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
       
        final_destination_listener = std::make_shared<tf2_ros::TransformListener>(*final_destination_buffer);

    }

    void spin();
    
    void goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg);
    
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    void set_goal();

    void timer_callback();

private:
    // attributes
    std::shared_ptr<BotController> m_bot_controller;

    double final_x{0};
    double final_y{0};

    int flag{1};

    std::string final_origin;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_subscriber;
    
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr spin_publisher;

    //final transform
    rclcpp::TimerBase::SharedPtr broadcast_timer;
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> final_destination_broadcaster;
    
    std::shared_ptr<tf2_ros::TransformListener> final_destination_listener;
    
    std::unique_ptr<tf2_ros::Buffer> final_destination_buffer;
    
};

