#pragma once

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <string>

class OdomBroadcaster : public rclcpp::Node
{
public:
    OdomBroadcaster(std::string node_name) : Node(node_name)
    {
        // Initialize the transform broadcaster
        tf_broadcaster =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create a timer
        timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / 1.0)),
                                          std::bind(&OdomBroadcaster::timer_callback, this));

        std::ostringstream stream;
        stream << "/robot1/odom";
        std::string topic_name = stream.str();

        subscription_ = this->create_subscription<nav_msg::msg::Odometry::pose>(
        topic_name, 10,
        std::bind(&FramePublisher::odom, this, std::placeholders::_1));
    }

private:
    // attributes
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // methods
    void broadcast_odom(const std::shared_ptr<turtlesim::msg::Pose> msg);
    void timer_callback();
};