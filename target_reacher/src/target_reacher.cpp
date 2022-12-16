#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"
#include <string>

void TargetReacher::spin(){

    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0.2;
    spin_publisher->publish(msg);
}

void TargetReacher::goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg){
   if(msg->data && (flag==1)){
        spin();
   }
}

void TargetReacher::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
    flag = 0;
    if(msg->marker_ids.at(0) == 0){
        // get parameters
        rclcpp::Parameter aruco_0_x = this->get_parameter("final_destination.aruco_0.x");
        rclcpp::Parameter aruco_0_y = this->get_parameter("final_destination.aruco_0.y");
        
        final_x = aruco_0_x.as_double();
        final_y = aruco_0_y.as_double();
    }

    if(msg->marker_ids.at(0) == 1){
        // get parameters
        rclcpp::Parameter aruco_1_x = this->get_parameter("final_destination.aruco_1.x");
        rclcpp::Parameter aruco_1_y = this->get_parameter("final_destination.aruco_1.y");
        
        final_x = aruco_1_x.as_double();
        final_y = aruco_1_y.as_double();
    }

    if(msg->marker_ids.at(0) == 2){
        // get parameters
        rclcpp::Parameter aruco_2_x = this->get_parameter("final_destination.aruco_2.x");
        rclcpp::Parameter aruco_2_y = this->get_parameter("final_destination.aruco_2.y");
        
        final_x = aruco_2_x.as_double();
        final_y = aruco_2_y.as_double();
    }

    if(msg->marker_ids.at(0) == 3){
        // get parameters
        rclcpp::Parameter aruco_3_x = this->get_parameter("final_destination.aruco_3.x");
        rclcpp::Parameter aruco_3_y = this->get_parameter("final_destination.aruco_3.y");
        
        final_x = aruco_3_x.as_double();
        final_y = aruco_3_y.as_double();
    }
    set_goal();
}

void TargetReacher::timer_callback(){
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    
    rclcpp::Parameter origin = this->get_parameter("final_destination.frame_id");
    final_origin = origin.as_string();
    
    tf.header.frame_id = final_origin;
    tf.child_frame_id = "final_destination";
    
    tf.transform.translation.x = final_x;
    tf.transform.translation.y = final_y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    final_destination_broadcaster->sendTransform(tf);
}

void TargetReacher::set_goal(){
    geometry_msgs::msg::TransformStamped tf;
    
    tf = final_destination_buffer->lookupTransform("robot1/odom", "final_destination", tf2::TimePointZero);
    
    m_bot_controller->set_goal(tf.transform.translation.x, tf.transform.translation.y);
}