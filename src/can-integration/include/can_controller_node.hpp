#pragma once 

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "system_controller.hpp"
#include "can_interface.hpp"

#include <chrono>

using namespace std::chrono;
using namespace std::literals::chrono_literals; 


class CanControllerNode : public rclcpp::Node{

    public:
        CanControllerNode() : Node("can_controller_node") {

            //check to see if can has been sucessfully configured 
            if(CanManager::configureCan("can0") != SUCCESS){
                RCLCPP_ERROR(this->get_logger(), "Failed to configure CAN interface...");
                rclcpp::shutdown();
                return; 
            }
            RCLCPP_INFO(this->get_logger(), "CAN interface configured successfully!");


            //intiallize subscriptions 

            // twist_msgs_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS(), [this]
            // (geometry_msgs::msg::Twist::ConstSharedPtr& msg) {return getTwistMessages(msg); });
            joint_state_msgs_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm_xyz_cmd", rclcpp::SensorDataQoS(), [this]
            (const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {return getJointStateMessages(msg); });
            joy_msgs_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::SystemDefaultsQoS(), [this]
            (const sensor_msgs::msg::Joy::ConstSharedPtr& msg){return getjoyfeedback(msg); });

            RCLCPP_INFO(this->get_logger(), "Subscribed to the required topics");

            //initialize can start up function call. 

        }

        // void getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg); 
        void getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg);
        void getjoyfeedback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg); 

    private: 

        SystemFrameBuilder systemframebuilder_;
        
        // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_msgs_; 
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_msgs_; 
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_msgs_; 

        rclcpp::TimerBase::SharedPtr timer_; 


}; 

