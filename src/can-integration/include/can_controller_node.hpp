#pragma once 

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>

using namespace std::chrono;
using namespace std::literals::chrono_literals; 


class CanControllerNode : public rclcpp::Node{

    public:
        CanControllerNode() : Node("can_controller_node") {

            //check to see if can has been sucessfully configured 



            //intiallize subscriptions 

                twist_msgs_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS(), [this]
                (geometry_msgs::msg::Twist::ConstSharedPtr& msg) {return getTwistMessages(msg); });

                joint_state_msgs_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm_xyz_cmd", rclcpp::SensorDataQoS(), [this]
                (sensor_msgs::msg::JointState::ConstSharedPtr& msg) {return getJointStateMessages(msg); });

            //initialize can start up function call. 

        }

        void getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr twist_msg); 
        void getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state_msg);

    private: 

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_msgs_; 
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_msgs_; 

        rclcpp::TimerBase::SharedPtr timer_; 


}; 

