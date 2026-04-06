#pragma once 

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "system_controller.hpp"
#include "can_interface.hpp"
#include "rev_motor_controller.h"

#include <chrono>

using namespace std::chrono;
using namespace std::literals::chrono_literals; 


class CanControllerNode : public rclcpp::Node{

    public:
        CanControllerNode() : Node("can_controller_node") {

            multiplier = this->declare_parameter("multiplier", 500);

            auto can_controller = can_util::CANController::make_shared("vcan0", get_logger());

            if (const auto status = can_controller->initialize(); !status) {
                RCLCPP_ERROR(this->get_logger(), "Failed to configure CAN interface...");
                rclcpp::shutdown();
                return; 
            }

            RCLCPP_INFO(this->get_logger(), "CAN interface configured successfully!");


            //intiallize subscriptions 
            // twist_msgs_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS(), [this]
            // (const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {return getTwistMessages(msg); });
            RCLCPP_INFO(this->get_logger(), "CAN interface configured successfully!");
            joint_state_msgs_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm_xyz_cmd", rclcpp::SensorDataQoS(), [this]
            (const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
                RCLCPP_INFO(this->get_logger(), "hello"); 
                getJointStateMessages(msg);
            });

            joy_msgs_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::SystemDefaultsQoS(),
            [this] (const sensor_msgs::msg::Joy::ConstSharedPtr& msg){
                getjoyfeedback(msg);
            });

            // parameter_event_handler = std::make_shared<rclcpp::ParameterEventHandler>(this);

            // auto multiplier_callback = [this](const rclcpp::Parameter parameter) {
            //     if (parameter.get_name() == "multiplier")
            //         multiplier = parameter.as_int();
            // };

            // multiplier_callback_handle = parameter_event_handler->add_parameter_callback("multiplier", multiplier_callback);

            RCLCPP_INFO(this->get_logger(), "Subscribed to the required topics");

            //initialize can start up function call. 
            // Start the motors
            // uint64_t mask = 0x7E;
            // RevMotorController::startMotor(mask);

            // RCLCPP_INFO(this->get_logger(), "Movement controller initialized.");

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
        
        std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler;
        rclcpp::ParameterCallbackHandle::SharedPtr multiplier_callback_handle;
        int multiplier;

}; 

