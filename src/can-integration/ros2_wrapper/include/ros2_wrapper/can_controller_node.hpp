#pragma once 

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "can-utils/system_controller.hpp"
#include "can-utils/can_interface.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono;
using namespace std::literals::chrono_literals; 
#define MAX_MOTOR_SPEED 1024.f


class CanControllerNode : public rclcpp::Node{

    public:
        explicit CanControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg); 
        void getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg);
        void getjoyfeedback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg); 

    private: 

        std::string can_interface_; 
        
        std::shared_ptr<can_util::CANController> can_controller_; 
        std::unique_ptr<SystemFrameBuilder> frame_builder_; 

        ros2_fmt_logger::Logger logger; 
        int multiplier; 
        

        std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler;
        rclcpp::ParameterCallbackHandle::SharedPtr multiplier_callback_handle;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_msgs_; 
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_msgs_; 
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_msgs_; 

        rclcpp::TimerBase::SharedPtr timer_; 


}; 

