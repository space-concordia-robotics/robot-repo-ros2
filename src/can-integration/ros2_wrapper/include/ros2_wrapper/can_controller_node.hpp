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
#include <mutex>

using namespace std::chrono;
using namespace std::literals::chrono_literals; 

class CanControllerNode : public rclcpp::Node{

    public:
        explicit CanControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg); 
        void getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg);
        void getjoyfeedback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg); 

    private: 

        void sendCanFrames();

        std::string can_interface_; 
        
        std::shared_ptr<can_util::CANController> can_controller_; 
        std::unique_ptr<SystemFrameBuilder> frame_builder_; 

        ros2_fmt_logger::Logger logger; 
        int multiplier; 
        double arm_velocity_scale_;
        int can_send_rate_hz_;

        std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler;
        rclcpp::ParameterCallbackHandle::SharedPtr multiplier_callback_handle;
        rclcpp::ParameterCallbackHandle::SharedPtr arm_velocity_scale_callback_handle_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_msgs_; 
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_msgs_; 
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_msgs_; 

        rclcpp::TimerBase::SharedPtr can_send_timer_; 

        std::mutex cmd_mutex_;
        geometry_msgs::msg::Twist::ConstSharedPtr latest_twist_;
        sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;
        bool twist_dirty_{false};
        bool joint_state_dirty_{false};
}; 

