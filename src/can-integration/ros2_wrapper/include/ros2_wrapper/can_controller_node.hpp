#pragma once 

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "can-utils/system_controller.hpp"
#include "can-utils/can_interface.hpp"
#include <array>
#include <atomic>
#include <memory>
#include <chrono>
#include <mutex>

using namespace std::chrono;
using namespace std::literals::chrono_literals; 
//#define MAX_MOTOR_SPEED 1024.f


class CanControllerNode : public rclcpp::Node{

    public:
        explicit CanControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg);
        void getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg);

    private: 

        void sendCanFrames();

        std::string can_interface_; 
        
        std::shared_ptr<can_util::CANController> can_controller_; 
        std::unique_ptr<SystemFrameBuilder> frame_builder_; 

        ros2_fmt_logger::Logger logger; 
        int multiplier;
        double arm_velocity_scale_; 
        

        // Servo shaping (joystick [-1,1] -> rad or rad/s).
        std::string spin_servo_mode_;
        std::string clamp_servo_mode_;
        float spin_servo_max_rad_{0.0f};
        float spin_servo_max_rad_s_{0.0f};
        float clamp_servo_max_rad_{0.0f};
        float clamp_servo_max_rad_s_{0.0f};

        std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler;
        rclcpp::ParameterCallbackHandle::SharedPtr multiplier_callback_handle;
        rclcpp::ParameterCallbackHandle::SharedPtr arm_velocity_scale_callback_handle;        

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_msgs_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_msgs_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wheel_stopped_sub_;

        // Inhibit flags driven by /can_safety/wheel_stopped (latched). The
        // legacy node initialised these to true so commands flowed by default;
        // we keep that behaviour. The arm flag is no longer toggled by the
        // joystick (the stop logic was already a no-op) but stays around so
        // future safety wiring can re-enable it without touching control flow.
        std::atomic_bool inhibit_arm_cmds_{true};
        std::atomic_bool inhibit_wheel_cmds_{true};

        rclcpp::TimerBase::SharedPtr can_send_timer_; 
        int can_send_rate_hz_{100};

        std::mutex cmd_mutex_;
        geometry_msgs::msg::Twist::ConstSharedPtr latest_twist_;
        sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;
        bool twist_dirty_{false};
        bool joint_state_dirty_{false};

        float wheel_rpm_slew_rate_{0.F};
        std::array<float, 6> wheel_rpm_smoothed_{};
}; 

