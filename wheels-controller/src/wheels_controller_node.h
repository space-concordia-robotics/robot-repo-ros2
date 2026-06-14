#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono;

static constexpr auto DEVICE_1_ID = 1;
static constexpr auto DEVICE_2_ID = 2;
static constexpr auto DEVICE_3_ID = 3;
static constexpr auto DEVICE_4_ID = 4;
static constexpr auto DEVICE_5_ID = 5;
static constexpr auto DEVICE_6_ID = 6;

class WheelsControllerNode : public rclcpp::Node {
public:
    WheelsControllerNode();
    void TwistMessageCallback(const geometry_msgs::msg::Twist::UniquePtr& twist_msg) const;

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_msg_callback;
    std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler;
    rclcpp::ParameterCallbackHandle::SharedPtr multiplier_callback_handle;

    int multiplier;
};
