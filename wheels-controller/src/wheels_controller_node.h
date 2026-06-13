#ifndef WHEELS_CONTROLLER_NODE_H
#define WHEELS_CONTROLLER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono;

#define DEVICE_1_ID 1
#define DEVICE_2_ID 2
#define DEVICE_3_ID 3
#define DEVICE_4_ID 4
#define DEVICE_5_ID 5
#define DEVICE_6_ID 6

using namespace std::literals::chrono_literals;

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

#endif
