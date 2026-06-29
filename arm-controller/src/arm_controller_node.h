#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp" // Updated to use JointState

static constexpr auto SET_MOTOR_SPEED = 0x4E;
static constexpr auto MAX_MOTOR_SPEED = 1024.f;

class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode();
    ~ArmControllerNode() override;

    void armJointCallback(const sensor_msgs::msg::JointState::UniquePtr& msg) const; // Updated callback signature

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_sub; // Updated subscription type

    int fd;
    int controller_type = -1;
};
