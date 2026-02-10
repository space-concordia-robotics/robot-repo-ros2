#ifndef ARM_CONTROLLER_NODE_H
#define ARM_CONTROLLER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp" // Updated to use JointState

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <iostream>
#include <fcntl.h>

class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode();
    ~ArmControllerNode();

    void ArmJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg); // Updated callback signature

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_sub; // Updated subscription type

    int controller_type = -1;
};

#endif
