#ifndef ARM_CONTROLLER_NODE_H
#define ARM_CONTROLLER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

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

#define SET_MOTOR_SPEED 0x4E
#define MAX_MOTOR_SPEED 1024.f

class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode();
    ~ArmControllerNode();

    void ArmXYZCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr arm_xyz_sub;

    int fd;
    int controller_type = -1;
};

#endif
