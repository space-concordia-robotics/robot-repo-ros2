#pragma once


#include <assert.h>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <memory>
#include <thread>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>


// if stop command is sent from a board, have a heartbeat thread listen for a continous heartbeat. 

inline bool decodeFrame(ssize_t CANID); 