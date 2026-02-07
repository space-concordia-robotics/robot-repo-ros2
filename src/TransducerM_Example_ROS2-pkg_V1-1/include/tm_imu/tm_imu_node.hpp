# pragma once
// Linux library:
#include <stdio.h>
#include "serialib.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS library
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
// ROS Msg
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

// To use the communication library, we need to include the following
// two header files:
#include "EasyObjectDictionary.h" // TransducerM Data
#include "EasyProfile.h"          // TransducerM communication protocol

using namespace std::chrono_literals;

// Debug switch:
//#define  DEBUG_MODE

class TMSerial : public rclcpp::Node
{
public:
  TMSerial();
  ~TMSerial();
  
private:
  void TimerCallback();
  char SerialportOpen();
  bool OnSerialRX();
  void FillCovarianceMatrices();
  void PublishTransform();
  #ifdef DEBUG_MODE
  rclcpp::TimerBase::SharedPtr timer_10;
  void    TimerCallback2();
  int     count;
  int     count2;
  #endif

private:
  serialib* serialib1;  // We use linux serialib to interface with serial port.

  sensor_msgs::msg::Imu imu_data_msg;
  sensor_msgs::msg::MagneticField imu_data_rpy_msg; // RPY msg uses the same data structure as msg::MagneticField
  sensor_msgs::msg::MagneticField imu_data_mag_msg;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_IMU;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_IMU_RPY;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_IMU_MAG;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

