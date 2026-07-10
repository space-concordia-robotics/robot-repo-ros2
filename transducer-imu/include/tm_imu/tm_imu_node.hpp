#pragma once

#include <memory>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "serialib/serialib.hpp"

// To use the communication library, we need to include the following
// two header files:

using namespace std::chrono_literals;

// Debug switch:
//#define  DEBUG_MODE

class TMSerial : public rclcpp::Node {
public:
    TMSerial();
    ~TMSerial() override;

private:
    void TimerCallback();
    char SerialportOpen() const;
    bool OnSerialRX();
    void FillCovarianceMatrices();
    void PublishTransform();
    #ifdef DEBUG_MODE
    rclcpp::TimerBase::SharedPtr timer_10;
    void TimerCallback2();
    int count;
    int count2;
    #endif

    serialib* serialib1; // We use linux serialib to interface with serial port.

    sensor_msgs::msg::Imu imu_data_msg;
    sensor_msgs::msg::MagneticField imu_data_rpy_msg; // RPY msg uses the same data structure as msg::MagneticField
    sensor_msgs::msg::MagneticField imu_data_mag_msg;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_IMU;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_IMU_RPY;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_IMU_MAG;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    ros2_fmt_logger::Logger logger;
};
