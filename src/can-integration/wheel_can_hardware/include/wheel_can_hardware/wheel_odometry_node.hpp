// wheel_odometry_node.hpp
//
// Subscribes to the /joint_states stream produced by joint_state_broadcaster
// (fed by WheelCanInterface) and integrates differential-drive odometry into
// nav_msgs/Odometry plus the standard odom -> base_link TF.
//
// The rover has six wheels arranged in a rocker-bogey: rear wheels turn
// independently while the middle and front on each side share a bogey. We
// take the unweighted mean of the three per-side wheel velocities, which is
// robust to a single slipping wheel and matches the plan's recommendation
// for the most general case.

#pragma once

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <memory>
#include <string>
#include <vector>

namespace wheel_can_hardware
{

class WheelOdometryNode : public rclcpp::Node
{
public:
  explicit WheelOdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr & msg);

  // Pull the velocity (rad/s) for the named joints out of the joint_state
  // message. Returns std::nullopt if any expected joint is missing.
  // Filled into `out`, returns true on success.
  bool extractAverageVelocity(const sensor_msgs::msg::JointState & msg,
                              const std::vector<std::string> & wanted,
                              double & out) const;

  // Parameters
  std::vector<std::string> right_joints_;
  std::vector<std::string> left_joints_;
  // Match rover-description (base.urdf) and diff_drive_base_controller on autonomy.
  double wheel_radius_{0.15};
  double track_width_{1.25};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  bool publish_tf_{true};
  // Cap to avoid huge integration steps after a callback gap (e.g. start-up).
  double max_dt_s_{0.5};

  // Integrated pose
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
  bool have_last_stamp_{false};
  rclcpp::Time last_stamp_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace wheel_can_hardware
