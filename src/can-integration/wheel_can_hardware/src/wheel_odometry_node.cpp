#include "wheel_can_hardware/wheel_odometry_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>

namespace wheel_can_hardware
{

WheelOdometryNode::WheelOdometryNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("wheel_odometry_node", options),
      last_stamp_(0, 0, RCL_ROS_TIME)
{
  // Defaults match rover_wheels.ros2_control.xacro; can be overridden in a
  // launch file or YAML for vehicles with different wheel naming.
  right_joints_ = this->declare_parameter<std::vector<std::string>>(
    "right_joints", std::vector<std::string>{"wheel_right_front", "wheel_right_mid", "wheel_right_rear"});
  left_joints_ = this->declare_parameter<std::vector<std::string>>(
    "left_joints", std::vector<std::string>{"wheel_left_front", "wheel_left_mid", "wheel_left_rear"});

  wheel_radius_ = this->declare_parameter<double>("wheel_radius", wheel_radius_);
  track_width_  = this->declare_parameter<double>("track_width", track_width_);
  odom_frame_   = this->declare_parameter<std::string>("odom_frame", odom_frame_);
  base_frame_   = this->declare_parameter<std::string>("base_frame", base_frame_);
  publish_tf_   = this->declare_parameter<bool>("publish_tf", publish_tf_);
  max_dt_s_     = this->declare_parameter<double>("max_dt_s", max_dt_s_);

  if (track_width_ <= 0.0 || wheel_radius_ <= 0.0) {
    RCLCPP_FATAL(this->get_logger(),
                 "wheel_radius (%.4f) and track_width (%.4f) must be positive",
                 wheel_radius_, track_width_);
    throw std::runtime_error("invalid odometry geometry");
  }

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

  js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) { onJointStates(msg); });

  RCLCPP_INFO(this->get_logger(),
              "wheel_odometry_node ready -- wheel_radius=%.3fm, track_width=%.3fm, "
              "odom_frame='%s', base_frame='%s', publish_tf=%s",
              wheel_radius_, track_width_, odom_frame_.c_str(), base_frame_.c_str(),
              publish_tf_ ? "true" : "false");
}

bool WheelOdometryNode::extractAverageVelocity(const sensor_msgs::msg::JointState & msg,
                                               const std::vector<std::string> & wanted,
                                               double & out) const
{
  if (msg.name.size() != msg.velocity.size()) {
    return false;
  }
  double sum = 0.0;
  size_t found = 0;
  for (const auto & target : wanted) {
    for (size_t i = 0; i < msg.name.size(); ++i) {
      if (msg.name[i] == target) {
        sum += msg.velocity[i];
        ++found;
        break;
      }
    }
  }
  if (found == 0) {
    return false;
  }
  out = sum / static_cast<double>(found);
  // Note: we report success even with partial coverage so that one missing
  // wheel doesn't black out the whole odometry stream. The caller logs it.
  if (found < wanted.size()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "joint_state missing %zu of %zu wheels for one side",
                         wanted.size() - found, wanted.size());
  }
  return true;
}

void WheelOdometryNode::onJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
{
  double v_left_radps  = 0.0;
  double v_right_radps = 0.0;
  if (!extractAverageVelocity(*msg, left_joints_, v_left_radps) ||
      !extractAverageVelocity(*msg, right_joints_, v_right_radps)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Unable to compute odometry: missing wheel velocities");
    return;
  }

  // Convert wheel angular velocity (rad/s) to ground-plane linear velocity
  // (m/s) at each side. Sign convention: positive joint velocity means the
  // wheel rotates such that the rover moves forward (the WheelCanInterface
  // already inverts the right side, so both sides positive = pure forward).
  const double v_left  = v_left_radps  * wheel_radius_;
  const double v_right = v_right_radps * wheel_radius_;
  const double linear_x   = 0.5 * (v_right + v_left);
  const double angular_z  = (v_right - v_left) / track_width_;  // positive = CCW (left turn)

  const rclcpp::Time stamp = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
                             ? this->now()
                             : rclcpp::Time(msg->header.stamp);

  if (!have_last_stamp_) {
    last_stamp_ = stamp;
    have_last_stamp_ = true;
  }

  double dt = (stamp - last_stamp_).seconds();
  if (dt < 0.0 || dt > max_dt_s_) {
    // Reset the integrator on bag jumps / time discontinuities.
    dt = 0.0;
  }
  last_stamp_ = stamp;

  // Mid-step integration: project the heading half a step forward so the
  // straight-line motion is closer to the true arc when angular_z != 0.
  const double mid_theta = theta_ + 0.5 * angular_z * dt;
  x_     += linear_x * std::cos(mid_theta) * dt;
  y_     += linear_x * std::sin(mid_theta) * dt;
  theta_ += angular_z * dt;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id  = base_frame_;
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = linear_x;
  odom.twist.twist.angular.z = angular_z;
  odom_pub_->publish(odom);

  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id  = base_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }
}

}  // namespace wheel_can_hardware

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_can_hardware::WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
