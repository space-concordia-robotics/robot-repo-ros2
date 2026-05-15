#include "can_safety_node/can_safety_node.hpp"

#include "can-utils/prefixes.hpp"

namespace can_safety_node
{

CanSafetyNode::CanSafetyNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("can_safety_node", options)
{
  can_interface_name_       = this->declare_parameter<std::string>("can_interface", can_interface_name_);
  wheel_force_stop_button_  = this->declare_parameter<int>("wheel_force_stop_button", wheel_force_stop_button_);
  wheel_resume_button_      = this->declare_parameter<int>("wheel_resume_button",     wheel_resume_button_);

  can_ = std::make_shared<can_util::CANController>(can_interface_name_, this->get_logger());
  if (!can_->configureCan()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to configure CAN on '%s'", can_interface_name_.c_str());
    throw std::runtime_error("CAN configure failed");
  }
  frame_builder_ = std::make_unique<SystemFrameBuilder>(can_);

  // Latched QoS so late subscribers see the current state without a tick.
  auto latched = rclcpp::QoS(1).reliable().transient_local();
  wheel_stopped_pub_ = this->create_publisher<std_msgs::msg::Bool>("can_safety/wheel_stopped", latched);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::Joy::ConstSharedPtr msg) { onJoy(msg); });

  // Publish the initial (un-stopped) state so /can_safety/wheel_stopped is
  // never empty for downstream consumers.
  std_msgs::msg::Bool msg;
  msg.data = wheel_stopped_;
  wheel_stopped_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
              "can_safety_node ready -- can_interface='%s', force_stop_button=%d, resume_button=%d",
              can_interface_name_.c_str(), wheel_force_stop_button_, wheel_resume_button_);
}

bool CanSafetyNode::risingEdge(bool current, bool & prev)
{
  const bool fired = current && !prev;
  prev = current;
  return fired;
}

void CanSafetyNode::onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
{
  const auto needed = static_cast<size_t>(std::max(wheel_force_stop_button_, wheel_resume_button_)) + 1;
  if (msg->buttons.size() < needed) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Joy message has %zu buttons; need at least %zu for safety mappings",
                         msg->buttons.size(), needed);
    return;
  }

  const bool fs_pressed = msg->buttons[wheel_force_stop_button_] == 1;
  const bool rs_pressed = msg->buttons[wheel_resume_button_]     == 1;

  const bool fs_edge = risingEdge(fs_pressed, prev_force_stop_pressed_);
  const bool rs_edge = risingEdge(rs_pressed, prev_resume_pressed_);

  if (fs_edge && !wheel_stopped_) {
    wheel_stopped_ = true;
    frame_builder_->sendForceStop(deviceType::DeviceType::RELAY, DeviceId::ID::HUB);
    std_msgs::msg::Bool out;
    out.data = true;
    wheel_stopped_pub_->publish(out);
    RCLCPP_INFO(this->get_logger(), "Wheel force-stop CAN frame sent");
  } else if (rs_edge && wheel_stopped_) {
    wheel_stopped_ = false;
    frame_builder_->sendResume(deviceType::DeviceType::RELAY, DeviceId::ID::HUB);
    std_msgs::msg::Bool out;
    out.data = false;
    wheel_stopped_pub_->publish(out);
    RCLCPP_INFO(this->get_logger(), "Wheel resume CAN frame sent");
  }
}

}  // namespace can_safety_node

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<can_safety_node::CanSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
