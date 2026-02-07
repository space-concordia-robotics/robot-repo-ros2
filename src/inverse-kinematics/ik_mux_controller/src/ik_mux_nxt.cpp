
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string WHEEL_VEL_TOPIC = "/cmd_vel";
const std::string EEF_FRAME_ID = "gripper_claw_link";
const std::string BASE_FRAME_ID = "base_structure_link";

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
   X = 0,
   Y = 1,
   Z = 5,
   A1_UP = 9,
   A1_SIDE = 8

};
enum Button
{
  DEADMAN = 0,
  A2 = 2,
  PINKY_BUTTON = 4,
  THMB_HAT_FWD = 15, 
  THMB_HAT_LEFT = 18, 
  THMB_HAT_RIGHT = 16,
  THMB_HAT_BCK = 17, 
  A3_FWD = 5, 
  A3_LEFT = 8,
  A3_RIGHT = 6,
  A3_BCK = 7, 
  A4_FWD = 10,
  A4_LEFT = 13,
  A4_RIGHT = 11,
  A4_BCK = 12,
};

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
  // Give joint jogging priority because it is only buttons
  // If any joint jog command is requested, we are only publishing joint commands
  if (buttons[THMB_HAT_FWD] || buttons[THMB_HAT_LEFT] || buttons[THMB_HAT_RIGHT] || buttons[THMB_HAT_BCK] ||
      buttons[A3_FWD] || buttons[A3_LEFT] || buttons[A3_RIGHT] || buttons[A3_BCK] ||
      buttons[A4_FWD] || buttons[A4_LEFT] || buttons[A4_RIGHT] || buttons[A4_BCK])
  {
    joint->joint_names.push_back("joint1");
    joint->velocities.push_back(buttons[THMB_HAT_BCK] - buttons[THMB_HAT_FWD]);
    joint->joint_names.push_back("joint2");
    joint->velocities.push_back(buttons[THMB_HAT_LEFT] - buttons[THMB_HAT_RIGHT]);

    joint->joint_names.push_back("joint3");
    joint->velocities.push_back(buttons[A3_BCK] - buttons[A3_FWD]);
    joint->joint_names.push_back("joint5");
    joint->velocities.push_back(buttons[A4_RIGHT] - buttons[A4_LEFT]);
    // joint->joint_names.push_back("joint7");
    // joint->velocities.push_back(buttons[LEFT_STICK_CLICK] - buttons[RIGHT_STICK_CLICK]);
    return false;
  }

  // The bread and butter: map buttons to twist commands
  twist->twist.linear.x = axes[X];
  twist->twist.linear.y = axes[Y];
  twist->twist.linear.z = axes[A1_UP];
  twist->twist.angular.z = axes[Z]; 


  return true;
}


namespace moveit_servo
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
  {
    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    collision_pub_ =
        this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(this->get_logger(), "Ik mux node publishing topic data");

  }

  ~JoyToServoPub() override
  {
    if (collision_pub_thread_.joinable())
      collision_pub_thread_.join();
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
   
    // Check deadman button state
    bool deadman_pressed = msg->buttons[DEADMAN];
    if (deadman_pressed != deadman_)
    {
        deadman_ = deadman_pressed;
        RCLCPP_INFO(this->get_logger(), deadman_ ? "Deadman Active" : "Deadman Released");
    }

    if (!deadman_)
    {
      return;
    }

    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
    // publish the TwistStamped
    twist_msg->header.frame_id = frame_to_publish_;
    twist_msg->header.stamp = this->now();
    twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
    // publish the JointJog
    joint_msg->header.stamp = this->now();
    joint_msg->header.frame_id = "base_structure_link";
    joint_msg->duration = 1.0;
    joint_pub_->publish(std::move(joint_msg));
    }

  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_pub_; 
  std::string frame_to_publish_;

  std::thread collision_pub_thread_;
  bool deadman_ = false; 
};  // class JoyToServoPub

}  // namespace moveit_servo

// Register the component with class_loader
int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<moveit_servo::JoyToServoPub>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}