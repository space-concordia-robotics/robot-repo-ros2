#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "control_msgs/msg/joint_jog.hpp"


class GoalTwistPublisher: public rclcpp::Node
{
public:
    GoalTwistPublisher(): Node("vel_goal_control_node")
    {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("goal_twist", 10);
        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>("joint_states", 10); 
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10, std::bind(&GoalTwistPublisher::PublishEEVelocityCommands, this, std::placeholders::_1));
        joint_sub_ = this->create_subscription<control_msgs::msg::JointJog>("/cmd_vel", 10, std::bind(&GoalTwistPublisher::PublishJointCommands, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "GoalTwistPublisher node started");
    }
  
    
private:

void PublishJointCommands(const control_msgs::msg::JointJog::SharedPtr msg)
{
    auto joint_msg = control_msgs::msg::JointJog();
    joint_msg.header.stamp = this->now();
    joint_msg.header.frame_id = "base_structure_link";
    joint_msg.joint_names = {
        "base_structure_link", 
        "base_pivot_shoulder_gearbox_joint", 
        "shoulder_tube_joint", 
        "bicep_tube_gearbox_joint",
        "bicep_gearbox_forearm_tube_joint", 
        "forearm_tube_wrist_gearbox_joint",
        "wrist_gearbox_wrist_joint", 
        "gripper_claw_joint"
    };

    joint_msg.velocities = { 
        msg->velocities[0], 
        msg->velocities[1], 
        msg->velocities[2],
        msg->velocities[3], 
        msg->velocities[4], 
        msg->velocities[5], 
        msg->velocities[6], 
        msg->velocities[7]
    };

    joint_pub_->publish(joint_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing joint velocity commands to arm:" "[%f, %f, %f, %f, %f, %f, %f, %f]", 
        joint_msg.velocities[0], joint_msg.velocities[1], joint_msg.velocities[2], 
        joint_msg.velocities[3], joint_msg.velocities[4], joint_msg.velocities[5],
        joint_msg.velocities[6], joint_msg.velocities[7]);
} 

void PublishEEVelocityCommands(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_structure_link";
    twist_msg.twist.linear.x = msg->twist.linear.x;
    twist_msg.twist.linear.y = msg->twist.linear.y;
    twist_msg.twist.linear.z = msg->twist.linear.z;
    twist_msg.twist.angular.x = msg->twist.angular.x;
    twist_msg.twist.angular.y = msg->twist.angular.y;
    twist_msg.twist.angular.z = msg->twist.angular.z;
    
    twist_pub_->publish(twist_msg); 

    RCLCPP_INFO(this->get_logger(), "Publishing linear velocity commands to end effector:" "[%f, %f, %f]", 
        twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);    
    RCLCPP_INFO(this->get_logger(), "Publishing angular velocity commands to end effector:" "[%f, %f, %f]",
        twist_msg.twist.angular.x, twist_msg.twist.angular.y, twist_msg.twist.angular.z);

}

rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_sub_;
rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalTwistPublisher>(); 
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    //rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
