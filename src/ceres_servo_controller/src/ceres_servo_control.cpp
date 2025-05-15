#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


class GoalTwistPublisher: public rclcpp::Node
{
public:
    GoalTwistPublisher(): Node("vel_goal_control_node")
    {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("goal_twist", 10);
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10, std::bind(&GoalTwistPublisher::PublishVelocityCommands, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "GoalTwistPublisher node started");
    }
  
    
private:

void PublishVelocityCommands(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
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

    RCLCPP_INFO(this->get_logger(), "Publishing linear velocity commands to arm:" "[%f, %f, %f]", 
        twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);    
    RCLCPP_INFO(this->get_logger(), "Publishing angular velocity commands to arm:" "[%f, %f, %f]",
        twist_msg.twist.angular.x, twist_msg.twist.angular.y, twist_msg.twist.angular.z);

}


rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
rclcpp::TimerBase::SharedPtr timer_;


};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalTwistPublisher>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}