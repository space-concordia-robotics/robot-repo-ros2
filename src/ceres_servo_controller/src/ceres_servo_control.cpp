#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class VelocityPublisherNode: public rclcpp::Node
{
public:
    VelocityPublisherNode(): Node("vel_goal_control_node")
    {

        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);
        joint_angles_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/servo_node/delta_joint_angles", 10); 
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&VelocityPublisherNode::PublishJointAngles, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&VelocityPublisherNode::PublishJointVelocities, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "GoalTwistPublisher node started");
    }
  
    
private:

void PublishJointVelocities(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    auto joint_msgs = std::make_unique<control_msgs::msg::JointJog>();
    joint_msgs->header.stamp = this->now();
    joint_msgs->header.frame_id = "base_structure_link";        
    joint_msgs->joint_names = {
            "base_structure_link",
            "base_pivot_shoulder_gearbox_link",
            "bicep_tube_gearbox_link",
            "forearm_tube_wrist_gearbox_link",
            "gripper_claw_link"
        };

    joint_msgs->velocities.push_back(msg->axes.size() > 0 ? msg->axes[0] : 0.0);
    joint_msgs->velocities.push_back(msg->axes.size() > 1 ? msg->axes[1] : 0.0);
    joint_msgs->velocities.push_back(msg->axes.size() > 2 ? msg->axes[2] : 0.0);
    joint_msgs->velocities.push_back(msg->axes.size() > 3 ? msg->axes[3] : 0.0);
    joint_msgs->velocities.push_back(msg->axes.size() > 4 ? msg->axes[4] : 0.0);


     RCLCPP_INFO(this->get_logger(), "Publishing joint velocities");
    RCLCPP_INFO(this->get_logger(), "Joint velocities: %f, %f, %f, %f, %f", 
        joint_msgs->velocities[0], joint_msgs->velocities[1], joint_msgs->velocities[2], 
        joint_msgs->velocities[3], joint_msgs->velocities[4]);

    joint_pub_->publish(std::move(joint_msgs));
}

void PublishJointAngles(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    auto joint_angles_ = std::make_unique<sensor_msgs::msg::JointState>(); 
    joint_angles_->header.stamp = this->now();
    joint_angles_->header.frame_id = "base_structure_link";
    joint_angles_->name = {
            "base_structure_link",
            "base_pivot_shoulder_gearbox_link",
            "bicep_tube_gearbox_link",
            "forearm_tube_wrist_gearbox_link",
            "gripper_claw_link"
        };

    joint_angles_->position.push_back(msg->position.size() > 0 ? msg->position[0] : 0.0);
    joint_angles_->position.push_back(msg->position.size() > 1 ? msg->position[1] : 0.0);
    joint_angles_->position.push_back(msg->position.size() > 2 ? msg->position[2] : 0.0);
    joint_angles_->position.push_back(msg->position.size() > 3 ? msg->position[3] : 0.0);
    joint_angles_->position.push_back(msg->position.size() > 4 ? msg->position[4] : 0.0);
    RCLCPP_INFO(this->get_logger(), "Publishing joint angles");
    RCLCPP_INFO(this->get_logger(), "joint angles: %f, %f, %f, %f, %f",
        joint_angles_->position[0], joint_angles_->position[1], joint_angles_->position[2], 
        joint_angles_->position[3], joint_angles_->position[4]);

    joint_angles_pub_->publish(std::move(joint_angles_));    
    

}


rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_; 
rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_angles_pub_;


};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityPublisherNode>(); 
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    //rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}