#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"


class JoyMuxNode : public rclcpp::Node
{

    public: 

        JoyMuxNode() : Node("joy_mux_controller")
        {
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyMuxNode::JoyToCmdCB, this, std::placeholders::_1));
            twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_xyz_cmd", 10);
        }

    private:

        enum Axis
        {
            LEFT_STICK_X = 0,
            LEFT_STICK_Y = 1,
            LEFT_TRIGGER = 2,
            RIGHT_STICK_X = 3,
            RIGHT_STICK_Y = 4,
            RIGHT_TRIGGER = 5,
            D_PAD_X = 6,
            D_PAD_Y = 7
        };
        enum Button
        {
            square= 0,
            triangle = 1,
            X = 2,
            circle = 3,
            LEFT_BUMPER = 4,
            RIGHT_BUMPER = 5,
            CHANGE_VIEW = 6,
            MENU = 7,
            HOME = 10,
            LEFT_STICK_CLICK = 9,
            RIGHT_STICK_CLICK = 12
        };
    void JoyToCmdCB(const sensor_msgs::msg::Joy::SharedPtr joy_cmds)
    {
        static bool last_deadman_state = false;
        bool current_deadman_state = joy_cmds->buttons[HOME];

        // Toggle mode on button press (edge detection)
        if (current_deadman_state && !last_deadman_state) {
            publish_twist_mode_ = !publish_twist_mode_;
            RCLCPP_INFO(this->get_logger(), "Rover mode Switched to %s", publish_twist_mode_? "Rover" : "Arm");
        }
        last_deadman_state = current_deadman_state;

        if(publish_twist_mode_)
        {
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = joy_cmds->axes[LEFT_STICK_X];
            twist.linear.y = joy_cmds->axes[LEFT_STICK_Y];
            twist.linear.z = joy_cmds->axes[LEFT_TRIGGER - 1];
            twist.angular.z = joy_cmds->axes[RIGHT_TRIGGER - 1];
            twist_pub_->publish(twist);
        }
        else
        {
            auto joint = sensor_msgs::msg::JointState();
            joint.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
            joint.velocity = {
                joy_cmds->axes[D_PAD_X],
                joy_cmds->axes[D_PAD_Y],
                joy_cmds->axes[LEFT_STICK_X],
                joy_cmds->axes[LEFT_STICK_Y],
                joy_cmds->axes[LEFT_TRIGGER - 1],
                joy_cmds->axes[RIGHT_TRIGGER -1]
            };
            joint_state_pub_->publish(joint);
        }
    }
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_; 
        bool publish_twist_mode_ = true;
        

};




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyMuxNode>());
    rclcpp::shutdown();
}