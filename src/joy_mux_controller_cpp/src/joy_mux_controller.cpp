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

        // Axis and button mapping to match the Python code
        enum Axis {
            AXIS_LEFT_STICK_X = 0,
            AXIS_LEFT_STICK_Y = 1,
            AXIS_RIGHT_STICK_X = 3,
            AXIS_RIGHT_STICK_Y = 4,
            AXIS_DPAD_X = 6,
            AXIS_DPAD_Y = 7
        };
        enum Button {
            BUTTON_A = 0, // X in PS
            BUTTON_B = 1, // Circle in PS
            BUTTON_X = 2, // Square in PS
            BUTTON_Y = 3, // Triangle in PS
            LEFT_BUMPER = 4,
            RIGHT_BUMPER = 5,
            // ... other buttons ...
            TOGGLE_BUTTON = 10 // Mode toggle (HOME)
        };
    void JoyToCmdCB(const sensor_msgs::msg::Joy::SharedPtr joy_cmds)
    {
        static int last_toggle = 0;
        // Mode toggle logic (edge detection)
        if (joy_cmds->buttons.size() > TOGGLE_BUTTON && joy_cmds->buttons[TOGGLE_BUTTON] == 1 && last_toggle == 0) {
            publish_twist_mode_ = !publish_twist_mode_;
            RCLCPP_INFO(this->get_logger(), "Switched to %s mode", publish_twist_mode_ ? "Rover" : "Arm");
        }
        if (joy_cmds->buttons.size() > TOGGLE_BUTTON)
            last_toggle = joy_cmds->buttons[TOGGLE_BUTTON];

        // Deadman button (LEFT_BUMPER)
        if (joy_cmds->buttons.size() > LEFT_BUMPER && joy_cmds->buttons[LEFT_BUMPER] == 1) {
            if (publish_twist_mode_) {
                auto twist = geometry_msgs::msg::Twist();
                // Map axes as in Python code
                if (joy_cmds->axes.size() > 7) {
                    twist.linear.x = joy_cmds->axes[AXIS_LEFT_STICK_Y]; // axes[1]
                    twist.angular.z = joy_cmds->axes[AXIS_LEFT_STICK_X]; // axes[0]
                    twist.linear.y = joy_cmds->axes[AXIS_DPAD_Y]; // axes[7]
                    twist.linear.z = joy_cmds->axes[AXIS_DPAD_X]; // axes[6]
                }
                twist_pub_->publish(twist);
            } else {
                auto joint = sensor_msgs::msg::JointState();
                joint.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
                size_t n = joint.name.size();
                joint.velocity.resize(n, 0.0);
                joint.position.resize(n, 0.0);
                joint.effort.resize(n, 0.0);
                // Fill velocity array as in Python code
                if (joy_cmds->axes.size() > 7 && joy_cmds->buttons.size() > 3) {
                    joint.velocity[0] = static_cast<double>(joy_cmds->axes[AXIS_DPAD_Y]); // axes[7]
                    joint.velocity[1] = static_cast<double>(joy_cmds->axes[AXIS_DPAD_X]); // axes[6]
                    joint.velocity[2] = static_cast<double>(joy_cmds->axes[AXIS_RIGHT_STICK_Y]); // axes[4]
                    joint.velocity[3] = static_cast<double>((joy_cmds->buttons.size() > 3 ? (joy_cmds->buttons[BUTTON_Y] ? 1 : 0) : 0) - (joy_cmds->buttons.size() > 1 ? (joy_cmds->buttons[BUTTON_B] ? 1 : 0) : 0));
                    joint.velocity[4] = static_cast<double>(joy_cmds->axes[AXIS_RIGHT_STICK_X]); // axes[3]
                    joint.velocity[5] = static_cast<double>(joy_cmds->axes[AXIS_LEFT_STICK_X]); // axes[0]
                    joint.velocity[6] = static_cast<double>((joy_cmds->buttons.size() > 0 ? (joy_cmds->buttons[BUTTON_A] ? 1 : 0) : 0) - (joy_cmds->buttons.size() > 1 ? (joy_cmds->buttons[BUTTON_B] ? 1 : 0) : 0));
                }
                joint_state_pub_->publish(joint);
            }
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