#include <thread>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <sensor_msgs/msg/joy.hpp>

// We'll just set up parameters here
static constexpr auto JOY_TOPIC = "/joy";
static constexpr auto TWIST_TOPIC = "/servo_node/delta_twist_cmds";
static constexpr auto JOINT_TOPIC = "/servo_node/delta_joint_cmds";
static constexpr auto WHEEL_VEL_TOPIC = "/cmd_vel";
static constexpr auto EEF_FRAME_ID = "gripper_claw_link";
static constexpr auto BASE_FRAME_ID = "base_structure_link";

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis : uint8_t { // NOLINT(*-use-enum-class)
    LEFT_STICK_X  = 0,
    LEFT_STICK_Y  = 1,
    LEFT_TRIGGER  = 2,
    RIGHT_STICK_X = 3,
    RIGHT_STICK_Y = 4,
    RIGHT_TRIGGER = 5,
    D_PAD_X       = 6,
    D_PAD_Y       = 7
};

enum Button : uint8_t { // NOLINT(*-use-enum-class)
    X                 = 0,
    CIRCLE            = 1,
    TRIANGLE          = 2,
    SQUARE            = 3,
    LEFT_BUMPER       = 4,
    RIGHT_BUMPER      = 5,
    CHANGE_VIEW       = 6,
    SHARE             = 8,
    HOME              = 10,
    LEFT_STICK_CLICK  = 11,
    RIGHT_STICK_CLICK = 12
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
static constexpr std::array AXIS_DEFAULTS = {
    0.0, /* LEFT_STICK_X */
    0.0, /* LEFT_STICK_Y */
    1.0, /* LEFT_TRIGGER */
    0.0, /* RIGHT_STICK_X */
    0.0, /* RIGHT_STICK_Y */
    1.0, /* RIGHT_TRIGGER */
    0.0, /* D_PAD_X */
    0.0, /* D_PAD_Y */
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
                     const geometry_msgs::msg::TwistStamped::UniquePtr& twist,
                     const control_msgs::msg::JointJog::UniquePtr& joint) {
    // Give joint jogging priority because it is only buttons
    // If any joint jog command is requested, we are only publishing joint commands
    if (
        buttons.at(X) != 0 || buttons.at(CIRCLE) != 0 || buttons.at(TRIANGLE) != 0 || buttons.at(SQUARE) != 0 ||
        axes.at(D_PAD_X) != 0 || axes.at(D_PAD_Y) != 0 || buttons.at(RIGHT_STICK_CLICK) != 0 || buttons.at(LEFT_STICK_CLICK) != 0
    ) {
        // Map the D_PAD to the proximal joints
        joint->joint_names.emplace_back("joint1");
        joint->velocities.emplace_back(axes.at(D_PAD_X));
        joint->joint_names.emplace_back("joint2");
        joint->velocities.emplace_back(axes.at(D_PAD_Y));

        // Map the diamond to the distal joints
        joint->joint_names.emplace_back("joint3");
        joint->velocities.emplace_back(buttons.at(CIRCLE) - buttons.at(SQUARE));
        joint->joint_names.emplace_back("joint5");
        joint->velocities.emplace_back(buttons.at(TRIANGLE) - buttons.at(X));
        // joint->joint_names.emplace_back("joint7");
        // joint->velocities.emplace_back(buttons[LEFT_STICK_CLICK] - buttons[RIGHT_STICK_CLICK]);
        return false;
    }

    // The bread and butter: map buttons to twist commands
    twist->twist.linear.z = axes.at(RIGHT_STICK_X);
    twist->twist.linear.y = axes.at(RIGHT_STICK_Y);

    const double lin_x_right = -0.5 * (axes.at(RIGHT_TRIGGER) - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
    const double lin_x_left = 0.5 * (axes.at(LEFT_TRIGGER) - AXIS_DEFAULTS.at(LEFT_TRIGGER));
    twist->twist.linear.x = lin_x_right + lin_x_left;

    twist->twist.angular.y = axes.at(LEFT_STICK_Y);
    twist->twist.angular.x = axes.at(LEFT_STICK_X);

    const double roll_positive = buttons.at(RIGHT_BUMPER);
    const double roll_negative = -1 * buttons.at(LEFT_BUMPER);
    twist->twist.angular.z = roll_positive + roll_negative;

    return true;
}


namespace moveit_servo {
    class JoyToServoPub : public rclcpp::Node {
    public:
        explicit JoyToServoPub(const rclcpp::NodeOptions& options)
            : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID) {
            // Setup pub/sub
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                JOY_TOPIC,
                rclcpp::SystemDefaultsQoS(),
                [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
                    joyCB(msg);
                }
            );

            twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
            joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
            collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

            // Create a service client to start the ServoNode
            servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
            servo_start_client_->wait_for_service(std::chrono::seconds(1));
            servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            RCLCPP_INFO(this->get_logger(), "Ik mux node publishing topic data");
        }

        ~JoyToServoPub() override {
            if (collision_pub_thread_.joinable())
                collision_pub_thread_.join();
        }

        void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg) const {
            // Create the messages we might publish
            auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

            // Convert the joystick message to Twist or JointJog and publish
            if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg)) {
                // publish the TwistStamped
                twist_msg->header.frame_id = frame_to_publish_;
                twist_msg->header.stamp = this->now();
                twist_pub_->publish(std::move(twist_msg));
            } else {
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
        std::string frame_to_publish_;

        std::thread collision_pub_thread_;
    };
}

// Register the component with class_loader
int main(const int argc, char** argv) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<moveit_servo::JoyToServoPub>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
