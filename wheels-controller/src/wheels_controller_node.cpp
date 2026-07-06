#include "wheels_controller_node.h"

#include "can_controller.h"
#include "rev_motor_controller.h"

using namespace std::literals::chrono_literals;

WheelsControllerNode::WheelsControllerNode()
    : Node("wheels_controller_node"), logger(get_logger()) {
    this->declare_parameter("can_path", "can0");
    multiplier = this->declare_parameter("multiplier", 500);

    if (CANController::configureCAN("can0") != SUCCESS) {
        logger.error("Failed to configure CAN interface");
        rclcpp::shutdown();
    }
    logger.info("Initialized CAN interface: {}", "can0");
    // Subscribe to cmd_vel
    twist_msg_callback = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        [this](const geometry_msgs::msg::Twist::UniquePtr& msg) {
            TwistMessageCallback(msg);
        }
    );

    parameter_event_handler = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto multiplier_callback = [this](const rclcpp::Parameter& parameter) {
        if (parameter.get_name() == "multiplier")
            multiplier = parameter.as_int();
    };

    multiplier_callback_handle = parameter_event_handler->add_parameter_callback("multiplier", multiplier_callback);


    logger.info("Subscribed to cmd_vel topic.");

    // Start the motors
    constexpr uint64_t mask = 0x7E;
    RevMotorController::startMotor(mask);

    logger.info("Movement controller initialized.");
}

void WheelsControllerNode::TwistMessageCallback(const geometry_msgs::msg::Twist::UniquePtr& twist_msg) const {
    // Extract linear and angular velocities from cmd_vel
    auto linear_y = twist_msg->linear.x;
    auto angular_z = twist_msg->angular.z;

    // multiply them for exponential control
    linear_y *= linear_y * linear_y;
    angular_z *= angular_z * angular_z;

    // Calculate wheel velocities
    constexpr auto slip_track = 1.2f; // Distance between wheels
    const auto right_wheels_velocity = -(linear_y - -angular_z * slip_track * 0.5f);
    const auto left_wheels_velocity = -(linear_y + -angular_z * slip_track * 0.5f);

    // Convert to RPM using the multiplier parameter
    const auto right_wheels_vel_rpm = right_wheels_velocity * this->multiplier;
    const auto left_wheels_vel_rpm = left_wheels_velocity * this->multiplier;

    // Send commands to the motors
    RevMotorController::velocityControl(1, right_wheels_vel_rpm);
    RevMotorController::velocityControl(2, right_wheels_vel_rpm);
    RevMotorController::velocityControl(3, right_wheels_vel_rpm);

    RevMotorController::velocityControl(4, left_wheels_vel_rpm);
    RevMotorController::velocityControl(5, left_wheels_vel_rpm);
    RevMotorController::velocityControl(6, left_wheels_vel_rpm);

    // Start the motors
    constexpr uint64_t mask = 0x7E;
    RevMotorController::startMotor(mask);

    logger.info("Motor commands sent: Right RPM = {:.2f}, Left RPM = {:.2f}", right_wheels_vel_rpm, left_wheels_vel_rpm);
}

int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<WheelsControllerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
