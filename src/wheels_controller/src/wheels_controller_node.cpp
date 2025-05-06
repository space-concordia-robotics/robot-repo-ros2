#include "wheels_controller_node.h"

WheelsControllerNode::WheelsControllerNode() : rclcpp::Node("wheels_controller_node") {
    this->declare_parameter("can_path", "can0");
    this->declare_parameter("multiplier", 500);

   
   if (CANController::configureCAN("can0") != SUCCESS) {
        
        RCLCPP_ERROR(this->get_logger(), "Failed to configure CAN interface");
        rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "Initialized CAN interface: %s", "can0");
    // Subscribe to cmd_vel
    twist_msg_callback = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&WheelsControllerNode::TwistMessageCallback, this, std::placeholders::_1)
    );


    RCLCPP_INFO(this->get_logger(), "Subscribed to cmd_vel topic.");

    RCLCPP_INFO(this->get_logger(), "Movement controller initialized.");
}

void WheelsControllerNode::TwistMessageCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg) {
    // Extract linear and angular velocities from cmd_vel
    float linear_y = twist_msg->linear.x;
    float angular_z = twist_msg->angular.z;

    // Calculate wheel velocities
    float slip_track = 1.2f; // Distance between wheels
    float right_wheels_velocity = linear_y - (-angular_z * slip_track * 0.5f);
    float left_wheels_velocity = linear_y + (-angular_z * slip_track * 0.5f);

    // Convert to RPM using the multiplier parameter
    float multiplier = this->get_parameter("multiplier").as_int();
    float right_wheels_vel_rpm = right_wheels_velocity * multiplier;
    float left_wheels_vel_rpm = left_wheels_velocity * multiplier;

    // Send commands to the motors
    RevMotorController::velocityControl(1, right_wheels_vel_rpm);
    RevMotorController::velocityControl(2, right_wheels_vel_rpm);
    RevMotorController::velocityControl(3, right_wheels_vel_rpm);

    RevMotorController::velocityControl(4, left_wheels_vel_rpm);
    RevMotorController::velocityControl(5, left_wheels_vel_rpm);
    RevMotorController::velocityControl(6, left_wheels_vel_rpm);

    // Start the motors
    uint64_t mask = 0x7E;
    RevMotorController::startMotor(mask);

    RCLCPP_INFO(this->get_logger(), "Motor commands sent: Right RPM = %.2f, Left RPM = %.2f", right_wheels_vel_rpm, left_wheels_vel_rpm);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WheelsControllerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
