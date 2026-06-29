#include "arm_controller_node.h"

#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sensor_msgs/msg/joint_state.hpp>

ArmControllerNode::ArmControllerNode()
    : Node("arm_controller_node") {
    this->declare_parameter("local_mode", false);
    const bool local_mode = this->get_parameter("local_mode").as_bool();
    RCLCPP_INFO(this->get_logger(), "Initialized node: %s", this->get_name());

    if (!local_mode) {
        fd = open("/dev/ttyTHS1", O_RDWR);
        if (fd < 0) {
            const auto code = std::make_error_code(static_cast<std::errc>(errno));

            RCLCPP_ERROR(this->get_logger(), "Error opening file: %i. Message: %s", code.value(), code.message().c_str());
            errno = 0;
            rclcpp::shutdown();
            return;
        }

        termios ttycfg{};
        ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
        ttycfg.c_lflag = 0;
        ttycfg.c_iflag = 0;
        ttycfg.c_oflag = 0;
        ttycfg.c_line = 0;
        ttycfg.c_cc[VTIME] = 1; // 100ms timeout
        ttycfg.c_cc[VMIN] = 0; // Return anything read so far
        cfsetispeed(&ttycfg, B57600);
        cfsetospeed(&ttycfg, B57600);

        tcsetattr(fd, TCSANOW, &ttycfg);
    }

    // Subscription to JointState messages
    arm_joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/arm_xyz_cmd", 10, [this](const sensor_msgs::msg::JointState::UniquePtr& msg) {
            armJointCallback(msg);
        });

    RCLCPP_INFO(this->get_logger(), "Subscriptions initialized.");
}

ArmControllerNode::~ArmControllerNode() {
    if (!this->get_parameter("local_mode").as_bool()) {
        if (fd >= 0) {
            close(fd);
        }
    }
}

void ArmControllerNode::armJointCallback(const sensor_msgs::msg::JointState::UniquePtr& msg) const {
    if (msg->velocity.size() < 7) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Received JointState message with insufficient velocity data. Joint size is %zu, expected at least 7.",
            msg->velocity.size()
        );
        return;
    }

    // Create a buffer to send motor commands
    std::array<uint8_t, 1 + 1 + sizeof(float) * 6 + 2> out_buf = {}; // Correct buffer size
    out_buf.at(0) = SET_MOTOR_SPEED;
    out_buf.at(1) = sizeof(float) * 6;

    // Map JointState velocities to motor speeds
    for (int i = 0; i < 6; i++) {
        const auto speed = static_cast<float>(msg->velocity.at(i)) * MAX_MOTOR_SPEED;
        memcpy(&out_buf.at(i * sizeof(float) + 2), &speed, sizeof(float));
    }
    out_buf.at(27) = 0x0A; // End of message

    // Send the motor commands if not in local mode
    if (!this->get_parameter("local_mode").as_bool()) {
        if (const auto status = write(fd, out_buf.data(), sizeof(out_buf)); status == -1) {
            const auto code = std::make_error_code(static_cast<std::errc>(errno));
            RCLCPP_ERROR(this->get_logger(), "Error writing to device: %s", code.message().c_str());
        }
    }

    // TODO 2026-06-28 (Will Free): remove this message constantly logging the joint state
    RCLCPP_INFO(
        this->get_logger(),
        "Received JointState velocities: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        static_cast<float>(msg->velocity.at(0)),
        static_cast<float>(msg->velocity.at(1)),
        static_cast<float>(msg->velocity.at(2)),
        static_cast<float>(msg->velocity.at(3)),
        static_cast<float>(msg->velocity.at(4)),
        static_cast<float>(msg->velocity.at(5)),
        static_cast<float>(msg->velocity.at(6))
    );
}

int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<ArmControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
