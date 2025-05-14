#include "arm_controller_node.h"
#include <sensor_msgs/msg/joint_state.hpp>
#include <byteswap.h>
#include <string.h>

ArmControllerNode::ArmControllerNode() : Node("arm_controller_node") {
    this->declare_parameter("local_mode", false);
    bool local_mode = this->get_parameter("local_mode").as_bool();
    RCLCPP_INFO(this->get_logger(), "Initialized node: %s", this->get_name());

    if (!local_mode) {
        fd = open("/dev/ttyTHS1", O_RDWR);
        if (fd < 0) {
            int errno0 = errno;
            RCLCPP_ERROR(this->get_logger(), "Error opening file: %i. Message: %s", errno0, strerror(errno));
            errno = 0;
            rclcpp::shutdown();
            return;
        }

        struct termios ttycfg;
        ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
        ttycfg.c_lflag = 0;
        ttycfg.c_iflag = 0;
        ttycfg.c_oflag = 0;
        ttycfg.c_line = 0;
        ttycfg.c_cc[VTIME] = 1; // 100ms timeout
        ttycfg.c_cc[VMIN] = 0;  // Return anything read so far
        cfsetispeed(&ttycfg, B57600);
        cfsetospeed(&ttycfg, B57600);

        tcsetattr(fd, TCSANOW, &ttycfg);
    }

    // Subscription to JointState messages
    arm_joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/arm_xyz_cmd", 10, std::bind(&ArmControllerNode::ArmJointCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscriptions initialized.");
}

ArmControllerNode::~ArmControllerNode() {
    if (!this->get_parameter("local_mode").as_bool()) {
        if (fd >= 0) {
            close(fd);
        }
    }
}

void ArmControllerNode::ArmJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->velocity.size() < 7) {
        RCLCPP_ERROR(this->get_logger(), "Received JointState message with insufficient velocity data.");
        return;
    }

    // Create a buffer to send motor commands
    uint8_t out_buf[1 + 1 + sizeof(float) * 6 + 1] = {}; // Correct buffer size
    out_buf[0] = SET_MOTOR_SPEED;
    out_buf[1] = sizeof(float) * 6;

    // Map JointState velocities to motor speeds
    for (int i = 0; i < 6; i++) {
        float speed = static_cast<float>(msg->velocity[i]) * MAX_MOTOR_SPEED;
        memcpy(&out_buf[(i * sizeof(float)) + 2], &speed, sizeof(float));
    }
    out_buf[14] = 0x0A; // End of message

    // Send the motor commands if not in local mode
    if (!this->get_parameter("local_mode").as_bool()) {
        int status = write(fd, out_buf, sizeof(out_buf));
        if (status == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error writing to device: %s", strerror(errno));
        }
    }

    RCLCPP_INFO(this->get_logger(), "Received JointState velocities: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                static_cast<float>(msg->velocity[0]), static_cast<float>(msg->velocity[1]),
                static_cast<float>(msg->velocity[2]), static_cast<float>(msg->velocity[3]),
                static_cast<float>(msg->velocity[4]), static_cast<float>(msg->velocity[5]),
                static_cast<float>(msg->velocity[6]));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
