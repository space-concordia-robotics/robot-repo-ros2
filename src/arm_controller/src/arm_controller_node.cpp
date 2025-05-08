#include "arm_controller_node.h"
#include <geometry_msgs/msg/vector3.hpp>
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

    // Subscription to Joy messages
    //joy_msg_callback = this->create_subscription<sensor_msgs::msg::Joy>(
//"joy", 10, std::bind(&ArmControllerNode::JoyMessageCallback, this, std::placeholders::_1));

    // Subscription to Vector3 messages
    arm_xyz_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/arm_xyz_cmd", 10, std::bind(&ArmControllerNode::ArmXYZCallback, this, std::placeholders::_1));

    // Publisher for arm_xyz_cmd
   // arm_pub = this->create_publisher<geometry_msgs::msg::Vector3>("/arm_xyz_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "Subscriptions and publishers initialized.");
}

ArmControllerNode::~ArmControllerNode() {
    if (!this->get_parameter("local_mode").as_bool()) {
        if (fd >= 0) {
            close(fd);
        }
    }
}

/*void ArmControllerNode::JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    if (controller_type == -1) {
        if (joy_msg->axes[2] == 1.0 && joy_msg->axes[5] == 1.0) {
            RCLCPP_INFO(this->get_logger(), "Controller type 0");
            controller_type = 0;
        } else if (joy_msg->axes[4] == 1.0 && joy_msg->axes[5] == 1.0) {
            RCLCPP_INFO(this->get_logger(), "Controller type 1");
            controller_type = 1;
        } else {
            return;
        }
    }

    if (controller_type == 0) {
        if (joy_msg->buttons[4] == 0 || joy_msg->buttons[5] == 0) {
            return;
        }
    } else {
        if ((joy_msg->buttons[9] == 0 || joy_msg->buttons[10] == 0)) {
            return;
        }
    }

    float rotation;
    if (controller_type == 0) {
        rotation = (joy_msg->axes[2] - joy_msg->axes[5]) / 2;
    } else {
        if (joy_msg->axes[4] < 1) {
            rotation = -(joy_msg->axes[4] - 1.f) / 2.f;
        } else if (joy_msg->axes[5] < 1) {
            rotation = (joy_msg->axes[5] - 1.f) / 2.f;
        }
    }

    float speeds[6] = {rotation, joy_msg->axes[0], joy_msg->axes[1], joy_msg->axes[3], 0, 0};

    if (controller_type == 1) {
        speeds[2] = -speeds[2];
        speeds[3] = -speeds[3];
    }

    if (controller_type == 0) {
        speeds[4] = joy_msg->buttons[1] - joy_msg->buttons[3];
        speeds[5] = joy_msg->buttons[2] - joy_msg->buttons[0];
    } else {
        speeds[4] = joy_msg->buttons[1] - joy_msg->buttons[2];
        speeds[5] = joy_msg->buttons[3] - joy_msg->buttons[0];
    }

    uint8_t out_buf[1 + 1 + sizeof(float) * 6 + 1] = {};
    out_buf[0] = SET_MOTOR_SPEED;
    out_buf[1] = sizeof(float) * 6;

    for (int i = 0; i < 6; i++) {
        float speed = speeds[i] * MAX_MOTOR_SPEED;
        memcpy(&out_buf[(i * sizeof(float)) + 2], &speed, sizeof(float));
    }
    out_buf[26] = 0x0A;

    if (!this->get_parameter("local_mode").as_bool()) {
        int status = write(fd, out_buf, sizeof(out_buf));
        if (status == -1) {
            std::cout << "Error: " << errno << "\n";
        }
    }

    // Create a Vector3 message
    geometry_msgs::msg::Vector3 vec;

    // Map joystick axes to Vector3 fields
    vec.x = joy_msg->axes[0];  // Map joystick axis 0 to x
    vec.y = joy_msg->axes[1];  // Map joystick axis 1 to y
    vec.z = joy_msg->axes[2];  // Map joystick axis 2 to z

    // Publish the Vector3 message to /arm_xyz_cmd
    arm_pub->publish(vec);

    RCLCPP_INFO(this->get_logger(), "Published Vector3: x=%.2f, y=%.2f, z=%.2f", vec.x, vec.y, vec.z);
}*/

void ArmControllerNode::ArmXYZCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    // Create a buffer to send motor commands
    uint8_t out_buf[1 + 1 + sizeof(float) * 6 + 1] = {}; // Correct buffer size
    out_buf[0] = SET_MOTOR_SPEED;
    out_buf[1] = sizeof(float) * 6;

    // Map Vector3 fields to motor speeds (explicitly cast to float)
    float speeds[6] = {static_cast<float>(msg->x), static_cast<float>(msg->y), static_cast<float>(msg->z),
                       0.f, 0.f, 0.f}; // Initialize the rest to 0

    for (int i = 0; i < 6; i++) { // Iterate over 3 elements, not 6
        float speed = speeds[i] * MAX_MOTOR_SPEED;
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

    RCLCPP_INFO(this->get_logger(), "Received Vector3: x=%.2f, y=%.2f, z=%.2f", static_cast<float>(msg->x), static_cast<float>(msg->y), static_cast<float>(msg->z));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
