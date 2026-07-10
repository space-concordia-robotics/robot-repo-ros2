#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/executors.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "absenc_interface/absenc.hpp"


using namespace std::chrono_literals;
static constexpr auto GAIN = 20.0;

float scaleClamp(float val, const float scale, const float min, const float max) {
    val *= scale;
    val = val < min ? min : val;
    val = val > max ? max : val;
    return val;
}

Absenc::Absenc()
    : Node("absenc_node"), logger(get_logger()) {
    // Declare parameters
    this->declare_parameter("absenc_path", "/dev/ttyUSB0");
    this->declare_parameter("absenc_polling_rate", 100);


    // Initialize publisher
    angles_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Initialize timer for polling
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->get_parameter("absenc_polling_rate").as_int()),
        [this] {
            absEncPollingCallback();
        }
    );


    // Open serial connection
    if (AbsencError err = AbsencDriver::openPort(this->get_parameter("absenc_path").as_string().c_str(), s_fd); err.error != AbsencErrorCause::NONE) {
        const auto code = std::make_error_code(static_cast<std::errc>(err.cause));
        logger.error("Error opening file: {}. Message: {}", err.cause, code.message());
        rclcpp::shutdown();
    } else {
        logger.info("Successfully opened serial connection to {}", absenc_path_);
    }
}

Absenc::~Absenc() {
    if (s_fd >= 0) {
        AbsencDriver::closePort(s_fd);
    }
}

void Absenc::absEncPollingCallback() {
    // NOLINTBEGIN(*-pro-type-member-init)
    AbsencMeasurement absenc_meas_1;
    AbsencMeasurement absenc_meas_2;
    AbsencMeasurement absenc_meas_3;
    AbsencMeasurement absenc_meas_4;
    // NOLINTEND(*-pro-type-member-init)

    auto [error1, cause1, line1] = AbsencDriver::pollSlave(1, &absenc_meas_1, s_fd);
    auto [error2, cause2, line2] = AbsencDriver::pollSlave(2, &absenc_meas_2, s_fd);
    auto [error3, cause3, line3] = AbsencDriver::pollSlave(3, &absenc_meas_3, s_fd);
    auto [error4, cause4, line4] = AbsencDriver::pollSlave(4, &absenc_meas_4, s_fd);

    if (error1 != AbsencErrorCause::NONE) {
        logger.error("Error on 1: {} cause {} line {}\n", to_string(error1), cause1, line1);
    }
    if (error2 != AbsencErrorCause::NONE) {
        logger.error("Error on 2: {} cause {} line {}\n", to_string(error2), cause2, line2);
    }
    if (error3 != AbsencErrorCause::NONE) {
        logger.error("Error on 3: {} cause {} line {}\n", to_string(error3), cause3, line3);
    }
    if (error4 != AbsencErrorCause::NONE) {
        logger.error("Error on 4: {} cause {} line {}\n", to_string(error4), cause4, line4);
    }

    if (absenc_meas_1.status != 0 || absenc_meas_2.status != 0 || absenc_meas_3.status != 0 || absenc_meas_4.status != 0) {
        logger.error(
            "One of the absenc status returned an error. Here are the error codes: 0x{:04x} 0x{:04x} 0x{:04x} 0x{:04x}\n",
            absenc_meas_1.status, absenc_meas_2.status, absenc_meas_3.status, absenc_meas_4.status
        );
        //return;
    }


    // Fix the Home
    float angle_1 = absenc_meas_1.angval + 25; //-355
    float angle_2 = absenc_meas_2.angval - 174; //-175
    float angle_3 = absenc_meas_3.angval * -1;
    float angle_4 = absenc_meas_4.angval / 4.0f;

    // Normalize angles to range [-180, 180) rn it's 0 to 360
    //////////////////////////////////////////////////
    angle_1 = angle_1 < 180 ? angle_1 : angle_1 - 360;
    angle_2 = angle_2 > -180 ? angle_2 : angle_2 + 360;


    // update the old angle
    this->old_angle_4 = angle_4;

    angle_4 = angle_4 + this->angle_4_zone * 90 - 30;
    /////////////////////////////////////////////////

    // Publish angles
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = {"base", "shoulder", "bicep", "wrist"};
    joint_state_msg.position = {absenc_meas_4.angval, absenc_meas_1.angval, absenc_meas_2.angval, absenc_meas_3.angval};

    angles_publisher_->publish(joint_state_msg);

    // Print angles to the terminal
    if (absenc_meas_1.status == 0 || absenc_meas_2.status == 0 || absenc_meas_3.status == 0 || absenc_meas_4.status == 0) {
        logger.info("Angles: [{}, {}, {}, {}]", angle_4, angle_1, angle_2, angle_3);
    }
}


int main(const int argc, char* argv[]) {
    std::cout << "Starting absenc node\n";
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;

    const auto abs_node = std::make_shared<Absenc>();

    exe.add_node(abs_node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
