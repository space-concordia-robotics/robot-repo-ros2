#ifndef ABSENC_H
#define ABSENC_H

#include <string>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

enum class AbsencErrorCause : uint8_t {
    NONE,
    SERIAL_FAILURE,
    SLAVE_INVALID,
    NO_RESPONSE,
    FRAME_CORRUPTED,
};

const char* to_string(AbsencErrorCause cause);

struct AbsencError {
    AbsencErrorCause error;
    int cause;
    int line;
};

struct AbsencMeasurement {
    uint8_t slvnum;
    uint16_t status;
    double angval;
    double angspd;
};

static constexpr auto NO_ERROR = AbsencError{
    .error = AbsencErrorCause::NONE,
    .cause = 0,
    .line  = __LINE__,
};

class AbsencDriver {
public:
    static AbsencError openPort(const char* path, int& s_fd);
    static AbsencError pollSlave(int slvnum, AbsencMeasurement* meas, int s_fd);
    static AbsencError closePort(int s_fd);
};

class Absenc : public rclcpp::Node {
public:
    Absenc();
    ~Absenc() override;

private:
    float old_angle_4 = 0;

    int8_t angle_4_zone = 0;


    void absEncPollingCallback();

    int s_fd = -1;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angles_publisher_;

    std::string absenc_path_;

    ros2_fmt_logger::Logger logger;
};

#endif
