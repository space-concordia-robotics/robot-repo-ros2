#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

static constexpr auto NO_ERROR = 0;
static constexpr auto ERR_SERIAL_FAILURE = 1;
static constexpr auto ERR_SLAVE_INVALID = 2;
static constexpr auto ERR_NO_RESPONSE = 3;
static constexpr auto ERR_FRAME_CORRUPTED = 4;

struct ABSENC_Error_t {
    int error;
    int cause;
    int line;
};

const char* strAbsencErr(int err);


// TODO 2026-06-13 (Will Free): wtf is this
struct ABSENC_Meas_t {
    uint8_t slvnum;
    uint16_t status;
    double angval;
    double angspd;
};

#define no_error (ABSENC_Error_t{0, 0, __LINE__})

class AbsencDriver {
public:
    static ABSENC_Error_t OpenPort(const char* path, int& s_fd);
    static ABSENC_Error_t PollSlave(int slvnum, ABSENC_Meas_t* meas, int s_fd);
    static ABSENC_Error_t ClosePort(int s_fd);
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
    int absenc_polling_rate_;
};
