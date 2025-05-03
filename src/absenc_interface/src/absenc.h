#ifndef ABSENC_H
#define ABSENC_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <array>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#define NO_ERROR            0
#define ERR_SERIAL_FAILURE  1
#define ERR_SLAVE_INVALID   2
#define ERR_NO_RESPONSE     3
#define ERR_FRAME_CORRUPTED 4

typedef struct {
    int error;
    int cause;
    int line;
} ABSENC_Error_t;

const char* strAbsencErr(int err);


typedef struct {
    uint8_t slvnum;
    uint16_t status;
    double angval;
    double angspd;
} ABSENC_Meas_t;

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
    ~Absenc();

private:
    void absEncPollingCallback();

    int s_fd = -1;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angles_publisher_;

    std::string absenc_path_;
    int absenc_polling_rate_;
};

#endif