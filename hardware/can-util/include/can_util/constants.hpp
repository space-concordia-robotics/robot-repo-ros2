#pragma once

#include <stdint.h>

namespace can_util::constants {
    //! see https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html#device-type
    enum class DeviceType : uint8_t {
        BROADCAST_MESSAGE = 0,
        ROBOT_CONTROLLER = 1,
        MOTOR_CONTROLLER = 2,
        RELAY_CONTROLLER = 3,
        GYRO_SENSOR = 4,
        ACCELEROMETER = 5,
        DISTANCE_SENSOR = 6,
        ENCODER = 7,
        POWER_DISTRIBUTION_MODULE = 8,
        PNEUMATICS_CONTROLLER = 9,
        MISCELLANEOUS = 10,
        IO_BREAKOUT = 11,
        SERVO_CONTROLLER = 12,
        COLOR_SENSOR = 13,
        // 14-30 are reserved. allocate them as necessary.
        FIRMWARE_UPDATE = 31,
    };

    //! see https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html#manufacturer
    enum class Manufacturer : uint8_t {
        BROADCAST = 0,
        NI = 1,
        LUMINARY_MICRO = 2,
        DEKA = 3,
        CTR_ELECTRONICS = 4,
        REV_ROBOTICS = 5,
        GRAPPLE = 6,
        MINDSENSORS = 7,
        TEAM_USE = 8,
        KAUAI_LABS = 9,
        COPPERFORGE = 10,
        PLAYING_WITH_FUSION = 11,
        STUDICA = 12,
        THE_THRIFTY_BOT = 13,
        REDUX_ROBOTICS = 14,
        ANDYMARK = 15,
        VIVID_HOSTING = 16,
        VERTOS_ROBOTICS = 17,
        SWYFT_ROBOTICS = 18,
        LUMYN_LABS = 19,
        BRUSHLAND_LABS = 20,
        // 21-255 are reserved. allocate them as necessary.
    };

    //! see https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html#api-class
    enum class ApiClass : uint8_t {
        VOLTAGE_CONTROL_MODE = 0,
        SPEED_CONTROL_MODE = 1,
        VOLTAGE_COMPENSATION_MODE = 2,
        POSITION_CONTROL_MODE = 3,
        CURRENT_CONTROL_MODE = 4,
        STATUS = 5,
        PERIODIC_STATUS = 6,
        CONFIGURATION = 7,
        ACK = 8,
    };

    //! see https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html#api-index
    enum class ApiIndex : uint8_t {
        ENABLE_CONTROL = 0,
        DISABLE_CONTROL = 1,
        SET_SETPOINT = 2,
        P_CONSTANT = 3,
        I_CONSTANT = 4,
        D_CONSTANT = 5,
        SET_REFERENCE = 6,
        TRUSTED_ENABLE = 7,
        TRUSTED_SET_NO_ACK = 8,
        TRUSTED_SET_SETPOINT_NO_ACK = 10,
        SET_SETPOINT_NO_ACK = 11,
    };

    enum class Severity : uint8_t {
        MANUAL_INTERVENTION = 0x00,
        AUTOMATIC_INTERVENTION = 0x01,
        STATUS = 0x02,
        CNTRL = 0x03
    };
}
