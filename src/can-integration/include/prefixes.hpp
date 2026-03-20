#pragma once
#include <cstdint>

namespace DeviceId{
    enum : uint8_t{   

        BAB = 0x00,
        JETSON = 0x01,
        AIRLINK_RECEIVER = 0x02,
        WHEEL_HARD_STOP = 0x03,
        ARM_HARD_STOP = 0X04,
        JMSB = 0x05, 
        COMPAT_BOARD = 0x06,
        BASE_ENCODER = 0x07,
        SHOULDER_ENCODER = 0x08,
        ELBOW_ENCODER = 0x09,
        FOREARM_ENCODER = 0x0A,
        WRIST_ENCODER = 0x0B,
        SPIN_SERVO_ENCODER = 0x0C,
        CLAMP_SERVO_ENCODER = 0x0D, 
        HUB = 0x0E,
        RESERVED_PAYLOAD = 0x0F, 
        SIL = 0x10
    }; 
} //namespace DeviceId

namespace manufacturer{
    enum : uint8_t{ 
        TEAM_USE = 0x08, 
        REV_ROBOTICS = 0x05,
    };
} //namespace manufacturer

namespace severity{
    enum : short{
        MANUAL_EMERGENCY_INTERVENTION = 0x00, 
        AUTOMATIC_EMERGENCY_INTERVENTION = 0x01,
        SYSTEM_STATUS = 0x02,
        CONTROL = 0x03
    };
} // namespace severity

namespace instructions{
    enum : uint8_t{
        CUT_POWER = 0x00,
        SEND_VELOCITY = 0,
        SEND_POSITION = 0, 
        STATUS = 0, 
        CHECK_VOLTAGE = 0, 
        TEMP = 0, 
        STOP = 0, 
        CHECK_HEALTH = 0, 
        ZERO_ENCODERS = 0, 
        SET_LED_COLOR = 0,
    };
} //namespace instructions

namespace deviceType{
    enum : uint8_t{
        POWER_DISTRIBUTION_MODULE = 0x08,
        MOTOR_CONTROLLER = 0x2,
        ENCODER = 0x07,
        COLOR_SENS0R = 0x0D, 
        SERVO_CONTROLLER = 0x0C,
        RESERVED = 0x0E
    };
} //namespace deviceType


