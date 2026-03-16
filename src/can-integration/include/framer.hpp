#pragma once 

#include "can_controller_node.hpp"
#include "can_interface.hpp"

//6 bites
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
}

// 8 bits
namespace manufacturer{
    enum : uint8_t{ 
        TEAM_USE = 0x08, 
        REV_ROBOTICS = 0x05,
    };
}

// 10 bits
namespace instructions{
    enum : uint16_t{
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
}

//5 bits
namespace deviceType{
    enum : uint8_t{
        MOTOR_CONTROLLER = 0x2,
        ENCODER = 0x07,
        COLOR_SENS0R = 0x0D, 
        SERVO_CONTROLLER = 0x0C,
        RESERVED = 0x0E
    };
}

struct fields{
    uint32_t device_id: 6;
    uint32_t device_type: 5; 
    uint32_t instruction: 10; 
    uint32_t manufacturer: 8; 
};

class Framer{

    public: 

    private: 
}; 