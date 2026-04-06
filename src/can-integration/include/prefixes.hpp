#pragma once
#include <cstdint>


#define COMMAND_PREFIX_VELOCITY_CONTROL 0x820504

namespace deviceType{
    enum class DeviceType : uint8_t{
        COMPAT = 0x02, 
        RELAY = 0x03, 
        ENCODER = 0X07, 
        POWER_DISTRIBUTION = 0X08, 
        COLOR_SENSOR = 0X13
    };
} //namespace deviceType

namespace Manufacturer{
    enum : uint8_t{ 
        REV_ROBOTICS = 0x05,
        TEAM_USE = 0x08
    };
} //namespace manufacturer

namespace severity{
    enum : uint8_t{
        SEV_MAN_INTERVENTION = 0x00,
        SEV_AUTOMATIC_INTERVENTION = 0x01,
        SEV_STATUS = 0x02, 
        SEV_CNTRL = 0x03
    };
} // namespace severity

namespace Instructions{
    enum class Inst: uint8_t{
        ARM_MOTOR_1 = 0x09,
        ARM_MOTOR_2 = 0x0A,
        ARM_MOTOR_3 = 0x0B,
        ARM_MOTOR_4 = 0x0C,
        ARM_MOTOR_5 = 0x0D, 
        
        STOP_COMPAT = 0x00,
        STOP_HUB = 0x00, 
        RESUME_MOTOR = 0x01, 
        RESUME_HUB = 0x01, 

    };
} //namespace instructions

namespace DeviceId{
    enum class ID : uint8_t{   
        BAB = 0X00, 
        AIRLINK = 0X02, 
        WHEEL_EMERGENCY_INTERVENTION = 0X03,
        ARM_EMERGENCY_INTERVENTION = 0x04,
        JMSB = 0x05, 
        COMPAT_BOARD_ID = 0x06, 

        BASE_ENCODER = 0X07, 
        SHOULDER_ENCODER = 0X08, 
        ELBOW_ENCODER = 0X09, 
        FOREARM_ENCODER = 0X0A,
        WRIST_ENCODER = 0X0B,
        SPIN_SERVO_ENCODER = 0X0C,
        CLAMP_SERVO_ENCODER = 0X0D, 

        HUB = 0X0E,
        WHEEL_MOT1 = 1,
        WHEEL_MOT2 = 2, 
        WHEEL_MOT3 = 3, 
        WHEEL_MOT4 = 4, 
        WHEEL_MOT5 = 5,
        WHEEL_MOT6 = 6,
        
        SIL = 0X10
    }; 
} //namespace DeviceId


