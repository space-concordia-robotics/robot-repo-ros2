#pragma once
#include <cstdint>


#define COMMAND_PREFIX_SET_DEVCE_ID 0x205C0

#define COMMAND_PREFIX_RAMP 0x205CE

#define COMMAND_PREFIX_COAST 0x205C1
#define COMMAND_PREFIX_BRAKE 0x205C1

#define COMMAND_PREFIX_HARD_FORWARD_LIMIT 0x205CD0
#define COMMAND_PREFIX_HARD_REVERSE_LIMIT 0x205CD4

#define COMMAND_PREFIX_MOVE_MOTORS 0x20500

#define COMMAND_PREFIX_MAINTAIN_VELOCITY 0x82052c80 
#define COMMAND_PREFIX_MAINTAIN_SPEED 0x02052C80
#define COMMAND_PREFIX_READ_STATUS 0x502C0

#define COMMAND_PREFIX_PERIODIC_FRAME  0x2051
#define COMMAND_PREFIX_VELOCITY_CONTROL 0x820504

#define STATUS_FRAME_ID_OFFSET 0x40
#define DEVICE_MAX_ID 0x3E

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
        
        STOP_MOTOR = 0x00,
        RESUME_OPERATION = 0x01, 

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
        
        SIL = 0X10
    }; 
} //namespace DeviceId


