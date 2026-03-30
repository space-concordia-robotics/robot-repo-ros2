#pragma once
#include <cstdint>

//to stop da compat board: (5 bits for device type)...000010000100000001... (missing 6 bits for deviceID)
#define STOP_BOARD 0b000010000100000001
#define REBOOT_MOTORS 0b000010000100000010

namespace deviceType{
    enum class DeviceType : uint8_t{
        COMPAT = 0x07, 
    };
} //namespace deviceType

namespace Manufacturer{
    enum : uint8_t{ 
        TEAM_USE = 0x08, 
        REV_ROBOTICS = 0x05,
    };
} //namespace manufacturer

namespace severity{
    enum : uint8_t{
       SEV_CNTRL = 0x01,
       SEV_STATUS = 0x02, 
    };
} // namespace severity

namespace Instructions{
    enum class Inst: uint8_t{
        ARM_MOTOR_1 = 0x09,
        ARM_MOTOR_2 = 0x0A,
        ARM_MOTOR_3 = 0x0B,
        ARM_MOTOR_4 = 0x0C,
        ARM_MOTOR_5 = 0x0D, 
        RESUME_MOTOR = 0x02
    };
} //namespace instructions

namespace DeviceId{
    enum class ID : uint8_t{   
        COMPAT_BOARD_ID = 0x06,  
    }; 
} //namespace DeviceId


