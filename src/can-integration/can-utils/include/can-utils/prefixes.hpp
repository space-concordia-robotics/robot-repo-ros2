#pragma once
#include <cstdint>


#define COMMAND_PREFIX_VELOCITY_CONTROL 0x820504

namespace deviceType{
    enum class DeviceType : uint32_t{
        JMSB = 0x00, 
        BAB = 0x01, 
        COMPAT = 0x02, 
        RELAY = 0x03, 
        ENCODER = 0X07, 
        POWER_DISTRIBUTION = 0X08, 
        ARM_MOTOR_CONTROLLER = 0X10,
        COLOR_SENSOR = 0X13, 
    };
} //namespace deviceType

namespace Manufacturer{
    enum : uint32_t{ 
        REV_ROBOTICS = 0x05,
        TEAM_USE = 0x08,
        ARM_MOTOR_CONTROLLER = 0x81
    };
} //namespace manufacturer

namespace severity{
    enum : uint32_t{
        SEV_MAN_INTERVENTION = 0x00,
        SEV_AUTOMATIC_INTERVENTION = 0x01,
        SEV_STATUS = 0x02, 
        SEV_CNTRL = 0x03
    };
} // namespace severity

namespace Instructions{
    enum class Inst: uint32_t{
        ARM_MOTOR_1 = 0x12,
        ARM_MOTOR_2 = 0x14,
        ARM_MOTOR_3 = 0x16,
        ARM_MOTOR_4 = 0x18,
        ARM_MOTOR_5 = 0x1A, 
        
        STOP_COMMAND = 0x01,
        RESUME_COMMAND = 0x02, 

        // Legacy servo instruction bytes removed — servo commands now use hardcoded
        // CAN IDs in the ServoCAN namespace below (int32 BE degree payloads).


        CUT_PDS_OUTPUTS = 0x8F,
        AUTOMATIC_RAIL_SHUTDOWN = 0x02, 


        COMMAND_OFF = 0x04,
        TURN_OFF_RELAY = 0X01, 
        TURN_OFF_FAN = 0X08, 
        
        COMMAND_ON = 0X06, 
        TURN_ON_RELAY = 0X02, 
        TURN_ON_FAN = 0X0A,


        // BAB telemetry instruction bytes -- see src/can-integration/docs/BAB-docs.md.
        // RAIL_TELEM is 0x02 in the BAB firmware; the previous 0x01 value was a typo.
        BATTERY_TELEM = 0x00,
        RAIL_TELEM = 0x02,
        TCU_TELEM = 0x03,


        RELAY_STATUS = 0x08,
        TCU_STATUS = 0x00




    };
} //namespace instructions

namespace DeviceId{
    enum class ID : uint32_t{   
        BAB = 0X00, 
        AIRLINK = 0X02, 
        WHEEL_EMERGENCY_INTERVENTION = 0X03,
        ARM_EMERGENCY_INTERVENTION = 0x04,
        JMSB = 0x01, 
        COMPAT_BOARD_ID = 0x06, 

        BASE_ENCODER = 0X07, 
        SHOULDER_ENCODER = 0X08, 
        ELBOW_ENCODER = 0X09, 
        FOREARM_ENCODER = 0X0A,
        WRIST_ENCODER = 0X0B,
        SPIN_SERVO_ENCODER = 0X0C,
        CLAMP_SERVO_ENCODER = 0X0D, 
        // Servo device IDs kept for reference; commands now use hardcoded
        // CAN IDs in the ServoCAN namespace (both map to device 0x0C).
        SPIN_SERVO  = 0X0C,
        CLAMP_SERVO = 0X0C,


        HUB = 0X0E,
        ARM_MOTOR_CONTROLLER = 0X0C,
        WHEEL_MOT1 = 1,
        WHEEL_MOT2 = 2, 
        WHEEL_MOT3 = 3, 
        WHEEL_MOT4 = 4, 
        WHEEL_MOT5 = 5,
        WHEEL_MOT6 = 6,
        
        SIL = 0X10
    }; 
} //namespace DeviceId

// Hardcoded CAN IDs for the servo protocol.
// Payload is always a signed int32 big-endian value (degrees for position
// commands, zero for queries and LED toggle).
//
// ID derivation (for reference):
//   base     = 0x0C08C000
//   CLAMP    = base | 0x00002040 | 0x0C = 0x0C08E04C
//   SPIN     = base | 0x00001040 | 0x0C = 0x0C08D04C
//   LED      = base | 0x00000400 | 0x0C = 0x0C08C40C
//   Q_CLAMP  = 0x0C08E08C
//   Q_SPIN   = 0x0C08D08C
namespace ServoCAN {
    constexpr uint32_t CLAMP_POSITION  = 0x0C08E04Cu;
    constexpr uint32_t SPIN_POSITION   = 0x0C08D04Cu;
    constexpr uint32_t LED             = 0x0C08C40Cu;
    constexpr uint32_t QUERY_CLAMP     = 0x0C08E08Cu;
    constexpr uint32_t QUERY_SPIN      = 0x0C08D08Cu;

    constexpr int32_t CLAMP_MAX_DEG = 100;
    constexpr int32_t SPIN_MAX_DEG  = 360;
} // namespace ServoCAN

