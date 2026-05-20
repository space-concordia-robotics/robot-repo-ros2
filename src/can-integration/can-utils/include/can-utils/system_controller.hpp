#include "buildAddress.hpp"
#include "can_interface.hpp"
#include "parser.hpp"
#include <cassert>
#include "prefixes.hpp"
#include <iostream>


#define COMMAND_PREFIX_MAINTAIN_VELOCITY 0x82052c80 

//This class takes care of all the system handlers, each motor has a handler function with switch cases depening on the instruction type 
//being sent. 
class SystemFrameBuilder{

    public:

        explicit SystemFrameBuilder(std::shared_ptr<can_util::CANController> can_manager);

        uint32_t startMotors(uint32_t mask); 

        void requestStatusFrame();

        uint32_t sendWheelMotorVelocity(DeviceId::ID device_id, float velocity_payload);

        //Function to send arm motor velocity to each motor
        void sendArmMotorVelocity(deviceType::DeviceType deviceT, 
                                  Instructions::Inst motor_id, 
                                  DeviceId::ID device_id, 
                                  float velocity_rads);
        void sendForceStop(deviceType::DeviceType DeviceType, DeviceId::ID deviceID);
        void sendResume(deviceType::DeviceType DeviceType, DeviceId::ID deviceID);

        // Spin servo (SPIN wrist) — legacy FRC-style frames via buildServoFrame.
        // Position values are in rad, speed in rad/s.
        uint32_t sendSpinServoPosition(float position_rad);
        uint32_t sendSpinServoSpeed(float speed_rad_s);

        // Gripper (CLAMP) via Firmware_SPIN bridge — see docs/SERVO_API.md.
        // CAN ID: 0x180B0 (MAKE_ID ctrl/MOVE_POSITION/device 0x18), int32 BE relative degrees.
        // Firmware clamps to ±100°. Host-side clamp also applied here.
        // Replaces the legacy sendClampServoPosition / sendClampServoSpeed for production gripper use.
        uint32_t sendGripperMovePosition(int32_t degrees_relative);

        // Legacy clamp servo frames (old FRC-style ID 0x0708C04D/0x0708C10D).
        // Kept for spin servo parity; do NOT use for the gripper — use sendGripperMovePosition.
        uint32_t sendClampServoPosition(float position_rad);
        uint32_t sendClampServoSpeed(float speed_rad_s);

        ~SystemFrameBuilder(){std::cout << "System frame builder destructor called" << std::endl; }

    private: 
        
        std::shared_ptr<can_util::CANController> can_manager_; 
        buildAddress::BuildAddress builder_; 


};
