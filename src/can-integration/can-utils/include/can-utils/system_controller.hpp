#include "buildAddress.hpp"
#include "can_interface.hpp"
#include "parser.hpp"
#include "prefixes.hpp"
#include <iostream>


//This class takes care of all the system handlers, each motor has a handler function with switch cases depening on the instruction type 
//being sent. 
class SystemFrameBuilder{

    public:

        SystemFrameBuilder(); 

        inline uint32_t sendWheelMotorVelocity(DeviceId::ID device_id, float velocity_payload){
            
            struct can_frame frame{}; 

            frame.can_id = (COMMAND_PREFIX_VELOCITY_CONTROL << 8) | (static_cast<uint8_t>(device_id) + 0x80) | CAN_EFF_FLAG;
            frame.can_dlc = 8; 
            
            memcpy(frame.data, &velocity_payload, sizeof(float));
            return can_manager_.sendBlockingFrame(frame); 
        }

        //Function to send arm motor velocity to each motor
        inline void sendArmMotorVelocity(deviceType::DeviceType deviceT, Instructions::Inst motor_id, DeviceId::ID device_id, float velocity_rads){
            builder_.buildAddress(static_cast<uint8_t>(deviceT), Manufacturer::TEAM_USE, severity::SEV_CNTRL, static_cast<uint8_t>(motor_id), 
            static_cast<uint8_t>(device_id), velocity_rads); 
        }
        
        inline void sendForceStop(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
            builder_.sendShutDownRequest(static_cast<uint8_t>(DeviceType), static_cast<uint8_t>(deviceID)); 
        }
        inline void sendResume(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
            builder_.sendRestartCommand(static_cast<uint8_t>(DeviceType), static_cast<uint8_t>(deviceID));
        }

         ~SystemFrameBuilder(){std::cout << "System frame builder destructor called" << std::endl; }

    private: 

        CanManager can_manager_; 
        BuildAddress builder_; 


};
