#include "buildAddress.hpp"
#include "can_interface.hpp"
#include "parser.hpp"
#include "prefixes.hpp"
#include <iostream>


//This class takes care of all the system handlers, each motor has a handler function with switch cases depening on the instruction type 
//being sent. 
class SystemFrameBuilder{

    public:

        // Function to send arm motor velocity to each motor
        inline void sendArmMotorVelocity(deviceType::DeviceType deviceT, Instructions::Inst motor_id, DeviceId::ID device_id, float velocity_rads){
            std::vector<uint8_t> data(8, 0);
            std::memcpy(data.data(), &velocity_rads, sizeof(velocity_rads));
            const auto can_id = builder_.buildCANID(static_cast<uint8_t>(deviceT), static_cast<uint8_t>(Manufacturer::TEAM_USE), static_cast<uint8_t>(severity::SEV_CNTRL), static_cast<uint8_t>(motor_id), static_cast<uint8_t>(device_id));
            can_controller->sendBlockingFrame(can_id, data);
//            return builder_.buildAddress(static_cast<uint8_t>(deviceT), Manufacturer::TEAM_USE, severity::SEV_CNTRL, static_cast<uint8_t>(motor_id), 
  //          static_cast<uint8_t>(device_id), velocity_rads); 
        }
        inline void sendForceStop(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
            builder_.sendShutDownRequest(static_cast<uint8_t>(DeviceType), static_cast<uint8_t>(deviceID)); 
        }
        inline void sendResume(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
            builder_.sendRestartCommand(static_cast<uint8_t>(DeviceType), static_cast<uint8_t>(deviceID));
        }
        // inline uint8_t sendError(uint8_t error_code){


        // }


        ~SystemFrameBuilder(){std::cout << "System frame builder destructor called" << std::endl; }

    private: 

        can_util::CANController::SharedPtr can_controller;
        BuildAddress builder_; 
        CANParser Parser_; 


};
