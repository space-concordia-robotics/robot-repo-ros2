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

        uint32_t sendWheelMotorVelocity(DeviceId::ID device_id, float velocity_payload);

        //Function to send arm motor velocity to each motor
        void sendArmMotorVelocity(deviceType::DeviceType deviceT, 
                                  Instructions::Inst motor_id, 
                                  DeviceId::ID device_id, 
                                  float velocity_rads);
        void sendForceStop(deviceType::DeviceType DeviceType, DeviceId::ID deviceID);
        void sendResume(deviceType::DeviceType DeviceType, DeviceId::ID deviceID);

        ~SystemFrameBuilder(){std::cout << "System frame builder destructor called" << std::endl; }

    private: 
        
        std::shared_ptr<can_util::CANController> can_manager_; 
        buildAddress::BuildAddress builder_; 


};
