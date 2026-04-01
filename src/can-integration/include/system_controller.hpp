#include "buildAddress.hpp"
#include "can_interface.hpp"
#include "parser.hpp"
#include "prefixes.hpp"
#include <iostream>


//This class takes care of all the system handlers, each motor has a handler function with switch cases depening on the instruction type 
//being sent. 
class SystemFrameBuilder{

    public:

        // Function to send motor velocity to each motor
        inline void sendMotorVelocity(deviceType::DeviceType deviceT, Instructions::Inst motor_id, DeviceId::ID device_id, float velocity_rads){
            builder_.buildAddress(static_cast<uint8_t>(deviceT), Manufacturer::TEAM_USE, severity::SEV_CNTRL, static_cast<uint8_t>(motor_id), 
            static_cast<uint8_t>(device_id), velocity_rads); 
        }
        
        inline void sendForceStop(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
            builder_.sendShutDownRequest(static_cast<uint8_t>(DeviceType), static_cast<uint8_t>(deviceID)); 
        }
        inline void sendResume(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
            builder_.sendRestartCommand(static_cast<uint8_t>(DeviceType), static_cast<uint8_t>(deviceID));
        }
        //inline void sendError(uint8_t error_code); 

         ~SystemFrameBuilder(){std::cout << "System frame builder destructor called" << std::endl; }

    private: 

        BuildAddress builder_; 


};


// class SystemParser{ 

//     //Other components (aka: bab, encoders, wheel encoders, wheel motors, etc) to jetson?? 

//     /*
//             The BAB will send commands to jetson such as:
//             1. send status of relays
//             2. send Voltage level of BMS
//             3. send health 
//             4. send temp 
//             5. send PDS telemetry
//             6. send satus of TCU

//             7. Send BMS triggered cutting power
//             8. send BAB triggered cutting a relay
//             9. Send notice that PDS failed
//             10. Send TCU report of any failure. 
//     */
//    public:

//     inline void CompatParser(); 

//    private:

//     BuildAddress builder_;

//    ~SystemParser() { std::cout << "System parser destructor called" << std::endl; }
// };