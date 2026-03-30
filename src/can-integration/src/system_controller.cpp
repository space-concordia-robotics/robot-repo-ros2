#include "system_controller.hpp"


//1. Send velocity cmds to each motor (5), 2. Send instruction to cut motors, 3. Send instruction to reinitialize motors. 
inline void SystemFrameBuilder::sendMotorVelocity(deviceType::DeviceType deviceT, Instructions::Inst motor_id, DeviceId::ID device_id, float velocity_rads){
    
    builder_.buildAddress(static_cast<uint8_t>(deviceT), Manufacturer::TEAM_USE, severity::SEV_CNTRL, static_cast<uint8_t>(motor_id), 
    static_cast<uint8_t>(device_id), velocity_rads); 
}

    
inline void SystemFrameBuilder::sendForceStop(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){

    builder_.sendShutDownRequest(static_cast<uint8_t>(DeviceType), static_cast<uint8_t>(deviceID)); 
    
}


inline void SystemFrameBuilder::sendResume(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
    
    builder_.sendRestartCommand(static_cast<uint8_t>(DeviceType), static_cast<uint8_t>(deviceID)); 

}



inline void SystemFrameBuilder::sendError(uint8_t error_code){

} 

