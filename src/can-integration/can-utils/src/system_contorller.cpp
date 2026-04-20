#include "can-utils/system_controller.hpp"



explicit SystemFrameBuilder::SystemFrameBuilder(std::shared_ptr<can_util::CANController> can_manager) : 
    can_manager_(std::move(can_manager)), builder_(can_manager_){

    if(!can_manager_){
        throw std::invalid_argument("can_manager_ must not be null"); 
    }
} 

inline uint32_t SystemFrameBuilder::sendWheelMotorVelocity(DeviceId::ID device_id, float velocity_payload){

    //TODO: (Michael) Assert here is broken. whenever 2 joysticks on the ps4 remote 
    //are used to simultaneously send commands, assert kicks in and terminates code. 
    assert(velocity_payload < 8 && "Error: Pyalad is greater than 8 bytes"); 
    struct can_frame frame{}; 

    frame.can_id = (COMMAND_PREFIX_VELOCITY_CONTROL << 8) | (static_cast<uint8_t>(device_id) + 0x80) | CAN_EFF_FLAG;
    frame.len = 8; 
    
    memcpy(frame.data, &velocity_payload, sizeof(float));
    return can_manager_->sendBlockingFrame(frame); 
}

//Function to send arm motor velocity to each motor
inline void SystemFrameBuilder::sendArmMotorVelocity(deviceType::DeviceType deviceT, Instructions::Inst motor_id, DeviceId::ID device_id, float velocity_rads){
    builder_.buildAddress(static_cast<uint8_t>(deviceT), Manufacturer::TEAM_USE, severity::SEV_CNTRL, static_cast<uint8_t>(motor_id), 
    static_cast<uint8_t>(device_id), velocity_rads); 
}

inline void SystemFrameBuilder::sendForceStop(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
    builder_.sendShutDownRequest(static_cast<uint8_t>(DeviceType), static_cast<uint8_t>(deviceID)); 
}
inline void SystemFrameBuilder::sendResume(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
    builder_.sendRestartCommand(static_cast<uint8_t>(DeviceType), static_cast<uint8_t>(deviceID));
}

