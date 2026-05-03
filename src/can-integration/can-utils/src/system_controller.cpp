#include "can-utils/system_controller.hpp"



SystemFrameBuilder::SystemFrameBuilder(std::shared_ptr<can_util::CANController> can_manager) 
        : can_manager_(std::move(can_manager)), builder_(can_manager_){

    if(!can_manager_){
        throw std::invalid_argument("can_manager_ must not be null"); 
    }
} 

uint32_t SystemFrameBuilder::startMotors(uint32_t mask){

    struct can_frame frame{};
    
    // uint64_t buf_data = (1ULL << deviceId);
    frame.can_id = COMMAND_PREFIX_MAINTAIN_VELOCITY;
    frame.can_id |= CAN_EFF_FLAG;
    frame.len = 8;
    memcpy(frame.data, &mask,sizeof(mask));
    
    return can_manager_->sendBlockingFrame(frame);

}

uint32_t SystemFrameBuilder::sendWheelMotorVelocity(DeviceId::ID device_id, float velocity_payload){

    static_assert(sizeof(float) <= 8, "Error: Payload must be 8 bytes or less");
    struct can_frame frame{}; 

    frame.can_id = (COMMAND_PREFIX_VELOCITY_CONTROL << 8) | (static_cast<uint32_t>(device_id) + 0x80) | CAN_EFF_FLAG;
    frame.len = 8; 
    
    memcpy(frame.data, &velocity_payload, sizeof(float));
    return can_manager_->sendBlockingFrame(frame); 
}

//Function to send arm motor velocity to each motor
void SystemFrameBuilder::sendArmMotorVelocity(deviceType::DeviceType deviceT, 
                                              Instructions::Inst motor_id, 
                                              DeviceId::ID device_id, 
                                              float velocity_rads){
                                                
    builder_.buildAddress(static_cast<uint32_t>(deviceT), 
                          Manufacturer::TEAM_USE, 
                          severity::SEV_CNTRL, 
                          static_cast<uint32_t>(motor_id), 
                          static_cast<uint32_t>(device_id), velocity_rads); 
}

void SystemFrameBuilder::sendForceStop(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
    builder_.sendShutDownRequest(static_cast<uint32_t>(DeviceType), static_cast<uint32_t>(deviceID)); 
}
void SystemFrameBuilder::sendResume(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
    builder_.sendRestartCommand(static_cast<uint32_t>(DeviceType), static_cast<uint32_t>(deviceID));
}

