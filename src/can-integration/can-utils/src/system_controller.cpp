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

void SystemFrameBuilder::requestStatusFrame(){
    struct can_frame frame{};
    frame.can_id = 0x000502C0 | CAN_EFF_FLAG;
    frame.len = 1;
    frame.data[0] = 1;
    can_manager_->sendBlockingFrame(frame);
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
                                                
    builder_.buildArmMotorVelocityFrame(static_cast<uint32_t>(deviceT),
                                        Manufacturer::ARM_MOTOR_CONTROLLER,
                                        severity::SEV_STATUS,
                                        static_cast<uint32_t>(motor_id),
                                        static_cast<uint32_t>(device_id),
                                        velocity_rads);
}

void SystemFrameBuilder::sendForceStop(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
    builder_.sendShutDownRequest(static_cast<uint32_t>(DeviceType), static_cast<uint32_t>(deviceID)); 
}
void SystemFrameBuilder::sendResume(deviceType::DeviceType DeviceType, DeviceId::ID deviceID){
    builder_.sendRestartCommand(static_cast<uint32_t>(DeviceType), static_cast<uint32_t>(deviceID));
}

namespace {
uint32_t packServoFrame(std::shared_ptr<can_util::CANController>& mgr,
                        uint32_t can_id, int32_t value) {
    struct can_frame frame{};
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.len = 4;
    std::memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    frame.data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    frame.data[2] = static_cast<uint8_t>((value >>  8) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((value >>  0) & 0xFF);
    return mgr->sendBlockingFrame(frame);
}
}  // namespace

uint32_t SystemFrameBuilder::sendSpinServoPosition(int32_t degrees) {
    return packServoFrame(can_manager_, ServoCAN::SPIN_POSITION,
                          std::clamp(degrees, -ServoCAN::SPIN_MAX_DEG, ServoCAN::SPIN_MAX_DEG));
}

uint32_t SystemFrameBuilder::sendClampServoPosition(int32_t degrees) {
    return packServoFrame(can_manager_, ServoCAN::CLAMP_POSITION,
                          std::clamp(degrees, -ServoCAN::CLAMP_MAX_DEG, ServoCAN::CLAMP_MAX_DEG));
}

uint32_t SystemFrameBuilder::sendServoLed(uint32_t value) {
    return packServoFrame(can_manager_, ServoCAN::LED, static_cast<int32_t>(value));
}

uint32_t SystemFrameBuilder::sendQuerySpinPosition() {
    return packServoFrame(can_manager_, ServoCAN::QUERY_SPIN, 0);
}

uint32_t SystemFrameBuilder::sendQueryClampPosition() {
    return packServoFrame(can_manager_, ServoCAN::QUERY_CLAMP, 0);
}

