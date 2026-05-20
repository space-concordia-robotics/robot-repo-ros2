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

uint32_t SystemFrameBuilder::sendSpinServoPosition(float position_rad){
    return builder_.buildServoFrame(
        static_cast<uint32_t>(Instructions::Inst::SERVO_MOVE_TO_POSITION),
        static_cast<uint32_t>(DeviceId::ID::SPIN_SERVO),
        ServoSelector::SPIN,
        position_rad);
}

uint32_t SystemFrameBuilder::sendSpinServoSpeed(float speed_rad_s){
    return builder_.buildServoFrame(
        static_cast<uint32_t>(Instructions::Inst::SERVO_MOVE_AT_SPEED),
        static_cast<uint32_t>(DeviceId::ID::SPIN_SERVO),
        ServoSelector::SPIN,
        speed_rad_s);
}

uint32_t SystemFrameBuilder::sendGripperMovePosition(int32_t degrees_relative) {
    // Firmware_SPIN MOVE_POSITION frame per docs/SERVO_API.md.
    // MAKE_ID(ctrl=0b11, instruction=0x01, device=0x18) => 0x180B0
    // Payload: signed int32 big-endian relative degrees. Firmware clamps ±100° on its side.
    const int32_t clamped = std::clamp(degrees_relative, -100, 100);
    struct can_frame frame{};
    constexpr uint32_t kGripperMovePositionId = 0x180B0u;
    frame.can_id = kGripperMovePositionId | CAN_EFF_FLAG;
    frame.len = 4;
    std::memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = static_cast<uint8_t>((clamped >> 24) & 0xFF);
    frame.data[1] = static_cast<uint8_t>((clamped >> 16) & 0xFF);
    frame.data[2] = static_cast<uint8_t>((clamped >>  8) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((clamped >>  0) & 0xFF);
    return can_manager_->sendBlockingFrame(frame);
}

uint32_t SystemFrameBuilder::sendClampServoPosition(float position_rad){
    return builder_.buildServoFrame(
        static_cast<uint32_t>(Instructions::Inst::SERVO_MOVE_TO_POSITION),
        static_cast<uint32_t>(DeviceId::ID::CLAMP_SERVO),
        ServoSelector::CLAMP,
        position_rad);
}

uint32_t SystemFrameBuilder::sendClampServoSpeed(float speed_rad_s){
    return builder_.buildServoFrame(
        static_cast<uint32_t>(Instructions::Inst::SERVO_MOVE_AT_SPEED),
        static_cast<uint32_t>(DeviceId::ID::CLAMP_SERVO),
        ServoSelector::CLAMP,
        speed_rad_s);
}

