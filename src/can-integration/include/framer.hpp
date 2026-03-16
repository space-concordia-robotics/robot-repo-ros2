#pragma once 

#include "can_controller_node.hpp"
#include "can_interface.hpp"

//6 bites
struct Device_Id{   

    uint32_t BAB:6 = 0x00; 
    uint32_t JETSON:6 = 0x01;
    uint32_t AIRLINK_RECEIVER:6 = 0x02;
    uint32_t WHEEL_HARD_STOP:6 = 0x03; 
    uint32_t ARM_HARD_STOP:6 = 0X04; 
    uint32_t JMSB:6 = 0x05; 
    uint32_t COMPAT_BOARD:6 = 0x06;
    uint32_t BASE_ENCODER:6 = 0x07;
    uint32_t SHOULDER_ENCODER:6 = 0x08;
    uint32_t ELBOWsrc_ENCODER:6 = 0x09;
    uint32_t FOREARM_ENCODER:6 = 0x0A;
    uint32_t WRIST_ENCODER:6 = 0x0B;
    uint32_t SPIN_SERVO_ENCODER:6 = 0x0C;
    uint32_t CLAMP_SERVO_ENCODER:6 = 0x0D; 
    uint32_t HUB:6 = 0x0E; 
    uint32_t RESERVED_PAYLOAD:6 = 0x0F; 
    uint32_t SIL:6 = 0x10; 
}; 

// 8 bits
struct manufacturer{
    // 8 bit message 
    uint32_t TEAM_USE:8 = 0x08; 
    // 5 bit message
    uint32_t REV_ROBOTICS:5 = 0x05; 

};

// 10 bits
struct instruction{


    
};

//5 bits
struct devicetype{
    uint32_t MOTOR_CONTROLLER:5 = 0x2;
    uint32_t ENCODER:5 = 0x07;
    uint32_t COLOR_SENS0R:5 = 0x0D; 
    uint32_t SERVO_CONTROLLER:5 = 0x0C;
    uint32_t RESERVED:5 = 0x0E; 
};

class Framer{

    public: 

    private: 
}; 