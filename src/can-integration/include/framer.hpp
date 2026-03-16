#pragma once 

#include "can_controller_node.hpp"
#include "can_interface.hpp"

//6
struct Device_Id{   

    uint32_t BAB = 0x0; 
    uint32_t JETSON = 0x1;
    uint32_t AIRLINK_RECEIVER = 0x2;
    uint32_t WHEEL_HARD_STOP = 0x3; 
    uint32_t ARM_HARD_STOP = 0X4; 
    uint32_t JMSB = 0x5; 
    uint32_t COMPAT_BOARD = 0x6;
    uint32_t BASE_ENCODER = 0x07;
    uint32_t SHOULDER_ENCODER = 0x08;
    uint32_t ELBOW_ENCODER = 0x09;
    uint32_t FOREARM_ENCODER = 0xA;
    uint32_t WRIST_ENCODER = 0xB;
    uint32_t SPIN_SERVO_ENCODER = 0xC;
    uint32_t CLAMP_SERVO_ENCODER = 0xD; 
    uint32_t HUB = 0xE; 
    uint32_t RESERVED_PAYLOAD = 0xF; 
    uint32_t SIL = 0x10; 
}; 

// 8 bits
struct manufacturer{

    
};

// 10 bits
struct instruction{

};

//5 bits
struct devicetype{

};


class Framer{


}; 