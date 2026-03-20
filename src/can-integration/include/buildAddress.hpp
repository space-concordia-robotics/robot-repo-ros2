#pragma once 

#include "can_controller_node.hpp"
#include "can_interface.hpp"
#include "prefixes.hpp"



//Note: This class is specifically for building the 29-bit adress. the Data holds any required info
// example: instruction: move at a specific velocity, data: is the float of the velocity value 
//The frame itself is CAN2.0B and SID and EID is the address. 
class BuildAddress{

    public: 

    //example can frame: 1100000111 --> send a control instruction to the encoder. from encoders to jetson 

    //                              5                   8                   10                  6
    //note: Order of frame is 1. Device type, 2. Manufacturer code, 3. instruction, and 4. DeviceID. 
    void buildAddress(uint16_t instructions, uint8_t deviceType, uint8_t manufacturerCode, uint8_t deviceID, auto sender){
        

        struct can_frame frame{}; 
        
        frame.can_id |= (deviceType << 24) | (manufacturerCode << 16) | (instructions << 6) || (deviceID); 
        frame.can_id |= CAN_EFF_FLAG; 
        frame.len8_dlc = 8; 
        
        memcpy(frame.data, &sender, sizeof(sender));
        CanManager::writeFrame(frame);
        
    } 

    virtual ~BuildAddress(){std::cout<< "BuildAddress destructor";}




    private: 
}; 