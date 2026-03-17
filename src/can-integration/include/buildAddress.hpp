#pragma once 

#include "can_controller_node.hpp"
#include "can_interface.hpp"



//Note: This class is specifically for building the 29-bit adress. the Data holds any required info
// example: instruction: move at a specific velocity, data: is the float of the velocity value 
//The frame itself is CAN2.0B and SID and EID is the address. 
class BuildAddress{

    public: 

    //example can frame: 1100000111 --> send a control instruction to the encoder. from encoders to jetson 
    virtual void buildAddress(); 

    virtual ~BuildAddress(){std::cout<< "BuildAddress destructor";}




    private: 
}; 