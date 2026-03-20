#pragma once 

#include "can_controller_node.hpp"
#include "can_interface.hpp"
#include "prefixes.hpp"



//Note: This class is specifically for building the 29-bit adress. the Data holds any required info
// example: instruction: move at a specific velocity, data: is the float of the velocity value 
//The frame itself is CAN2.0B and SID and EID is the address. 
class BuildAddress{

    public: 

        //initializing the different masks that will go into building the frame
        // by specifying their bitlengths. 
        static constexpr uint32_t DEVICE_TYPE_MASK = 0x1Fu; //5 bits
        static constexpr uint32_t MANUFACTURER_MASK = 0xFFu; //8 bits
        static constexpr uint32_t INSTRUCTION_MASK = 0x3FFu; // 10 bits
        static constexpr uint32_t DEVICE_ID_MASK = 0x3Fu; // 6 bits


        static uint32_t buildCANID(uint8_t DeviceType, uint8_t manufacturer, uint16_t instruction, uint8_t deviceId) {


            //bitwise & operator truncates the parameters to be a specific bit length long. 
            // Think of the bitwise & operator as taking the "smallest" binary value between the two expressions
            // example: 111 & 001 will give 001, similarily, 011 & 110 will give 010, since 0 < 1, therefore, 0 will be selected. 
            const uint32_t dt = (static_cast<uint32_t>(DeviceType) & DEVICE_TYPE_MASK);
            const uint32_t mfc = (static_cast<uint32_t>(manufacturer) & MANUFACTURER_MASK);
            const uint32_t inst = (static_cast<uint32_t>(instruction) & INSTRUCTION_MASK);
            const uint32_t id = (static_cast<uint32_t>(deviceId) & DEVICE_ID_MASK);

            return (dt << 24) | (mfc << 16) | (inst << 6) | (id << 0); //Bit shifting to the left, in order to create the can frame 
        }
        //                              5                   8                   10                  6
        //note: Order of frame is 1. Device type, 2. Manufacturer code, 3. instruction, and 4. DeviceID. 
        template <typename PayloadT> // fun fact, template variable is a variable that can work with any type specified when the variable is used ~ GFG.  
        void buildAddress(uint16_t instructions, uint8_t deviceType, uint8_t manufacturerCode, uint8_t deviceID, const PayloadT& payload){

                struct can_frame frame{}; 
                static_assert(sizeof(PayloadT) <= 8, "Payload must be <= 8 bytes for CAn2.0B");

                //calling buildCANID to build the 29-bit canID with the given parameters (which btw, will be specified in the system_controller)
                
                const uint32_t canID = buildCANID(deviceType, manufacturerCode, instructions, deviceID); 

                //building the full frame


                //The bitwise | operator, will compare both bits in the same position and takes the biggest one. 
                frame.can_id = canID | CAN_EFF_FLAG; 
                frame.can_dlc 8; 

                memset(frame.data, 0, sizeof(frame.data));
                memcpy(frame.data, &payload, sizeof(PayloadT));

                CanManager::writeFrame(frame); 
        } 
}; 