#pragma once 

#include "can_interface.hpp"
#include "prefixes.hpp"



//Note: This class is specifically for building the 29-bit adress. the Data holds any required info
// example: instruction: move at a specific velocity, data: is the float of the velocity value 
//The frame itself is CAN2.0B and SID and EID is the address. 
class BuildAddress{

    public: 

        //initializing the different masks that will go into building the frame
        // by specifying their bitlengths. 
        static constexpr uint32_t DEVICE_TYPE_MASK = 0x1Fu;  //5 bits
        static constexpr uint32_t MANUFACTURER_MASK = 0xFFu; //8 bits
        static constexpr uint32_t SEVERITY_VALUE_MASK = 0x3u;      //2bits
        static constexpr uint32_t INSTRUCTION_MASK = 0xFFu; // 8 bits
        static constexpr uint32_t DEVICE_ID_MASK = 0x3Fu;    // 6 bits


        uint32_t buildCANID(uint8_t DeviceType, uint8_t manufacturer, uint8_t severity, uint8_t instruction, uint8_t deviceId) {


            //bitwise & operator truncates the parameters to be a specific bit length long. 
            // Think of the bitwise & operator as taking the "smallest" binary value between the two expressions
            // example: 111 & 001 will give 001, similarily, 011 & 110 will give 010, since 0 < 1, therefore, 0 will be selected. 
            const uint32_t dt = (static_cast<uint32_t>(DeviceType) & DEVICE_TYPE_MASK);
            const uint32_t mfc = (static_cast<uint32_t>(manufacturer) & MANUFACTURER_MASK);

            const uint32_t sev2 = (static_cast<uint32_t>(severity) & SEVERITY_VALUE_MASK);
            const uint32_t inst8 = (static_cast<uint32_t>(instruction) & INSTRUCTION_MASK); 

            const uint32_t inst10 = (sev2 << 8) | inst8;
            const uint32_t id = (static_cast<uint32_t>(deviceId) & DEVICE_ID_MASK);

            return (dt << 24) | (mfc << 16) | (inst10 << 6) | id; //Bit shifting to the left, in order to create the can frame 
        }

        //build a new function that can send a force stop command and also a resume motor command, only taking in the device id as a parameter. 
        //and also the severity. 

        inline void sendShutDownRequest(uint8_t DeviceType, uint8_t deviceID){
            
            if(deviceID == static_cast<uint8_t>(DeviceId::ID::COMPAT_BOARD_ID)){
                const uint32_t compatID = buildCANID(DeviceType, Manufacturer::TEAM_USE, severity::SEV_CNTRL, 
                static_cast<uint8_t>(Instructions::Inst::STOP_MOTOR), deviceID);
                can_controller->sendBlockingFrame(compatID, std::vector<uint8_t>(0, 0)); 
            }

            if(deviceID == static_cast<uint8_t>(DeviceId::ID::HUB)){
                const uint32_t hubID = buildCANID(DeviceType, Manufacturer::TEAM_USE, severity::SEV_MAN_INTERVENTION, 
                    static_cast<uint8_t>(Instructions::Inst::STOP_MOTOR), deviceID); 
                can_controller->sendBlockingFrame(hubID, std::vector<uint8_t>(0,0)); 
            }

        } 

        inline void sendRestartCommand(uint8_t DeviceType, uint8_t deviceID){

            if(deviceID == static_cast<uint8_t>(DeviceId::ID::COMPAT_BOARD_ID)){
                const uint32_t compatID = buildCANID(DeviceType, Manufacturer::TEAM_USE, severity::SEV_CNTRL, 
                static_cast<uint8_t>(Instructions::Inst::RESUME_OPERATION), deviceID);
                can_controller->sendBlockingFrame(compatID, std::vector<uint8_t>(0, 0)); 
            }
            if(deviceID == static_cast<uint8_t>(DeviceId::ID::HUB)){
                const uint32_t hubID = buildCANID(DeviceType, Manufacturer::TEAM_USE, severity::SEV_MAN_INTERVENTION, 
                    static_cast<uint8_t>(Instructions::Inst::RESUME_OPERATION), deviceID); 
                can_controller->sendBlockingFrame(hubID, std::vector<uint8_t>(0, 0));
            }
        }

    private:
        can_util::CANController::SharedPtr can_controller;

}; 