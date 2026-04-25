#pragma once 

#include "can-utils/can_interface.hpp"
#include "prefixes.hpp"



//Note: This class is specifically for building the 29-bit adress. the Data holds any required info
// example: instruction: move at a specific velocity, data: is the float of the velocity value 
//The frame itself is CAN2.0B and SID and EID is the address. 
class BuildAddress{

    public: 

        explicit BuildAddress(const std::shared_ptr<can_util::CANController>& manager) : manager_(manager){
            if(!manager_){
                throw std::invalid_argument("manager_ must not be null"); 
            }
        }

        //initializing the different masks that will go into building the frame
        // by specifying their bitlengths. 
        static constexpr uint32_t DEVICE_TYPE_MASK = 0x1Fu;  //5 bits
        static constexpr uint32_t MANUFACTURER_MASK = 0xFFu; //8 bits
        static constexpr uint32_t SEVERITY_VALUE_MASK = 0x3u;      //2bits
        static constexpr uint32_t INSTRUCTION_MASK = 0xFFu; // 8 bits
        static constexpr uint32_t DEVICE_ID_MASK = 0x3Fu;    // 6 bits

        // [5 bits -> device type][6 bits -> manufactuer code][01 -> injection][2 bits -> manufacturer code][2 bits -> sev]
            //[8 bits -> inst][6 bits -> device type]

            // 00010/00001000//1100000000/001100

            // 00010/000010[01]00//1100000000/001100[0]

        static uint32_t buildCANID(uint8_t DeviceType, uint8_t manufacturer, uint8_t severity, uint8_t instruction, uint8_t deviceId) {

            const uint32_t mfc_raw = static_cast<uint32_t>(manufacturer) & MANUFACTURER_MASK; 
            const uint32_t mfc_high6 = (mfc_raw >> 2) & 0x3F;
            const uint32_t mfc_low2 = mfc_raw & 0x03;

    
            const uint32_t dt = (static_cast<uint32_t>(DeviceType) & DEVICE_TYPE_MASK);
            const uint32_t mfc = (mfc_high6 << 4) | (0x01u << 2) | mfc_low2; 
            const uint32_t sev2 = (static_cast<uint32_t>(severity) & SEVERITY_VALUE_MASK);
            const uint32_t inst8 = (static_cast<uint32_t>(instruction) & INSTRUCTION_MASK); 

            const uint32_t inst10 = (sev2 << 8) | inst8;
            const uint32_t id = (static_cast<uint32_t>(deviceId) & DEVICE_ID_MASK);

            return (dt << 27) | (mfc << 17) | (inst10 << 7) | id << 1; //Bit shifting to the left, in order to create the can frame 
        }
        //                              5                   8                   10                  6
        //note: Order of frame is 1. Device type, 2. Manufacturer code, 3. instruction, and 4. DeviceID. 
        template <typename PayloadT> 
        uint32_t buildAddress(uint8_t deviceType, uint8_t manufacturerCode, uint8_t SEVERITY, uint8_t inst, uint8_t deviceID, const PayloadT& payload){

                struct can_frame frame{}; 
                static_assert(sizeof(PayloadT) <= 8, "Payload must be <= 8 bytes for CAN2.0B");

                //calling buildCANID to build the 29-bit canID with the given parameters (which btw, will be specified in the system_controller)
                
                const uint32_t canID = buildCANID(deviceType, manufacturerCode, SEVERITY, inst, deviceID); 

                //building the full frame here


                //The bitwise | operator, will compare both bits in the same position and takes the biggest one. 
                frame.can_id = canID; 
                frame.can_dlc = 8; 

                memset(frame.data, 0, sizeof(frame.data));
                memcpy(frame.data, &payload, sizeof(PayloadT));

                return manager_->sendBlockingFrame(frame); 
        }

        //build a new function that can send a force stop command and also a resume motor command, only taking in the device id as a parameter. 
        //and also the severity. 

        uint32_t sendShutDownRequest(uint8_t DeviceType, uint8_t deviceID){
            
            struct can_frame frame{}; 
            if(deviceID == static_cast<uint8_t>(DeviceId::ID::COMPAT_BOARD_ID)){
                const uint32_t compatID = buildCANID(DeviceType, Manufacturer::TEAM_USE, severity::SEV_CNTRL, 
                static_cast<uint8_t>(Instructions::Inst::STOP_COMMAND), deviceID);
                frame.can_id = compatID | CAN_EFF_FLAG; 
                frame.len = 8; 
            } else if(deviceID == static_cast<uint8_t>(DeviceId::ID::HUB)){
                const uint32_t hubID = buildCANID(DeviceType, Manufacturer::TEAM_USE, severity::SEV_MAN_INTERVENTION, 
                    static_cast<uint8_t>(Instructions::Inst::STOP_COMMAND), deviceID); 
                frame.can_id = hubID | CAN_EFF_FLAG; 
                frame.len = 8; 
            }else{
                throw std::invalid_argument("Unknown Deviceid"); 
            }

            return manager_->sendBlockingFrame(frame); 

        } 

        uint32_t sendRestartCommand(uint8_t DeviceType, uint8_t deviceID){

            struct can_frame frame{}; 
            if(deviceID == static_cast<uint8_t>(DeviceId::ID::COMPAT_BOARD_ID)){
                const uint32_t compatID = buildCANID(DeviceType, Manufacturer::TEAM_USE, severity::SEV_CNTRL, 
                static_cast<uint8_t>(Instructions::Inst::RESUME_COMMAND), deviceID);
                frame.can_id = compatID | CAN_EFF_FLAG; 
                frame.can_dlc = 8; 
            }
            if(deviceID == static_cast<uint8_t>(DeviceId::ID::HUB)){
                const uint32_t hubID = buildCANID(DeviceType, Manufacturer::TEAM_USE, severity::SEV_MAN_INTERVENTION, 
                    static_cast<uint8_t>(Instructions::Inst::RESUME_COMMAND), deviceID); 
                frame.can_id = hubID | CAN_EFF_FLAG; 
                frame.can_dlc = 8; 
            }

            return manager_->sendBlockingFrame(frame); 
        }

    private: 

        std::shared_ptr<can_util::CANController> manager_; 

}; 