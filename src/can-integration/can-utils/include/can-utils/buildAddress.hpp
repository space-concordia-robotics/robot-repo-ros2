#pragma once 

#include "can-utils/can_interface.hpp"
#include "prefixes.hpp"


namespace buildAddress{

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

            
            /*
            *  @brief Helper function used to build CAN frame id  
            *  @param: uint32_t device type, manufacturerCode, severity, instruction, and device ID (all declared in prefixes.hpp)
            *  @details: The function builds CAN frame by bit shifting the different parameters to their respective positions.  
            *  @return Built 32-bit long CAN frame ID  
            * 
            */
            static uint32_t buildCANID(uint32_t DeviceType, 
                                    uint32_t manufacturer, 
                                    uint32_t severity, 
                                    uint32_t instruction, 
                                    uint32_t deviceId) {

                // Splitting 8-bit manufacturer code, to then add injection
                const uint32_t mfc_raw = manufacturer & MANUFACTURER_MASK;       
                const uint32_t dt = DeviceType & DEVICE_TYPE_MASK;
                const uint32_t sev2 = severity & SEVERITY_VALUE_MASK;
                const uint32_t inst8 = instruction & INSTRUCTION_MASK; 

                const uint32_t inst10 = (sev2 << 8) | inst8;
                const uint32_t id = deviceId & DEVICE_ID_MASK;
                
                //Returning a built frame
                return (dt << 24) | (mfc_raw << 16) | (inst10 << 6) | id; 
            }

            /*
            *  @brief Main handler function in charge of receiving built can frame and send it to the can manager 
            *  @param: uint32_t device type, manufacturerCode, severity, instruction, and device ID & payload
            *  @details: Builds CAN frame ID, 
            *            packs it into the frame.can_id struct,   
            *            Sets byte data payload to 8
            *            memset frame.data to zero in order to clear out any previous sent frames 
            *            memcpy's new frame.data and sends it to sendBlockingFrame
            *  @return returning frame via a shared_ptr pointing to sendBlockingFrame()
            * 
            */
            template <typename PayloadT> 
            uint32_t buildAddress(uint32_t deviceType, 
                                uint32_t manufacturerCode, 
                                uint32_t SEVERITY, 
                                uint32_t inst, 
                                uint32_t deviceID, 
                                const PayloadT& payload){

                    struct can_frame frame{}; 
                    static_assert(sizeof(PayloadT) <= 8, "Payload must be <= 8 bytes for CAN2.0B");

                    //calling buildCANID to build the 29-bit canID with the given parameters (which btw, will be specified in the system_controller)
                    
                    const uint32_t canID = buildCANID(deviceType, manufacturerCode, SEVERITY, inst, deviceID); 

                    //building the full frame here
                    frame.can_id = canID | CAN_EFF_FLAG; 
                    frame.can_dlc = 8; 

                    memset(frame.data, 0, sizeof(frame.data));
                    memcpy(frame.data, &payload, sizeof(PayloadT));

                    return manager_->sendBlockingFrame(frame); 
            }

            
            /*
            *  @brief Helper function used to send shutdown request to motors  
            *  @param: uint32_t device type, and device ID 
            *  @details: Function looks out to see if deviceID is either compat id or hub id. 
            *             it then sends the corresponding parameters to the buildCANID function to build the frame 
            *              depending on the id it was given.   
            *  @return returning frame via a shared_ptr pointing to sendBlockingFrame()  
            * 
            */
            uint32_t sendShutDownRequest(uint32_t DeviceType, uint32_t deviceID){
                
                struct can_frame frame{}; 
                if(deviceID == static_cast<uint32_t>(DeviceId::ID::COMPAT_BOARD_ID)){

                    const uint32_t compatID = buildCANID(DeviceType, 
                                                        Manufacturer::TEAM_USE, 
                                                        severity::SEV_MAN_INTERVENTION, 
                                                        static_cast<uint32_t>(Instructions::Inst::STOP_COMMAND),  
                                                        deviceID);
                    frame.can_id = compatID | CAN_EFF_FLAG; 
                    frame.len = 8; 

                } else if(deviceID == static_cast<uint32_t>(DeviceId::ID::HUB)){

                    const uint32_t hubID = buildCANID(DeviceType, 
                                                    Manufacturer::TEAM_USE, 
                                                    severity::SEV_MAN_INTERVENTION, 
                                                    static_cast<uint32_t>(Instructions::Inst::STOP_COMMAND),  
                                                    deviceID); 
                    frame.can_id = hubID | CAN_EFF_FLAG; 
                    frame.len = 8; 

                }else{
                    throw std::invalid_argument("Unknown Deviceid"); 
                }

                return manager_->sendBlockingFrame(frame); 

            } 

            /*
            *  @brief Helper function used to send Restart request to motors  
            *  @param: uint32_t device type, and device ID 
            *  @details: Function looks out to see if deviceID is either compat id or hub id. 
            *             it then sends the corresponding parameters to the buildCANID function to build the frame 
            *              depending on the id it was given.   
            *  @return returning frame via a shared_ptr pointing to sendBlockingFrame()  
            * 
            */
            uint32_t sendRestartCommand(uint32_t DeviceType, uint32_t deviceID){

                struct can_frame frame{}; 
                if(deviceID == static_cast<uint32_t>(DeviceId::ID::COMPAT_BOARD_ID)){

                    const uint32_t compatID = buildCANID(DeviceType, 
                                                        Manufacturer::TEAM_USE, 
                                                        severity::SEV_MAN_INTERVENTION, 
                                                        static_cast<uint32_t>(Instructions::Inst::RESUME_COMMAND), 
                                                        deviceID);
                    frame.can_id = compatID | CAN_EFF_FLAG; 
                    frame.can_dlc = 8; 
                    
                }else if(deviceID == static_cast<uint32_t>(DeviceId::ID::HUB)){

                    const uint32_t hubID = buildCANID(DeviceType,
                                                    Manufacturer::TEAM_USE, 
                                                    severity::SEV_MAN_INTERVENTION, 
                                                    static_cast<uint32_t>(Instructions::Inst::RESUME_COMMAND), 
                                                    deviceID); 
                    frame.can_id = hubID | CAN_EFF_FLAG; 
                    frame.can_dlc = 8; 

                }else{
                    throw std::invalid_argument("Unknown Deviceid"); 
                }

                return manager_->sendBlockingFrame(frame); 
            }

        private: 

            std::shared_ptr<can_util::CANController> manager_; 

    }; 

}