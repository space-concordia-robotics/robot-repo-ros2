#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <termios.h>
#include <net/if.h>
#include <iostream>
#include <array>
#include <bitset>
#include <stdio.h>
#include <stdlib.h>


//Testing to see if the proper CAN frame will be build with a series of instructions. 
class BuildAddress{

    public: 

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

            //00010 00001000 1100000000 00110 
        static uint32_t buildCANID(uint32_t DeviceType, uint32_t manufacturer, uint32_t severity, uint32_t instruction, uint32_t deviceId) {

            const uint32_t mfc_raw = manufacturer & MANUFACTURER_MASK; 
            const uint32_t dt = DeviceType & DEVICE_TYPE_MASK;
            const uint32_t sev2 = severity & SEVERITY_VALUE_MASK;
            const uint32_t inst8 = instruction & INSTRUCTION_MASK; 

            const uint32_t inst10 = (sev2 << 8) | inst8;
            const uint32_t id = deviceId & DEVICE_ID_MASK;

            return (dt << 24) | (mfc_raw << 16) | (inst10 << 6) | id; //Bit shifting to the left, in order to create the can frame 
        }
        //                              5                   8                   10                  6

        // [00011] [000010 [01] 00] [1100000010] [0011000] = 32
        //note: Order of frame is 1. Device type, 2. Manufacturer code, 3. instruction, and 4. DeviceID. 
        template <typename PayloadT> // fun fact, template variable is a variable that can work with any type specified when the variable is used ~ GFG.  
        static can_frame buildAddress(uint32_t deviceType, uint32_t manufacturerCode, uint32_t SEVERITY, uint16_t inst, uint32_t deviceID, const PayloadT& payload){

            struct can_frame frame{}; 
            static_assert(sizeof(PayloadT) <= 8, "Payload must be <= 8 bytes for CAN2.0B");

            //calling buildCANID to build the 29-bit canID with the given parameters (which btw, will be specified in the system_controller)
            
            const uint32_t canID = buildCANID(deviceType, manufacturerCode, SEVERITY, inst, deviceID); 

            //building the full frame


            //The bitwise | operator, will compare both bits in the same position and takes the biggest one. 
            //Example 011001 | 110001 = 111001
            frame.can_id = canID | CAN_EFF_FLAG; 
            frame.len = 8; 
            
            std::bitset<32> bits2(frame.can_id); 

            std::printf("Can id in buildAddress: %s\n\n", bits2.to_string().c_str()); 
            memset(frame.data, 0, sizeof(frame.data));
            memcpy(frame.data, &payload, sizeof(PayloadT));
            return frame;
            //CanManager::writeFrame(frame); 
        } 

        static uint32_t sendShutDownRequest(uint32_t DeviceType, uint32_t deviceID){
            
            struct can_frame frame{}; 
            if(deviceID == 0x06){
                const uint32_t compatID = buildCANID(DeviceType, 0x08, 0x03, 
                0x00, deviceID);
                frame.can_id = compatID | CAN_EFF_FLAG; 
                frame.len = 8; 

                std::bitset<29> pee(frame.can_id); 
                std::printf("CAN Frame ID in bits for the stop command (compat): %s\n\n", pee.to_string().c_str());
            } else if(deviceID == 0x0E){
                const uint32_t hubID = buildCANID(DeviceType, 0x08, 0x00, 
                   0x00, deviceID); 
                frame.can_id = hubID | CAN_EFF_FLAG; 
                frame.len = 8; 
                std::bitset<29> poo(frame.can_id); 
                std::printf("CAN Frame ID in bits for the stop command (Hub): %s\n\n", poo.to_string().c_str());
            }else{
                throw std::invalid_argument("Unknown Deviceid"); 
            }
            
            return 1; 
            //return manager_->sendBlockingFrame(frame); 

        } 

        static uint32_t sendRestartCommand(uint32_t DeviceType, uint32_t deviceID){

            struct can_frame frame{}; 
            if(deviceID == 0x06){
                const uint32_t compatID = buildCANID(DeviceType, 0x08, 0x03, 
                0x01, deviceID);
                frame.can_id = compatID | CAN_EFF_FLAG; 
                frame.can_dlc = 8; 
                std::bitset<29> peel(frame.can_id); 
                std::printf("CAN Frame ID in bits for the resume command (Compat): %s\n\n", peel.to_string().c_str());
            }
            if(deviceID == 0x0E){
                const uint32_t hubID = buildCANID(DeviceType,0x08, 0x00, 
                    0x01, deviceID); 
                frame.can_id = hubID | CAN_EFF_FLAG; 
                frame.can_dlc = 8; 
                std::bitset<32> pool(frame.can_id); 
                std::printf("CAN Frame ID in bits for the resume command (Hub): %s\n\n", pool.to_string().c_str());
            }
            
            return 1; 
            //return manager_->sendBlockingFrame(frame); 
        }
}; 

// static void printFrame(const can_frame& f)
// {
//     // Mask out EFF flag when printing the raw 29-bit ID:
//     uint32_t raw = f.can_id & CAN_EFF_MASK;

//     std::printf("can_id (with flags): 0x%08X\n", f.can_id);
//     std::printf("Can frame (with flags) length: %zu\n", sizeof(f.can_id)*8 );
//     std::printf("raw 29-bit id:       0x%08X\n", raw);

//     std::bitset<32> bits(f.can_id);
//     std::bitset<32> rawbits(raw);

//     std::printf("binary can (WITH FLAG)frame: %s\n", bits.to_string().c_str()); 
//     std::printf("binary can frame: %s\n", rawbits.to_string().c_str()); 
    
//     std::printf("dlc: %u\n", f.can_dlc);
//     std::printf("data:");
//     for (int i = 0; i < f.can_dlc; i++) {
//         std::printf(" %02X", f.data[i]);
//     }
//     std::printf("\n");
// }





// // Container for the decoded message components
// struct DecodedFrame {
//         uint32_t deviceType;
//         uint32_t manufacturer;
//         uint32_t severity;
//         uint16_t instruction;
//         uint32_t deviceId;
//         std::array<uint32_t, 8> data;  // Appends th DATA to an array 
// };


// class CANParser {
// public:
//     /**
//      * @brief Extracts protocol fields from a raw 29-bit CAN ID.
//      * Inverse logic of BuildAddress::buildCANID.
//      */
//     static DecodedFrame parse(uint32_t raw_id, const uint32_t* raw_data) {
//         DecodedFrame decoded;

//         // Shift right to reach the field, then AND with mask to isolate it
//         decoded.deviceType   = (raw_id >> 27) & 0x1F; // 5 bits
//         decoded.manufacturer = (raw_id >> 17) & 0xFF; // 8 bits
        
//         // Split the 10-bit instruction block into Severity and Instruction
//         uint16_t inst10      = (raw_id >> 7)  & 0x3FF; 
//         decoded.severity     = (inst10 >> 8)  & 0x03; // Top 2 bits of the 10
//         decoded.instruction  = inst10 & 0xFF;         // Bottom 8 bits of the 10
        
//         decoded.deviceId     = raw_id & 0x3F;         // Bottom 6 bits

//         // Copy raw bytes into the fixed-size array
//         for(int i = 0; i < 8; ++i) {
//             decoded.data[i] = raw_data[i];
//         }

//         return decoded;
//     }

//     /**
//      * @brief Casts the 8-byte array back into a specific data type (e.g. float).
//      */
//     template <typename T>
//     static T getValue(const std::array<uint32_t, 8>& data) {
//         // Reinterprets the memory address of the array as a pointer to type T
//         return *reinterpret_cast<const T*>(data.data());
//     }
// };



int main(){
    //   5      6     2   2  2    8       7
    // 00111 000010 [01] 00 11 00001111 0001000

    uint32_t severity           = 0x03; 
    uint32_t instruction       = 0x0F; // max 10-bit
    uint32_t deviceType         = 0x02;  // max 5-bit
    uint32_t manufacturerCode   = 0x08;  // 8-bit
    uint32_t deviceId           = 0x06;  // max 6-bit

    float payload = 2000.0f;


    std::cout << "\nTesting Builder" << std::endl;
    // Building the Frame 

    BuildAddress::buildAddress(deviceType, manufacturerCode, severity, instruction, deviceId, payload);
    BuildAddress::sendShutDownRequest(deviceType, deviceId); 
    BuildAddress::sendRestartCommand(deviceType, deviceId); 

    //printFrame(f);

    // std::cout << "\nTesting Parser" << std::endl; 
    // // Strip the CAN_EFF_FLAG for the parser logic to handle the pure 29-bit ID
    // auto decoded = CANParser::parse(f.can_id & CAN_EFF_MASK, f.data); 

    // // Verification Output
    // std::cout << "--- Decoded Results ---" << std::endl;
    // std::cout << "Device Type:  " << (int)decoded.deviceType << std::endl;
    // std::cout << "Manufacturer: " << (int)decoded.manufacturer << std::endl;
    // std::cout << "Severity:     " << (int)decoded.severity << std::endl;
    // std::cout << "Instruction:  " << (int)decoded.instruction << std::endl;
    // std::cout << "Device ID:    " << (int)decoded.deviceId << std::endl;

    // // Retrieve the payload back as a float
    // float decodedPayload = CANParser::getValue<float>(decoded.data);
    // std::cout << "Payload:      " << decodedPayload << std::endl;

    // // Final Test Check
    // if (decodedPayload == payload && decoded.deviceType == deviceType) {
    //     std::cout << "\nSUCCESS: Frame built and parsed correctly!" << std::endl;
    // } else {
    //     std::cout << "\nFAILURE: Mismatch detected between builder and parser." << std::endl;
    // }

    return 0; 
}