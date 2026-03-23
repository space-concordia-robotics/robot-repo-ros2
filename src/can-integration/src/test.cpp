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
#include <vector>


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


        static uint32_t buildCANID(uint8_t DeviceType, uint8_t manufacturer, uint8_t severity, uint16_t instruction, uint8_t deviceId) {


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
        //                              5                   8                   10                  6
        //note: Order of frame is 1. Device type, 2. Manufacturer code, 3. instruction, and 4. DeviceID. 
        template <typename PayloadT> // fun fact, template variable is a variable that can work with any type specified when the variable is used ~ GFG.  
        static can_frame buildAddress(uint8_t deviceType, uint8_t manufacturerCode, uint8_t SEVERITY, uint16_t inst, uint8_t deviceID, const PayloadT& payload){

            struct can_frame frame{}; 
            static_assert(sizeof(PayloadT) <= 8, "Payload must be <= 8 bytes for CAN2.0B");

            //calling buildCANID to build the 29-bit canID with the given parameters (which btw, will be specified in the system_controller)
            
            const uint32_t canID = buildCANID(deviceType, manufacturerCode, SEVERITY, inst, deviceID); 

            //building the full frame


            //The bitwise | operator, will compare both bits in the same position and takes the biggest one. 
            //Example 011001 | 110001 = 111001
            frame.can_id = canID | CAN_EFF_FLAG; 
            frame.can_dlc = sizeof(PayloadT); 

            memset(frame.data, 0, sizeof(frame.data));
            memcpy(frame.data, &payload, sizeof(PayloadT));
            return frame;
            //CanManager::writeFrame(frame); 
        } 
}; 

static void printFrame(const can_frame& f)
{
    // Mask out EFF flag when printing the raw 29-bit ID:
    uint32_t raw = f.can_id & CAN_EFF_MASK;

    std::printf("can_id (with flags): 0x%08X\n", f.can_id);
    std::printf("raw 29-bit id:       0x%08X\n", raw);
    std::printf("dlc: %u\n", f.can_dlc);
    std::printf("data:");
    for (int i = 0; i < f.can_dlc; i++) {
        std::printf(" %02X", f.data[i]);
    }
    std::printf("\n");
}





// Container for the decoded message components
struct DecodedFrame {
        uint8_t deviceType;
        uint8_t manufacturer;
        uint8_t severity;
        uint16_t instruction;
        uint8_t deviceId;
        std::array<uint8_t, 8> data;  // Appends th DATA to an array 
};


class CANParser {
public:
    /**
     * @brief Extracts protocol fields from a raw 29-bit CAN ID.
     * Inverse logic of BuildAddress::buildCANID.
     */
    static DecodedFrame parse(uint32_t raw_id, const uint8_t* raw_data) {
        DecodedFrame decoded;

        // Shift right to reach the field, then AND with mask to isolate it
        decoded.deviceType   = (raw_id >> 24) & 0x1F; // 5 bits
        decoded.manufacturer = (raw_id >> 16) & 0xFF; // 8 bits
        
        // Split the 10-bit instruction block into Severity and Instruction
        uint16_t inst10      = (raw_id >> 6)  & 0x3FF; 
        decoded.severity     = (inst10 >> 8)  & 0x03; // Top 2 bits of the 10
        decoded.instruction  = inst10 & 0xFF;         // Bottom 8 bits of the 10
        
        decoded.deviceId     = raw_id & 0x3F;         // Bottom 6 bits

        // Copy raw bytes into the fixed-size array
        for(int i = 0; i < 8; ++i) {
            decoded.data[i] = raw_data[i];
        }

        return decoded;
    }

    /**
     * @brief Casts the 8-byte array back into a specific data type (e.g. float).
     */
    template <typename T>
    static T getValue(const std::array<uint8_t, 8>& data) {
        // Reinterprets the memory address of the array as a pointer to type T
        return *reinterpret_cast<const T*>(data.data());
    }
};



int main(){

    uint8_t severity           = 0x01; 
    uint16_t instruction       = 0x01; // max 10-bit
    uint8_t deviceType         = 0x07;  // max 5-bit
    uint8_t manufacturerCode   = 0x08;  // 8-bit
    uint8_t deviceId           = 0x04;  // max 6-bit

    float payload = 2000.0f;

    std::cout << "\nTesting Builder" << std::endl;
    // Building the Frame 
    can_frame f = BuildAddress::buildAddress(deviceType, manufacturerCode, severity, instruction, deviceId, payload); 
    printFrame(f);


    std::cout << "\nTesting Parser" << std::endl; 
    // Strip the CAN_EFF_FLAG for the parser logic to handle the pure 29-bit ID
    auto decoded = CANParser::parse(f.can_id & CAN_EFF_MASK, f.data); 

    // Verification Output
    std::cout << "--- Decoded Results ---" << std::endl;
    std::cout << "Device Type:  " << (int)decoded.deviceType << std::endl;
    std::cout << "Manufacturer: " << (int)decoded.manufacturer << std::endl;
    std::cout << "Severity:     " << (int)decoded.severity << std::endl;
    std::cout << "Instruction:  " << (int)decoded.instruction << std::endl;
    std::cout << "Device ID:    " << (int)decoded.deviceId << std::endl;

    // Retrieve the payload back as a float
    float decodedPayload = CANParser::getValue<float>(decoded.data);
    std::cout << "Payload:      " << decodedPayload << std::endl;

    // Final Test Check
    if (decodedPayload == payload && decoded.deviceType == deviceType) {
        std::cout << "\nSUCCESS: Frame built and parsed correctly!" << std::endl;
    } else {
        std::cout << "\nFAILURE: Mismatch detected between builder and parser." << std::endl;
    }

    return 0; 
}