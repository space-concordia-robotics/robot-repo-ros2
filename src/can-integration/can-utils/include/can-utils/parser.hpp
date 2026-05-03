#pragma once

#include "prefixes.hpp"
#include <array>


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
    static DecodedFrame parse(uint32_t raw_id, const std::vector<uint8_t>& raw_data) {
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