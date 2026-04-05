#pragma once

#include "constants.hpp"

namespace can_util::FRCCan {
    template < typename
    T >
    static constexpr T mask(const uint8_t bits)
    {
        constexpr T one = 1;
        constexpr std::uint8_t width = std::numeric_limits<T>::digits;

        return bits == 0 ? 0 : bits >= width ? ~T{0} : (one << bits) - 1;
    }

    static constexpr auto DEVICE_TYPE_OFFSET = 24u;
    static constexpr auto MANUFACTURER_OFFSET = 16u;
    // FRC specific
    static constexpr auto API_CLASS_OFFSET = 10u;
    static constexpr auto API_INDEX_OFFSET = 6u;
    // our own custom spec
    static constexpr auto SEVERITY_OFFSET = 14u;
    static constexpr auto INSTRUCTION_OFFSET = 6u;

    static constexpr auto DEVICE_TYPE_MASK = mask<uint8_t>(6);
    static constexpr auto MANUFACTURER_MASK = mask<uint8_t>(8);
    static constexpr auto DEVICE_ID_MASK = mask<uint8_t>(6);
    // FRC specific
    static constexpr auto API_CLASS_MASK = mask<uint8_t>(6);
    static constexpr auto API_INDEX_MASK = mask<uint8_t>(4);
    // our own custom spec
    static constexpr auto SEVERITY_MASK = mask<uint8_t>(2);
    static constexpr auto INSTRUCTION_MASK = mask<uint8_t>(8);

    constexpr uint32_t createFrameId(
        const uint8_t device_type,
        const uint8_t manufacturer,
        const uint8_t severity,
        const uint8_t instruction,
        const uint8_t device_id
    )
    {
        return static_cast<uint32_t>(device_type & DEVICE_TYPE_MASK) << DEVICE_TYPE_OFFSET
            | static_cast<uint32_t>(manufacturer & MANUFACTURER_MASK) << MANUFACTURER_OFFSET
            | static_cast<uint32_t>(severity & SEVERITY_MASK) << SEVERITY_OFFSET
            | static_cast<uint32_t>(instruction & INSTRUCTION_MASK) << INSTRUCTION_OFFSET
            | static_cast<uint32_t>(device_id & DEVICE_ID_MASK);
    }

    constexpr uint32_t createFrameId(
        const constants::DeviceType device_type,
        const constants::Manufacturer manufacturer,
        const constants::Severity severity,
        const uint8_t instruction,
        const uint8_t device_id
    )
    {
        return createFrameId(
            static_cast<uint8_t>(device_type),
            static_cast<uint8_t>(manufacturer),
            static_cast<uint8_t>(severity),
            instruction,
            device_id
        );
    }

    constexpr uint32_t createFRCFrameId(
        const uint8_t device_type,
        const uint8_t manufacturer,
        const uint8_t api_class,
        const uint8_t api_index,
        const uint8_t device_id
    )
    {
        return static_cast<uint32_t>(device_type & DEVICE_TYPE_MASK) << DEVICE_TYPE_OFFSET
            | static_cast<uint32_t>(manufacturer & MANUFACTURER_MASK) << MANUFACTURER_OFFSET
            | static_cast<uint32_t>(api_class & API_CLASS_MASK) << API_CLASS_OFFSET
            | static_cast<uint32_t>(api_index & API_INDEX_MASK) << API_INDEX_OFFSET
            | static_cast<uint32_t>(device_id & DEVICE_ID_MASK);
    }

    constexpr uint32_t createFRCFrameId(
        const constants::DeviceType device_type,
        const constants::Manufacturer manufacturer,
        const constants::ApiClass api_class,
        const constants::ApiIndex api_index,
        const uint8_t device_id
    )
    {
        return createFRCFrameId(
            static_cast<uint8_t>(device_type),
            static_cast<uint8_t>(manufacturer),
            static_cast<uint8_t>(api_class),
            static_cast<uint8_t>(api_index),
            device_id
        );
    }
}
