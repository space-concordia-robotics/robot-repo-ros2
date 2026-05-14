#pragma once

namespace hardware_interface {
    /// Constant defining red channel interface name
    constexpr auto HW_IF_RED = "red";
    /// Constant defining green channel interface name
    constexpr auto HW_IF_GREEN = "green";
    /// Constant defining blue channel interface name
    constexpr auto HW_IF_BLUE = "blue";
    /// Constant defining brightness channel interface name
    constexpr auto HW_IF_BRIGHTNESS = "brightness";

    /** \brief Overflow-safe conversion from string to int32_t.
     * \throws std::out_of_range if the converted value would fall out of the range of int32_t
     * \throws std::invalid_argument if no conversion could be performed
     */
    template <typename T>
    T stoi_generic(const std::string& s, const int base) {
        static_assert(std::is_integral_v<T> && std::is_signed_v<T>, "T must be a signed integral type");

        size_t pos;
        const auto v = std::stol(s, &pos, base); // stol returns long
        if (pos != s.length()) {
            throw std::invalid_argument("Invalid characters in string");
        }
        if (v < std::numeric_limits<T>::min() || v > std::numeric_limits<T>::max()) {
            throw std::out_of_range("value not in range of target type");
        }

        return static_cast<T>(v);
    }

    // Explicit instantiations for common signed integer types
    inline int8_t stoi8(const std::string& s, const int base) {
        return stoi_generic<int8_t>(s, base);
    }

    inline int16_t stoi16(const std::string& s, const int base) {
        return stoi_generic<int16_t>(s, base);
    }

    inline int32_t stoi32(const std::string& s, const int base) {
        return stoi_generic<int32_t>(s, base);
    }

    /** \brief Overflow-safe conversion from string to uint32_t.
     * \throws std::out_of_range if the converted value would fall out of the range of int32_t
     * \throws std::invalid_argument if no conversion could be performed
     */
    template <typename T>
    T stoui_generic(const std::string& s, const int base) {
        static_assert(std::is_integral_v<T> && std::is_unsigned_v<T>, "T must be an unsigned integral type");

        size_t pos;
        const auto v = std::stoul(s, &pos, base); // stoul returns unsigned long
        if (pos != s.length()) {
            throw std::invalid_argument("Invalid characters in string");
        }
        if (v > std::numeric_limits<T>::max()) {
            throw std::out_of_range("value not in range of target type");
        }

        return static_cast<T>(v);
    }

    // Explicit instantiations for common unsigned integer types
    inline uint8_t stoui8(const std::string& s, const int base) {
        return stoui_generic<uint8_t>(s, base);
    }

    inline uint16_t stoui16(const std::string& s, const int base) {
        return stoui_generic<uint16_t>(s, base);
    }

    inline uint32_t stoui32(const std::string& s, const int base) {
        return stoui_generic<uint32_t>(s, base);
    }
}
