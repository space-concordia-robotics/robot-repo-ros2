#pragma once

#include <charconv>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>
#include <fmt/format.h>

namespace scrb::common_util {
    /**
     * Trims the whitespace from a string view.
     *
     * @param input The input string view/
     * @return The trimmed string view.
     */
    std::string_view trim_whitespace(const std::string_view& input);

    template <typename T>
    concept Stringish = std::same_as<T, std::string> || std::is_same_v<T, std::string_view>;

    template <typename T>
    concept Integral = std::integral<T>;

    template <typename T>
    concept FloatingPoint = std::floating_point<T>;

    template <typename T>
    concept HasFromChars = requires(const char* first, const char* last, T& value) {
        { std::from_chars(first, last, value) };
    };

    /**
     * Conversion from string to floating-point type T.
     *
     * @param input input string
     * @return The parsed floating-point value
     */
    template <FloatingPoint T, Stringish S>
    inline T parse_float_generic(const S& input) requires HasFromChars<T> {
        const auto trimmed = trim_whitespace(std::string_view(input));
        T value;
        // TODO 2026-06-20 (Will Free): this might not support *all* floating-point values.
        //  as of C++ 17, it supports float, double, and long double.
        //  as of C++ 26, it also supports std::float16_t, std::float32_t, std::float64_t, std::float128_t, and std::bloat16_t
        const auto result = std::from_chars(trimmed.begin(), trimmed.end(), value);

        if (result.ec == std::errc::invalid_argument || result.ptr != trimmed.end()) {
            throw std::invalid_argument("Invalid characters in string");
        }
        if (result.ec == std::errc::result_out_of_range) {
            throw std::out_of_range("Value not in range of target type");
        }

        return value;
    }

    /**
     * Helper function to convert a std::string to double in a locale-independent way.
     *
     * @throws std::invalid_argument if not a valid number or exceeds limits
    */
    template <Stringish S>
    inline double parse_double(const S& input) {
        return parse_float_generic<double>(input);
    }

    /**
     * Helper function to convert a std::string to float in a locale-independent way.
     *
     * @throws std::invalid_argument if not a valid number or exceeds limits
     */
    template <Stringish S>
    inline float parse_float(const S& input) {
        return parse_float_generic<float>(input);
    }

    /**
     * Conversion from string to integral type T.
     *
     * @throws std::out_of_range if the converted value would fall out of the range of T
     * @throws std::invalid_argument if no conversion could be performed
     */
    template <Integral T, Stringish S>
    inline T parse_int_generic(S& input, int base = 10) requires HasFromChars<T> {
        const auto trimmed = trim_whitespace(std::string_view(input));

        T value;
        const std::from_chars_result result = std::from_chars(trimmed.begin(), trimmed.end(), value, base);
        if (result.ec == std::errc::invalid_argument || result.ptr != trimmed.end()) {
            throw std::invalid_argument("Invalid characters in string");
        }
        if (result.ec == std::errc::result_out_of_range) {
            throw std::out_of_range("value not in range of target type");
        }

        return value;
    }

    template <Stringish S>
    inline int8_t parse_int8(S& input, int base = 10) {
        return parse_int_generic<int8_t>(input, base);
    }

    template <Stringish S>
    inline int16_t parse_int16(S& input, int base = 10) {
        return parse_int_generic<int16_t>(input, base);
    }

    template <Stringish S>
    inline int32_t parse_int32(S& input, int base = 10) {
        return parse_int_generic<int32_t>(input, base);
    }

    template <Stringish S>
    inline int64_t parse_int64(S& input, int base = 10) {
        return parse_int_generic<int64_t>(input, base);
    }

    template <Stringish S>
    inline uint8_t parse_uint8(S& input, int base = 10) {
        return parse_int_generic<uint8_t>(input, base);
    }

    template <Stringish S>
    inline uint16_t parse_uint16(S& input, int base = 10) {
        return parse_int_generic<uint16_t>(input, base);
    }

    template <Stringish S>
    inline uint32_t parse_uint32(S& input, int base = 10) {
        return parse_int_generic<uint32_t>(input, base);
    }

    template <Stringish S>
    inline uint64_t parse_uint64(S& input, int base = 10) {
        return parse_int_generic<uint64_t>(input, base);
    }

    /**
     * Convert a string to lower case.
     *
     * @param input The input string.
     * @return The lower case version of the input string.
     */
    std::string to_lower_case(const std::string& input);

    template <Stringish S>
    bool equals_case_insensitive(const S& first, const S& second) {
        return first.size() == second.size() && std::ranges::equal(first, second, [](const auto first_char, const auto second_char) {
            return std::tolower(first_char, std::locale::classic()) == std::tolower(second_char, std::locale::classic());
        });
    }

    /**
     * Parse a boolean value from a string.
     *
     * @param input The input string, can be "true", "false" (case insensitive)
     * @return The parsed boolean value.
     * @throws std::invalid_argument if the string is not a valid boolean representation.
     */
    template <Stringish S>
    bool parse_bool(const S& input) {
        const auto temp = to_lower_case(std::string(input));
        const auto lower_trimmed = trim_whitespace(temp);

        if (lower_trimmed == "0" || lower_trimmed == "f" || lower_trimmed == "false")
            return false;
        if (lower_trimmed == "1" || lower_trimmed == "t" || lower_trimmed == "true")
            return true;

        // If input is not "true" or "false" (any casing), throw or handle as error
        throw std::invalid_argument(
            fmt::format(
                "Not a valid boolean value, expected one of (case-insensitive): '0', 'f', or 'false' for false or '1', 't', or 'true' for true, but got: {}",
                input
            )
        );
    }

    // TODO 2026-06-03 (Will Free): this is not the best parsing code I've ever written, but whatever. it works.
    // NOLINTBEGIN(*-pro-bounds-pointer-arithmetic)
    template <Stringish S>
    inline std::chrono::nanoseconds parse_duration(const S& input) {
        using namespace std::chrono;

        auto total = nanoseconds(0);

        auto it = input.begin();
        const auto end = input.end();

        while (it != end) {
            while (it != end && std::isspace(*it, std::locale::classic()))
                ++it;

            if (it == end)
                break;

            const auto number_start = it;
            while (it != end && (std::isdigit(*it, std::locale::classic()) || *it == '.'))
                ++it;

            if (number_start == it) {
                auto position = std::distance(input.begin(), it);
                throw std::invalid_argument(
                    fmt::format(
                        "Expected a number:\n{}\n{:>{}}",
                        input, '^', position
                    )
                );
            }

            auto number = std::string_view(input).substr(std::distance(input.begin(), number_start), std::distance(number_start, it));

            uint64_t value = 0L;
            uint64_t decimal_factor = 1L;
            if (const auto period = number.find('.'); period == std::string::npos) {
                value = parse_uint64(number);
            } else {
                auto integer_part = number.substr(0, period);
                auto decimal_part = number.substr(period + 1);
                value = integer_part.empty() ? 0 : parse_uint64(integer_part);
                const auto decimal_value = decimal_part.empty() ? 0 : parse_uint64(decimal_part);

                for (std::size_t j = 0; j < decimal_part.size(); ++j)
                    decimal_factor *= 10;

                value = value * decimal_factor + decimal_value;
            }

            const auto unit_start = it;
            while (it != end && std::isalpha(*it, std::locale::classic()))
                ++it;

            if (it == unit_start) {
                auto position = std::distance(input.begin(), it);
                throw std::invalid_argument(fmt::format("Missing unit after number:\n{}\n{:>{}}", input, '^', position));
            }

            if (auto unit = std::string_view(input).substr(std::distance(input.begin(), unit_start), std::distance(unit_start, it)); unit == "d") {
                total += duration_cast<nanoseconds>(days(value)) / decimal_factor;
            } else if (unit == "h") {
                total += duration_cast<nanoseconds>(hours(value)) / decimal_factor;
            } else if (unit == "m") {
                total += duration_cast<nanoseconds>(minutes(value)) / decimal_factor;
            } else if (unit == "s") {
                total += duration_cast<nanoseconds>(seconds(value)) / decimal_factor;
            } else if (unit == "ms") {
                total += duration_cast<nanoseconds>(milliseconds(value)) / decimal_factor;
            } else if (unit == "us") {
                total += duration_cast<nanoseconds>(microseconds(value)) / decimal_factor;
            } else if (unit == "ns") {
                total += nanoseconds(value) / decimal_factor;
            } else {
                auto position = std::distance(input.begin(), unit_start) + 1;
                throw std::invalid_argument(fmt::format("Unknown unit '{}':\n{}\n{:>{}}{:~>{}}", unit, input, '^', position, ' ', unit.length()));
            }
        }

        return total;
    }

    // NOLINTEND(*-pro-bounds-pointer-arithmetic)

    template <typename T>
    std::vector<T> parse_array(const std::string& array_string) {
        if (array_string.size() < 2 || array_string.front() != '[' || array_string.back() != ']') {
            throw std::invalid_argument("String must start with '[' and end with ']'");
        }

        if (const auto inner = trim_whitespace(array_string.substr(1, array_string.size() - 2)); inner.empty())
            return {}; // Return empty array if input is "[]"

        std::vector<T> result;

        const auto push_token = [&](const std::string_view token, size_t start_index) {
            const auto trimmed_token = trim_whitespace(token);
            if (trimmed_token.empty()) {
                throw std::invalid_argument(fmt::format("String must be flat array with no empty elements.\n{}\n{:>{}}", array_string, '^', start_index));
            }

            if constexpr (std::is_same_v<T, std::string>) {
                result.emplace_back(trimmed_token);
            } else if constexpr (std::is_same_v<T, bool>) {
                result.push_back(parse_bool(trimmed_token));
            } else if constexpr (std::is_floating_point_v<T>) {
                try {
                    if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>) {
                        result.push_back(parse_float_generic<T>(trimmed_token));
                    } else {
                        static_assert(std::is_same_v<T, void>, "Unknown floating point type T");
                    }
                } catch (const std::exception& e) {
                    throw std::invalid_argument(
                        fmt::format(
                            "Failed converting string to floating point: {}\n{}\n{:>{}}{:~>{}}",
                            e.what(), array_string, '^', start_index + 1 + (token.size() - trimmed_token.size()), ' ', token.size() - 1
                        )
                    );
                }
            } else if constexpr (std::is_integral_v<T>) {
                try {
                    result.push_back(parse_int_generic<T>(trimmed_token));
                } catch (const std::exception& e) {
                    throw std::invalid_argument(
                        fmt::format(
                            "Failed converting string to integer: {}\n{}\n{:>{}}{:~>{}}",
                            e.what(), array_string, '^', start_index + 1 + (token.size() - trimmed_token.size()), ' ', token.size() - 1
                        )
                    );
                }
            } else {
                static_assert(std::is_same_v<T, void> && false, "Unsupported type for parsing"); // NOLINT(*-simplify-boolean-expr)
            }
        };

        size_t token_start = 1;
        for (size_t i = 1; i < array_string.size() - 2; ++i) {
            if (const auto c = array_string.at(i); c == '[' || c == ']') {
                throw std::invalid_argument(fmt::format("Nested brackets are not allowed in a flat array.\n{}\n{:>{}}", array_string, '^', i));
            } else if (c == ',') {
                push_token(std::string_view(array_string).substr(token_start, i - token_start), token_start);
                token_start = i + 1;
            }
        }

        push_token(std::string_view(array_string).substr(token_start, array_string.size() - token_start - 1), token_start);

        return result;
    }

    std::vector<std::string> parse_string_array(const std::string& string_array_string);
}
