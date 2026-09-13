#include "scrb_common_util/string_parsing.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <locale>
#include <variant>

namespace scrb::common_util {
    std::string to_lower_case(const std::string& input) {
        std::string lower_case_string = input;
        std::ranges::transform(
            lower_case_string,
            lower_case_string.begin(),
            [](const unsigned char c) {
                return std::tolower(c, std::locale::classic());
            }
        );
        return lower_case_string;
    }

    std::string_view trim_whitespace(const std::string_view& input) {
        static constexpr auto WHITESPACE = " \t\n\r\f\v";

        const auto first = input.find_first_not_of(WHITESPACE);

        if (first == std::string_view::npos)
            return {};

        const auto last = input.find_last_not_of(WHITESPACE);

        return input.substr(first, last - first + 1);
    }

    bool parse_bool(const std::string& input) {
        // Copy input to temp and make lowercase
        const auto temp = to_lower_case(input);

        if (temp == "true") {
            return true;
        }
        if (temp == "false") {
            return false;
        }
        // If input is not "true" or "false" (any casing), throw or handle as error
        throw std::invalid_argument(
            "Input string : '" + input +
            "' is not a valid boolean value. Expected 'true' or 'false'.");
    }

    std::vector<std::string> parse_string_array(const std::string& string_array_string) {
        return parse_array<std::string>(string_array_string);
    }
}
