#include "scrb_common_util/string_parsing.hpp"

#include <gtest/gtest.h>

template <scrb::common_util::FloatingPoint T>
class parse_float : public testing::Test {
public:
    std::vector<std::pair<std::string, T>> valid_cases = {
        {"0", 0},
        {"1", 1},
        {"-1", -1},
        {"3.1415926", 3.1415926},
        {"1e2", 1e2},
        {"1e-2", 1e-2},
        {"  1", 1},
        {"1  ", 1},
        {"  1  ", 1},
    };

    std::vector<std::string> invalid_cases = {
        "abc",
        "12.3abc",
        "--5",
        "++1",
        " ",
        "",
    };

    std::vector<std::string> out_of_range_cases = std::is_same_v < T,



    float
    >
    ?
    std::vector<std::string> {
        "1e40",
            "-1e40",
    }

    :
    std::vector<std::string> {
        "1e400",
            "-1e400",
    };
};

using FloatingPointTypes = testing::Types<float, double>;
TYPED_TEST_SUITE(parse_float, FloatingPointTypes);

TYPED_TEST(parse_float, valid) {
    static_assert(std::is_same_v<TypeParam, float> || std::is_same_v<TypeParam, double>);

    for (const auto& test : this->valid_cases) {
        if constexpr (std::is_same_v<TypeParam, float>) {
            EXPECT_FLOAT_EQ(scrb::common_util::parse_float_generic<TypeParam>(test.first), test.second) << "Input: " << test.first;
            EXPECT_FLOAT_EQ(scrb::common_util::parse_float(test.first), test.second) << "Input: " << test.first;
        } else {
            EXPECT_DOUBLE_EQ(scrb::common_util::parse_float_generic<TypeParam>(test.first), test.second) << "Input: " << test.first;
            EXPECT_DOUBLE_EQ(scrb::common_util::parse_double(test.first), test.second) << "Input: " << test.first;
        }
    }
}

TYPED_TEST(parse_float, invalid) {
    for (const auto& test : this->invalid_cases) {
        EXPECT_THROW(
            scrb::common_util::parse_float_generic<TypeParam>(test),
            std::invalid_argument
        ) << "Input: " << test;
    }
}

TYPED_TEST(parse_float, out_of_range) {
    for (const auto& test : this->out_of_range_cases) {
        EXPECT_THROW(
            scrb::common_util::parse_float_generic<TypeParam>(test),
            std::out_of_range
        ) << "Input: " << test;
    }
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
