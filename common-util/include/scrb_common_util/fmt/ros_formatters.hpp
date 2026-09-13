#pragma once

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>

template <>
struct fmt::formatter<builtin_interfaces::msg::Time> : formatter<std::chrono::sys_time<std::chrono::nanoseconds>> {
    auto format(const builtin_interfaces::msg::Time& time, format_context& ctx) const {
        using namespace std::chrono;

        const auto tp = sys_time(
            seconds(time.sec) + nanoseconds(time.nanosec)
        );
        return formatter<sys_time<nanoseconds>>::format(tp, ctx);
    }
};

template <>
struct fmt::formatter<rclcpp::Time> : formatter<std::chrono::sys_time<std::chrono::nanoseconds>> {
    auto format(const rclcpp::Time& time, format_context& ctx) const {
        using namespace std::chrono;

        const auto tp = sys_time(nanoseconds(time.nanoseconds()));
        return formatter<sys_time<nanoseconds>>::format(tp, ctx);
    }
};

template <>
struct fmt::formatter<rclcpp::Duration> : formatter<std::chrono::duration<double>> {
    auto format(const rclcpp::Duration& duration, format_context& ctx) const {
        return formatter<std::chrono::duration<double>>::format(duration.to_chrono<std::chrono::duration<double>>(), ctx);
    }
};

template <>
struct fmt::formatter<builtin_interfaces::msg::Duration> : formatter<std::chrono::sys_time<std::chrono::nanoseconds>> {
    auto format(const builtin_interfaces::msg::Time& time, format_context& ctx) const {
        using namespace std::chrono;

        const auto tp = sys_time(
            seconds(time.sec) + nanoseconds(time.nanosec)
        );
        return formatter<sys_time<nanoseconds>>::format(tp, ctx);
    }
};

template <typename T>
struct RateFormatter : fmt::formatter<double> {
    auto format(const T& rate, fmt::format_context& ctx) const {
        using namespace std::chrono_literals;
        const auto hz = 1.0s / rate.period();
        return fmt::format_to(formatter::format(static_cast<double>(hz), ctx), "Hz");
    }
};

template <>
struct fmt::formatter<rclcpp::Rate> : RateFormatter<rclcpp::Rate> {};

template <>
struct fmt::formatter<rclcpp::WallRate> : RateFormatter<rclcpp::WallRate> {};
