#pragma once

#include <rcl_interfaces/msg/log.hpp>

#include "foc2-gui/widget.hpp"
#include "foc2-gui/util/circular_buffer.hpp"

class RosLogWidget : public UiWidget {
    static constexpr auto MAX_LOGS = 2000;

    using Log = rcl_interfaces::msg::Log;

public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(RosLogWidget)

    explicit RosLogWidget(ImApplication& application) : UiWidget(application), logs(2000) {}

    void onInit() override;

protected:
    void draw() override;

private:
    enum class LogLevel {
        DEBUG = 1,
        INFO = 2,
        WARN = 3,
        ERROR = 4,
        FATAL = 5,
        UNKNOWN = 6,
    };

    struct LogEntry {
        builtin_interfaces::msg::Time timestamp;
        LogLevel level = LogLevel::UNKNOWN;
        std::string name;
        std::string msg;
        std::string file;
        std::string function;
        unsigned int line = 0;
    };

    std::mutex mutex;
    CircularBuffer<LogEntry> logs;

    rclcpp::Subscription<Log>::SharedPtr logs_subscription;

    LogLevel level_filter = LogLevel::DEBUG;

    std::string search_buf;
    std::string node_filter;

    void onLog(const Log::UniquePtr& msg);

    constexpr static LogLevel intToLogLevel(uint8_t level);

    static ImVec4 colorForSeverity(LogLevel level);

    bool severityAllowed(LogLevel lvl) const;

    void drawFilters();

    static void drawLogEntry(const LogEntry& entry);
};
