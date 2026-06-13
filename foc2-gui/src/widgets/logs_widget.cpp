#include "foc2-gui/widgets/logs_widget.hpp"

#include <imgui.h>
#include <fmt/chrono.h>
#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>
#include <misc/cpp/imgui_stdlib.h>

#include "foc2-gui/util/imgui_util.hpp"

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

void RosLogWidget::onInit() {
    UiWidget::onInit();

    logs_subscription = application.create_subscription<Log>(
        "/rosout", 1000,
        [this](const Log::UniquePtr& msg) {
            onLog(msg);
        }
    );
}

bool containsCaseInsensitive(const std::string& string, const std::string& search) {
    const auto iterator = std::ranges::search(
        string, search,
        [](const auto ch1, const auto ch2) {
            return std::toupper(ch1) == std::toupper(ch2);
        }
    ).begin();
    return iterator != string.end();
}

void RosLogWidget::draw() {
    drawFilters();

    const auto available = ImGui::GetContentRegionAvail();
    ImGui::BeginChild("ROS Logs", available, 0);

    std::lock_guard lock(mutex);

    const std::string search = search_buf;

    for (const auto& entry : logs) {
        if (!severityAllowed(entry.level))
            continue;

        if (!node_filter.empty() && entry.name != node_filter)
            continue;

        if (!search.empty()) {
            if (!containsCaseInsensitive(entry.msg, search) && !containsCaseInsensitive(entry.name, search))
                continue;
        }

        drawLogEntry(entry);
    }

    // autoscroll if already at bottom
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.0f);

    ImGui::EndChild();
}

void RosLogWidget::onLog(const Log::UniquePtr& msg) {
    std::lock_guard lock(mutex);
    const auto entry = LogEntry{
        .timestamp = msg->stamp,
        .level     = intToLogLevel(msg->level),
        .name      = msg->name,
        .msg       = msg->msg,
        .file      = msg->file,
        .function  = msg->function,
        .line      = msg->line
    };

    logs.push_back(entry);
}

constexpr RosLogWidget::LogLevel RosLogWidget::intToLogLevel(const uint8_t level) {
    switch (level) {
    case Log::DEBUG:
        return LogLevel::DEBUG;
    case Log::INFO:
        return LogLevel::INFO;
    case Log::WARN:
        return LogLevel::WARN;
    case Log::ERROR:
        return LogLevel::ERROR;
    case Log::FATAL:
        return LogLevel::FATAL;
    default:
        return LogLevel::UNKNOWN;
    }
}

ImVec4 RosLogWidget::colorForSeverity(const LogLevel level) {
    switch (level) {
    case LogLevel::DEBUG:
        return {0.6f, 0.6f, 0.6f, 1};
    case LogLevel::INFO:
        return {1, 1, 1, 1};
    case LogLevel::WARN:
        return {1, 0.8f, 0.2f, 1};
    case LogLevel::ERROR:
        return {1, 0.3f, 0.3f, 1};
    case LogLevel::FATAL:
    case LogLevel::UNKNOWN:
        return {1, 0, 0, 1};
    default: // should never happen
        return {1, 1, 1, 1};
    }
}

bool RosLogWidget::severityAllowed(const LogLevel lvl) const {
    return lvl >= level_filter;
}

void RosLogWidget::drawFilters() {
    ImGui::Separator();

    ImGui::Text("Filter:");
    ImGui::SameLine();

    const auto current_level_name = magic_enum::enum_name(level_filter);
    if (ImGui::BeginCombo("", current_level_name.data(), ImGuiComboFlags_WidthFitPreview)) {
        constexpr auto level_entries = magic_enum::enum_entries<LogLevel>();
        const auto level_index = magic_enum::enum_index(level_filter);

        for (auto i = 0u; i < level_entries.size(); i++) {
            const bool is_selected = level_index.value() == i;
            if (ImGui::Selectable(level_entries[i].second.data(), is_selected))
                level_filter = level_entries[i].first;

            if (is_selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::SameLine();

    ImGui::InputText("Search", &search_buf);

    ImGui::Separator();
}

void RosLogWidget::drawLogEntry(const LogEntry& entry) {
    const std::string message =
        fmt::format(
            "[{:%Y-%m-%d %H:%M:%S}] {} {} - {}",
            entry.timestamp,
            magic_enum::enum_name(entry.level),
            entry.name,
            entry.msg
        );

    ImGui::PushStyleColor(ImGuiCol_Text, colorForSeverity(entry.level));
    ImGui::TextUnformatted(message);
    ImGui::PopStyleColor();
}
