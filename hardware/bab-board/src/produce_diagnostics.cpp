#include "bab-board/produce_diagnostics.hpp"

#include <utility>

static constexpr auto UPDATE_PERIOD_SEC = 10.0;
static constexpr auto UPDATE_PERIOD = std::chrono::seconds(10);

static constexpr auto BATTERY_VOLT_MIN = 10.0f;
static constexpr auto BATTERY_VOLT_MAX = 14.8f;
static constexpr auto RAIL_VOLT_MIN = 10.0f;
static constexpr auto RAIL_VOLT_MAX = 14.8f;

static constexpr auto BATTERY_CURRENT_WARN_MIN = 60.0f;
static constexpr auto BATTERY_CURRENT_ERROR_MIN = 80.0f;
static constexpr auto BATTERY_CURRENT_FAIL_MIN = 90.0f;

static constexpr auto RAIL_CURRENT_ERROR_MIN = 30.0f;
static constexpr auto TEMP_WARN_MIN = 60.0f;

ProduceDiagnostics::ProduceDiagnostics(
    rclcpp::Node& node,
    std::shared_ptr<BAB> bab_ptr,
    rclcpp::CallbackGroup::SharedPtr callback_group
)
    : node(node),
      updater(std::make_shared<diagnostic_updater::Updater>(&node)),
      bab(std::move(bab_ptr)) {
    updater->setHardwareID("Rover-PowerBoard");

    updater->add("Battery Voltage Status", this, &ProduceDiagnostics::checkBatteryVoltage);
    updater->add("Rail Voltage Status", this, &ProduceDiagnostics::checkRailVoltage);
    updater->add("Battery Current Status", this, &ProduceDiagnostics::checkBatteryCurrent);
    updater->add("Rail Current Status", this, &ProduceDiagnostics::checkRailCurrent);
    updater->add("Rail Power Status", this, &ProduceDiagnostics::checkRailPower);
    updater->add("Battery Temperature Status", this, &ProduceDiagnostics::checkBatteryTemperature);
    updater->add("Rail Temperature Status", this, &ProduceDiagnostics::checkRailTemperature);
    updater->add("TCU Temperature Status", this, &ProduceDiagnostics::checkTCUTemperature);
    updater->add("TCU Module Status", this, &ProduceDiagnostics::checkTCUStatus);
    updater->add("Relay Module Status", this, &ProduceDiagnostics::checkRelayStatus);

    updater->setPeriod(UPDATE_PERIOD_SEC);
    diagnostics_timer = node.create_wall_timer(
        UPDATE_PERIOD,
        [this] {
            diagnosticsCallback();
        },
        std::move(callback_group)
    );
}

bool ProduceDiagnostics::ensureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) const {
    if (bab) {
        return true;
    }

    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "BAB interface not initialized");
    return false;
}

// ReSharper disable once CppMemberFunctionMayBeConst
void ProduceDiagnostics::checkBatteryVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    uint8_t worst = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string worst_msg = "Battery levels OK";
    bool any_data = false;

    for (size_t i = 0; i < BAB::BATTERIES_COUNT; ++i) {
        if (!bab->batteryEverReceived(i)) {
            continue;
        }
        any_data = true;
        const auto voltage = bab->getBatteryVoltageLevel(i);
        stat.add("Battery " + std::to_string(i + 1) + " Voltage (V)", voltage);

        if (voltage > BATTERY_VOLT_MAX) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            worst_msg = "Battery voltage exceeding criticality, shut rover off immediately";
        } else if (voltage < BATTERY_VOLT_MIN && worst < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            worst_msg = "Battery voltage low, replace batteries";
        }
    }

    if (!any_data) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No battery telemetry received yet");
        return;
    }
    stat.summary(worst, worst_msg);
}

// ReSharper disable once CppMemberFunctionMayBeConst
void ProduceDiagnostics::checkRailVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    uint8_t worst = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string worst_msg = "Rail voltage levels OK";
    bool any_data = false;

    for (size_t i = 0; i < BAB::RAILS_COUNT; ++i) {
        if (!bab->railEverReceived(i)) {
            continue;
        }
        any_data = true;
        const auto voltage = bab->getRailVoltageLevel(i);
        stat.add("Rail " + std::to_string(i + 1) + " Voltage (V)", voltage);

        if (voltage > RAIL_VOLT_MAX) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            worst_msg = "Rail voltage exceeding criticality, shut rover off immediately";
        } else if (voltage < RAIL_VOLT_MIN && worst < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            worst_msg = "Rail voltages reaching low levels, battery may be dying";
        }
    }

    if (!any_data) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No rail telemetry received yet");
        return;
    }
    stat.summary(worst, worst_msg);
}

void ProduceDiagnostics::checkBatteryCurrent(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    uint8_t worst = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string worst_msg = "Battery current draw OK";
    bool any_data = false;

    for (size_t i = 0; i < BAB::BATTERIES_COUNT; ++i) {
        if (!bab->batteryEverReceived(i)) {
            continue;
        }
        any_data = true;
        const auto current = bab->getBatteryCurrentLevel(i);
        stat.add("Battery " + std::to_string(i + 1) + " Current (A)", current);

        if (current >= BATTERY_CURRENT_FAIL_MIN) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            worst_msg = "Battery current draw so high, potential failure detected";
            fault_detected = true;
        } else if (current >= BATTERY_CURRENT_ERROR_MIN) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            worst_msg = "Battery current draw exceeding high levels, shutting rover off";
            fault_detected = true;
        } else if (current >= BATTERY_CURRENT_WARN_MIN &&
            worst < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            worst_msg = "Battery current draw is high";
        }
    }

    if (!any_data) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No battery telemetry received yet");
        return;
    }
    stat.summary(worst, worst_msg);
}

void ProduceDiagnostics::checkRailCurrent(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    uint8_t worst = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string worst_msg = "Rail current levels OK";
    bool any_data = false;

    for (size_t i = 0; i < BAB::RAILS_COUNT; ++i) {
        if (!bab->railEverReceived(i)) {
            continue;
        }
        any_data = true;
        const auto current = bab->getRailCurrent(i);
        stat.add("Rail " + std::to_string(i + 1) + " Current (A)", current);

        if (current >= RAIL_CURRENT_ERROR_MIN) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            worst_msg = "Current level on rail is dangerously high, shutting rover off now";
            fault_detected = true;
        }
    }

    if (!any_data) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No rail telemetry received yet");
        return;
    }
    stat.summary(worst, worst_msg);
}

// ReSharper disable once CppMemberFunctionMayBeConst
void ProduceDiagnostics::checkRailPower(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    bool any_data = false;
    for (size_t i = 0; i < BAB::RAILS_COUNT; ++i) {
        if (!bab->railEverReceived(i)) {
            continue;
        }
        any_data = true;
        stat.add("Rail " + std::to_string(i + 1) + " Power (W)",
                 bab->getRailPower(i));
    }

    if (!any_data) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No rail telemetry received yet");
        return;
    }
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Rail power level OK");
}

// ReSharper disable once CppMemberFunctionMayBeConst
void ProduceDiagnostics::checkBatteryTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    uint8_t worst = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string worst_msg = "Battery temperature OK";
    bool any_data = false;

    for (size_t i = 0; i < BAB::BATTERIES_COUNT; ++i) {
        if (!bab->batteryEverReceived(i)) {
            continue;
        }
        any_data = true;
        const auto temp = bab->getBatteryTemp(i);
        stat.add("Battery " + std::to_string(i + 1) + " Temperature (C)", temp);
        if (temp >= TEMP_WARN_MIN && worst < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            worst_msg = "Warning, battery is overheating";
        }
    }

    if (!any_data) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No battery telemetry received yet");
        return;
    }
    stat.summary(worst, worst_msg);
}

// ReSharper disable once CppMemberFunctionMayBeConst
void ProduceDiagnostics::checkRailTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    // Firmware does not transmit rail temperature; field is always 0.
    stat.add("Rail Temperature (C)", bab->getRailTemp());
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Rail temperature OK");
}

// ReSharper disable once CppMemberFunctionMayBeConst
void ProduceDiagnostics::checkTCUTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    const auto tcu_temp = bab->getTCUTemp();
    stat.add("TCU Temperature (C)", tcu_temp);
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "TCU temperature OK");
}

// ReSharper disable once CppMemberFunctionMayBeConst
void ProduceDiagnostics::checkTCUStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    const std::string tcu_status = bab->getTCUStatus();
    stat.add("TCU Status", tcu_status);

    if (tcu_status == "TCU ON") {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "TCU operating normally");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "TCU status fault: " + tcu_status);
    }
}

// ReSharper disable once CppMemberFunctionMayBeConst
void ProduceDiagnostics::checkRelayStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    bool all_closed = true;
    for (size_t i = 0; i < BAB::RELAYS_COUNT; ++i) {
        const auto closed = bab->getRelayClosed(i);
        stat.add("Relay " + std::to_string(i + 1) + " Status",
                 closed ? "Closed (ON)" : "OPEN (OFF)");
        if (!closed) {
            all_closed = false;
        }
    }

    if (all_closed) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Relays operating normally");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "One or more relays open");
    }
}

void ProduceDiagnostics::diagnosticsCallback() {
    updater->force_update();

    if (!bab) {
        return;
    }

    if (fault_detected && !shutdown_sent) {
        RCLCPP_ERROR(
            node.get_logger(),
            "Overcurrent detected on BAB telemetry — shutdown command NOT sent "
            "(BAB command TX disabled until bench validation)");
        shutdown_sent = true;
    }
}
