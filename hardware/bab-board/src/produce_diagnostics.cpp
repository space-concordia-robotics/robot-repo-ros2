#include "bab-board/produce_diagnostics.hpp"

#include <utility>

namespace {
    static constexpr auto K_UPDATE_PERIOD_SEC = 10.0;
    static constexpr auto K_UPDATE_PERIOD = std::chrono::seconds(10);

    static constexpr auto K_BATTERY_VOLT_MIN = 10.0f;
    static constexpr auto K_BATTERY_VOLT_MAX = 14.8f;
    static constexpr auto K_RAIL_VOLT_MIN = 10.0f;
    static constexpr auto K_RAIL_VOLT_MAX = 14.8f;

    static constexpr auto K_BATTERY_CURRENT_WARN_MIN = 60.0f;
    static constexpr auto K_BATTERY_CURRENT_ERROR_MIN = 80.0f;
    static constexpr auto K_BATTERY_CURRENT_FAIL_MIN = 90.0f;

    static constexpr auto K_RAIL_CURRENT_ERROR_MIN = 30.0f;
    static constexpr auto K_TEMP_WARN_MIN = 60.0f;
}

ProduceDiagnostics::ProduceDiagnostics(rclcpp::Node& node,
                                       std::shared_ptr<BAB> bab_ptr,
                                       rclcpp::CallbackGroup::SharedPtr callback_group)
    : node_(node),
      updater_(std::make_shared<diagnostic_updater::Updater>(&node_)),
      diagnostics_ptr_(std::move(bab_ptr)) {
    updater_->setHardwareID("Rover-PowerBoard");

    updater_->add("Battery Voltage Status", this, &ProduceDiagnostics::checkBatteryVoltage);
    updater_->add("Rail Voltage Status", this, &ProduceDiagnostics::checkRailVoltage);
    updater_->add("Battery Current Status", this, &ProduceDiagnostics::checkBatteryCurrent);
    updater_->add("Rail Current Status", this, &ProduceDiagnostics::checkRailCurrent);
    updater_->add("Rail Power Status", this, &ProduceDiagnostics::checkRailPower);
    updater_->add("Battery Temperature Status", this, &ProduceDiagnostics::checkBatteryTemperature);
    updater_->add("Rail Temperature Status", this, &ProduceDiagnostics::checkRailTemperature);
    updater_->add("TCU Temperature Status", this, &ProduceDiagnostics::checkTCUTemperature);
    updater_->add("TCU Module Status", this, &ProduceDiagnostics::checkTCUStatus);
    updater_->add("Relay Module Status", this, &ProduceDiagnostics::checkRelayStatus);

    updater_->setPeriod(kUpdatePeriodSec);
    diagnostics_timer_ = node_.create_wall_timer(
        kUpdatePeriod,
        [this]() {
            DiagnosticsCallback();
        },
        callback_group);
}

bool ProduceDiagnostics::ensureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) const {
    if (diagnostics_ptr_) {
        return true;
    }

    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "BAB interface not initialized");
    return false;
}

void ProduceDiagnostics::checkBatteryVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    uint8_t worst = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string worst_msg = "Battery levels OK";
    bool any_data = false;

    for (size_t i = 0; i < BAB::NUM_BATTERIES; ++i) {
        if (!diagnostics_ptr_->batteryEverReceived(i)) {
            continue;
        }
        any_data = true;
        const float v = diagnostics_ptr_->getBatteryVoltageLevel(i);
        stat.add("Battery " + std::to_string(i + 1) + " Voltage (V)", v);

        if (v > kBatteryVoltMax) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            worst_msg = "Battery voltage exceeding criticality, shut rover off immediately";
        } else if (v < kBatteryVoltMin && worst < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
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

void ProduceDiagnostics::checkRailVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    uint8_t worst = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string worst_msg = "Rail voltage levels OK";
    bool any_data = false;

    for (size_t i = 0; i < BAB::NUM_RAILS; ++i) {
        if (!diagnostics_ptr_->railEverReceived(i)) {
            continue;
        }
        any_data = true;
        const float v = diagnostics_ptr_->getRailVoltageLevel(i);
        stat.add("Rail " + std::to_string(i + 1) + " Voltage (V)", v);

        if (v > kRailVoltMax) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            worst_msg = "Rail voltage exceeding criticality, shut rover off immediately";
        } else if (v < kRailVoltMin && worst < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
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

    for (size_t i = 0; i < BAB::NUM_BATTERIES; ++i) {
        if (!diagnostics_ptr_->batteryEverReceived(i)) {
            continue;
        }
        any_data = true;
        const float current = diagnostics_ptr_->getBatteryCurrentLevel(i);
        stat.add("Battery " + std::to_string(i + 1) + " Current (A)", current);

        if (current >= kBatteryCurrentFailMin) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            worst_msg = "Battery current draw so high, potential failure detected";
            fault_detected_ = true;
        } else if (current >= kBatteryCurrentErrorMin) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            worst_msg = "Battery current draw exceeding high levels, shutting rover off";
            fault_detected_ = true;
        } else if (current >= kBatteryCurrentWarnMin &&
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

    for (size_t i = 0; i < BAB::NUM_RAILS; ++i) {
        if (!diagnostics_ptr_->railEverReceived(i)) {
            continue;
        }
        any_data = true;
        const float current = diagnostics_ptr_->getRailCurrent(i);
        stat.add("Rail " + std::to_string(i + 1) + " Current (A)", current);

        if (current >= kRailCurrentErrorMin) {
            worst = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            worst_msg = "Current level on rail is dangerously high, shutting rover off now";
            fault_detected_ = true;
        }
    }

    if (!any_data) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No rail telemetry received yet");
        return;
    }
    stat.summary(worst, worst_msg);
}

void ProduceDiagnostics::checkRailPower(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    bool any_data = false;
    for (size_t i = 0; i < BAB::NUM_RAILS; ++i) {
        if (!diagnostics_ptr_->railEverReceived(i)) {
            continue;
        }
        any_data = true;
        stat.add("Rail " + std::to_string(i + 1) + " Power (W)",
                 diagnostics_ptr_->getRailPower(i));
    }

    if (!any_data) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No rail telemetry received yet");
        return;
    }
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Rail power level OK");
}

void ProduceDiagnostics::checkBatteryTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    uint8_t worst = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string worst_msg = "Battery temperature OK";
    bool any_data = false;

    for (size_t i = 0; i < BAB::NUM_BATTERIES; ++i) {
        if (!diagnostics_ptr_->batteryEverReceived(i)) {
            continue;
        }
        any_data = true;
        const float temp = diagnostics_ptr_->getBatteryTemp(i);
        stat.add("Battery " + std::to_string(i + 1) + " Temperature (C)", temp);
        if (temp >= kTempWarnMin && worst < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
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

void ProduceDiagnostics::checkRailTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    // Firmware does not transmit rail temperature; field is always 0.
    stat.add("Rail Temperature (C)", diagnostics_ptr_->getRailTemp());
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Rail temperature OK");
}

void ProduceDiagnostics::checkTCUTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    const float tcu_temp = diagnostics_ptr_->getTCUTemp();
    stat.add("TCU Temperature (C)", tcu_temp);
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "TCU temperature OK");
}

void ProduceDiagnostics::checkTCUStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    const std::string tcu_status = diagnostics_ptr_->getTCUStatus();
    stat.add("TCU Status", tcu_status);

    if (tcu_status == "TCU ON") {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "TCU operating normally");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "TCU status fault: " + tcu_status);
    }
}

void ProduceDiagnostics::checkRelayStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!ensureDiagnostics(stat)) {
        return;
    }

    bool all_closed = true;
    for (size_t i = 0; i < BAB::NUM_RELAYS; ++i) {
        const bool closed = diagnostics_ptr_->getRelayClosed(i);
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

void ProduceDiagnostics::DiagnosticsCallback() {
    updater_->force_update();

    if (!diagnostics_ptr_) {
        return;
    }

    if (fault_detected_ && !shutdown_sent_) {
        RCLCPP_ERROR(
            node_.get_logger(),
            "Overcurrent detected on BAB telemetry — shutdown command NOT sent "
            "(BAB command TX disabled until bench validation)");
        shutdown_sent_ = true;
    }
}
