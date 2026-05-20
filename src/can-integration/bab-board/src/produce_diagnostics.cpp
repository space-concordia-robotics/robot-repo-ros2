#include "bab-board/produce_diagnostics.hpp"

#include <functional>
#include <utility>

namespace {
constexpr double kUpdatePeriodSec = 10.0;
constexpr auto kUpdatePeriod = std::chrono::seconds(10);

constexpr float kBatteryVoltMin = 10.0f;
constexpr float kBatteryVoltMax = 14.8f;
constexpr float kRailVoltMin = 10.0f;
constexpr float kRailVoltMax = 14.8f;

constexpr float kBatteryCurrentWarnMin = 60.0f;
constexpr float kBatteryCurrentErrorMin = 80.0f;
constexpr float kBatteryCurrentFailMin = 90.0f;

constexpr float kRailCurrentErrorMin = 30.0f;
constexpr float kTempWarnMin = 60.0f;
}

ProduceDiagnostics::ProduceDiagnostics(std::shared_ptr<BAB> bab_ptr)
	: Node("produce_diagnostic_node"),
	  updater_(this),
	  diagnostics_ptr(std::move(bab_ptr)) {
	updater_.setHardwareID("Rover-PowerBoard");

	updater_.add("Battery Voltage Status", this, &ProduceDiagnostics::checkBatteryVoltage);
	updater_.add("Rail Voltage Status", this, &ProduceDiagnostics::checkRailVoltage);
	updater_.add("Battery Current Status", this, &ProduceDiagnostics::checkBatteryCurrent);
	updater_.add("Rail Current Status", this, &ProduceDiagnostics::checkRailCurrent);
	updater_.add("Rail Power Status", this, &ProduceDiagnostics::checkRailPower);
	updater_.add("Battery Temperature Status", this, &ProduceDiagnostics::checkBatteryTemperature);
	updater_.add("Rail Temperature Status", this, &ProduceDiagnostics::checkRailTemperature);
	updater_.add("TCU Temperature Status", this, &ProduceDiagnostics::checkTCUTemperature);
	updater_.add("TCU Module Status", this, &ProduceDiagnostics::checkTCUStatus);
	updater_.add("Relay Module Status", this, &ProduceDiagnostics::checkRelayStatus);

	updater_.setPeriod(kUpdatePeriodSec);

	clock_ = this->create_wall_timer(
		kUpdatePeriod,
		std::bind(&ProduceDiagnostics::DiagnosticsCallback, this));
}

bool ProduceDiagnostics::ensureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) const {
	if (diagnostics_ptr) {
		return true;
	}

	stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "BAB interface not initialized");
	return false;
}

void ProduceDiagnostics::checkBatteryVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	if (!ensureDiagnostics(stat)) {
		return;
	}

	const float battery_voltage = diagnostics_ptr->getBatteryVoltageLevel();
	stat.add("Battery Voltage Level (V)", battery_voltage);

	if (battery_voltage >= kBatteryVoltMin && battery_voltage <= kBatteryVoltMax) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery levels OK");
	} else if (battery_voltage > kBatteryVoltMax) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
					 "Battery voltage exceeding criticality, shut rover off immediately");
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Battery voltage low, replace batteries");
	}
}

void ProduceDiagnostics::checkRailVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	if (!ensureDiagnostics(stat)) {
		return;
	}

	const float rail_voltage = diagnostics_ptr->getRailVoltageLevel();
	stat.add("Rail Voltage Level (V)", rail_voltage);

	if (rail_voltage >= kRailVoltMin && rail_voltage <= kRailVoltMax) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Rail voltage levels OK");
	} else if (rail_voltage > kRailVoltMax) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
					 "Rail voltage exceeding criticality, shut rover off immediately");
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
					 "Rail voltages reaching low levels, battery may be dying");
	}
}

void ProduceDiagnostics::checkBatteryCurrent(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	if (!ensureDiagnostics(stat)) {
		return;
	}

	const float battery_current = diagnostics_ptr->getBatteryCurrentLevel();
	stat.add("Battery Current Level (A)", battery_current);

	if (battery_current >= kBatteryCurrentFailMin) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
					 "Battery current draw so high, potential failure detected");
		fault_detected_ = true;
	} else if (battery_current >= kBatteryCurrentErrorMin) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
					 "Battery current draw exceeding high levels, shutting rover off");
		fault_detected_ = true;
	} else if (battery_current >= kBatteryCurrentWarnMin) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Battery current draw is high");
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery current draw OK");
	}
}

void ProduceDiagnostics::checkRailCurrent(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	if (!ensureDiagnostics(stat)) {
		return;
	}

	const float rail_current = diagnostics_ptr->getRailCurrent();
	stat.add("Rail Current Level (A)", rail_current);

	if (rail_current >= kRailCurrentErrorMin) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
					 "Current level on rail is dangerously high, shutting rover off now");
		fault_detected_ = true;
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Rail current levels OK");
	}
}

void ProduceDiagnostics::checkRailPower(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	if (!ensureDiagnostics(stat)) {
		return;
	}

	const float rail_power = diagnostics_ptr->getRailPower();
	stat.add("Rail Power Level (W)", rail_power);
	stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Rail power level OK");
}

void ProduceDiagnostics::checkBatteryTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	if (!ensureDiagnostics(stat)) {
		return;
	}

	const float battery_temp = diagnostics_ptr->getBatteryTemp();
	stat.add("Battery Temperature (C)", battery_temp);

	if (battery_temp >= kTempWarnMin) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Warning, battery is overheating");
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery temperature OK");
	}
}

void ProduceDiagnostics::checkRailTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	if (!ensureDiagnostics(stat)) {
		return;
	}

	const float rail_temp = diagnostics_ptr->getRailTemp();
	stat.add("Rail Temperature (C)", rail_temp);

	if (rail_temp >= kTempWarnMin) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Warning, rails are overheating");
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Rail temperature OK");
	}
}

void ProduceDiagnostics::checkTCUTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	if (!ensureDiagnostics(stat)) {
		return;
	}

	const float tcu_temp = diagnostics_ptr->getTCUTemp();
	stat.add("TCU Temperature (C)", tcu_temp);
	stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "TCU temperature OK");
}

void ProduceDiagnostics::checkTCUStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	if (!ensureDiagnostics(stat)) {
		return;
	}

	const std::string tcu_status = diagnostics_ptr->getTCUStatus();
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

	const std::string relay_status = diagnostics_ptr->getRelayStatus();
	stat.add("Relay Status", relay_status);

	if (relay_status == "Relay Closed (ON)") {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Relay operating normally");
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Relay status fault: " + relay_status);
	}
}

void ProduceDiagnostics::DiagnosticsCallback() {
	updater_.force_update();

	if (!diagnostics_ptr) {
		return;
	}

	if (fault_detected_ && !shutdown_sent_) {
		RCLCPP_FATAL(this->get_logger(), "Overcurrent confirmed - sending shutdown");
		diagnostics_ptr->sendKYSCommand();
		shutdown_sent_ = true;
	}
}