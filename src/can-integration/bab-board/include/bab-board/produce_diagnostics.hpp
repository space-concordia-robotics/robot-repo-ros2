#pragma once
#include "bab-board/battery_board.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <string>

class ProduceDiagnostics : public rclcpp::Node {
    public:
        explicit ProduceDiagnostics(std::shared_ptr<BAB> bab_ptr);

    private:
        bool ensureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) const;

        void checkBatteryVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat);
        void checkRailVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat);
        void checkBatteryCurrent(diagnostic_updater::DiagnosticStatusWrapper& stat);
        void checkRailCurrent(diagnostic_updater::DiagnosticStatusWrapper& stat);
        void checkRailPower(diagnostic_updater::DiagnosticStatusWrapper& stat);
        void checkBatteryTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat);
        void checkRailTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat);
        void checkTCUTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat);
        void checkTCUStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);
        void checkRelayStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

        void DiagnosticsCallback();

        diagnostic_updater::Updater updater_;
        rclcpp::TimerBase::SharedPtr clock_;
        std::shared_ptr<BAB> diagnostics_ptr;
        bool fault_detected_ = false;
        bool shutdown_sent_ = false;
};