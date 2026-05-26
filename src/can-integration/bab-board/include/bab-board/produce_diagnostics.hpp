#pragma once
#include "bab-board/battery_board.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

class ProduceDiagnostics {
    public:
        ProduceDiagnostics(rclcpp::Node& node,
                           std::shared_ptr<BAB> bab_ptr,
                           rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);

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

        void diagnosticsCallback();

        rclcpp::Node& node;
        std::shared_ptr<diagnostic_updater::Updater> updater;
        rclcpp::TimerBase::SharedPtr diagnostics_timer;
        std::shared_ptr<BAB> bab;
        std::atomic<bool> fault_detected = false;
        std::atomic<bool> shutdown_sent = false;
};