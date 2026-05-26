#pragma once

#include "can-utils/prefixes.hpp"
#include "can-utils/parser.hpp"
#include "can-utils/buildAddress.hpp"
#include <array>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>


static constexpr auto VOLTAGE_MULTIPLIER = 100;
static constexpr auto CURRENT_MULTIPLIER = 100.0;
static constexpr auto POWER_MULTIPLIER = 10.0;

struct BatteryTelemetry {
    int batteries = 0;
    double voltage = 0.0;
    double temperature = 0.0;
    double current = 0.0;
    std::chrono::steady_clock::time_point timestamp;
    bool has_received = false;
};

struct RailTelemetry {
    int rail_id;
    bool active = false;
    double voltage = 0.0;
    double temperature = 0.0;
    double current = 0.0;
    double power = 0.0;
    std::chrono::steady_clock::time_point timestamp;
    bool has_received = false;
};

struct TCUTelemetry {
    bool fan_enabled = false;
    double temperature = 0.0;
};

struct RelayTelemetry {
    int relay_id;
    bool active = false;
};

/*
 * @class BAB
 * @brief Battery Arbiter Board telemetry decoder.
 *
 * Decodes CAN frames published by the BAB firmware (per
 * src/can-integration/docs/BAB-docs.md) and stores the latest values
 * per-battery (2 batteries) and per-rail (3 PDS rails) so callers can
 * query individual channels.
 *
 * Backwards compatibility: the legacy zero-argument getters
 * (e.g. `getBatteryVoltageLevel()`) return data for index 0
 * (battery 1 / rail 1).
 */
class BAB {
public:
    static constexpr auto BATTERIES_COUNT = 2;
    static constexpr auto RAILS_COUNT = 3;
    static constexpr auto RELAYS_COUNT = 2;

    BAB(rclcpp::Logger logger,
        can_util::CANController& can_controller_,
        buildAddress::BuildAddress& build_frame,
        uint32_t deviceID);

    /// Build the expected 29-bit CAN ID for a BAB frame (telemetry or command).
    /// Uses firmware field values DevType=0x00, Mfr=0x08 (TEAM_USE), DevID=0x00.
    uint32_t validateFrameID(uint32_t sev, Instructions::Inst cmd) const;

    /// Frame callback invoked by `CANController::registerFrameCallback`.
    /// Decodes battery/rail/TCU/relay frames in-place.
    void handleFrames(uint32_t id, const std::vector<uint8_t>& data);

    // ----------------------- Battery accessors -----------------------
    // idx in [0, NUM_BATTERIES). Default 0 keeps legacy callers working.
    float getBatteryVoltageLevel(size_t idx = 0) const;
    float getBatteryCurrentLevel(size_t idx = 0) const;
    float getBatteryTemp(size_t idx = 0) const;
    bool batteryFresh(size_t idx = 0,
                      std::chrono::milliseconds max_age = std::chrono::seconds(3)) const;
    bool batteryEverReceived(size_t idx = 0) const;

    // ------------------------ Rail accessors -------------------------
    float getRailVoltageLevel(size_t idx = 0) const;
    float getRailCurrent(size_t idx = 0) const;
    float getRailPower(size_t idx = 0) const;
    float getRailTemp(size_t idx = 0) const; // firmware does not send; returns 0
    bool getRailSwitchOn(size_t idx = 0) const;
    bool railFresh(size_t idx = 0,
                   std::chrono::milliseconds max_age = std::chrono::seconds(3)) const;
    bool railEverReceived(size_t idx = 0) const;

    // ------------------------ TCU / relay ----------------------------
    float getTCUTemp() const;
    std::string getTCUStatus() const;
    std::string getRelayStatus(size_t idx = 0) const;
    bool getRelayClosed(size_t idx) const;
    std::string getBMSHealth() const;

    // Command emitters (receive-only on bus until kBabCommandTxEnabled in
    // battery_board.cpp is set true after bench validation).
    bool sendKYSCommand();
    bool cutFanPower(DeviceId::ID fanID);
    bool CutRelayCommand(DeviceId::ID relayID);
    bool sendManualPowerCommands(DeviceId::ID selectRailID, bool turnOn);

private:
    bool sendBabControlFrame(Instructions::Inst inst, uint16_t data_word);
    bool sendBabEmergencyFrame(Instructions::Inst inst);

    mutable std::mutex mtx;
    ros2_fmt_logger::Logger logger;
    can_util::CANController& can_controller;
    buildAddress::BuildAddress& build_frame;
    uint32_t device_id;
    std::shared_ptr<can_util::CANFrameCallback> frame_callback;

    std::array<BatteryTelemetry, BATTERIES_COUNT> battery_telemetry = {};
    std::array<RailTelemetry, RAILS_COUNT> rail_telemetry = {};
    std::array<RelayTelemetry, RELAYS_COUNT> relay_telemetry = {};
    TCUTelemetry tcu_telemetry;
};
