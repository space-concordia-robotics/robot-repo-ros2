#pragma once

#include "can-utils/prefixes.hpp"
#include "can-utils/parser.hpp"
#include "can-utils/buildAddress.hpp"
#include <array>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>


#define VOLTAGE_MULTIPLIER 100.0f
#define CURRENT_MULTIPLIER 100.0f
#define POWER_MULTIPLIER 10.0f

struct BatteryTelem {
    int BatteryNum{0};
    float voltage{0.f};
    float temperature{0.f};
    float current{0.f};
    std::chrono::steady_clock::time_point timestamp{};
    bool ever_received{false};
};

struct RailTelem {
    int RailNum{0};
    bool status{false};
    float voltage{0.f};
    float temperature{0.f};
    float current{0.f};
    float power{0.f};
    std::chrono::steady_clock::time_point timestamp{};
    bool ever_received{false};
};

struct TCUtelem {
    bool fan_status{false};
    float temperature{0.f};
};

struct RelayTelem {
    int RelayNum{0};
    bool status{false};
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
    static constexpr size_t NUM_BATTERIES = 2;
    static constexpr size_t NUM_RAILS = 3;
    static constexpr size_t NUM_RELAYS = 2;

    BAB(rclcpp::Logger logger,
        can_util::CANController & can_controller_,
        buildAddress::BuildAddress & build_frame,
        uint32_t deviceID);

    /// Build the expected 29-bit CAN ID for a BAB telemetry frame.
    /// Uses the manufacturer / device-ID values documented in BAB-docs.md
    /// (Manufacturer::SCC, DeviceID 0x01) rather than the generic TEAM_USE.
    uint32_t validateFrameID(uint32_t sev, Instructions::Inst cmd) const;

    /// Frame callback invoked by `CANController::registerFrameCallback`.
    /// Decodes battery/rail/TCU/relay frames in-place.
    void handleFrames(uint32_t id, const std::vector<uint8_t> & data);

    // ----------------------- Battery accessors -----------------------
    // idx in [0, NUM_BATTERIES). Default 0 keeps legacy callers working.
    float getBatteryVoltageLevel(size_t idx = 0) const;
    float getBatteryCurrentLevel(size_t idx = 0) const;
    float getBatteryTemp(size_t idx = 0) const;
    bool  batteryFresh(size_t idx = 0,
                       std::chrono::milliseconds max_age = std::chrono::seconds(3)) const;
    bool  batteryEverReceived(size_t idx = 0) const;

    // ------------------------ Rail accessors -------------------------
    float getRailVoltageLevel(size_t idx = 0) const;
    float getRailCurrent(size_t idx = 0) const;
    float getRailPower(size_t idx = 0) const;
    float getRailTemp(size_t idx = 0) const;  // firmware does not send; returns 0
    bool  getRailSwitchOn(size_t idx = 0) const;
    bool  railFresh(size_t idx = 0,
                    std::chrono::milliseconds max_age = std::chrono::seconds(3)) const;
    bool  railEverReceived(size_t idx = 0) const;

    // ------------------------ TCU / relay ----------------------------
    float getTCUTemp() const;
    std::string getTCUStatus() const;
    std::string getRelayStatus() const;
    std::string getBMSHealth() const;

    // ----------------------- Command emitters ------------------------
    bool sendKYSCommand();
    bool cutFanPower(DeviceId::ID fanID);
    bool CutRelayCommand(DeviceId::ID relayID);
    bool sendManualPowerCommands(DeviceId::ID selectRailID, bool turnOn);

private:
    mutable std::mutex mtx;
    ros2_fmt_logger::Logger logger;
    can_util::CANController & can_controller;
    buildAddress::BuildAddress & build_frame;
    uint32_t deviceId;
    std::shared_ptr<can_util::CANFrameCallback> frame_callback;

    std::array<BatteryTelem, NUM_BATTERIES> batteryTelems{};
    std::array<RailTelem,    NUM_RAILS>     railTelems{};
    std::array<RelayTelem,   NUM_RELAYS>    relayTelems{};
    TCUtelem tcuTelem{};
};
