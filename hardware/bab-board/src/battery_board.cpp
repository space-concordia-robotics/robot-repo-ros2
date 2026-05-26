#include "bab-board/battery_board.hpp"

#include <cstring>

#include "can_util/can_id_util.hpp"

static constexpr auto AUTOMATIC_RAIL_SHUTDOWN = 0x02;

static constexpr auto BATTERY_TELEM = 0x00;
static constexpr auto RAIL_TELEM = 0x02;
static constexpr auto TCU_TELEM = 0x03;
static constexpr auto RELAY_STATUS = 0x08;
static constexpr auto TCU_STATUS = 0x0A;

// CAN-ID fields used by BAB firmware for both telemetry TX and command RX
// (see src/can-integration/docs/BAB-docs copy.md / Firmware/BAB_MX).
static constexpr uint8_t BAB_FIRMWARE_DEVTYPE = 0x00;
static constexpr uint8_t BAB_FIRMWARE_MFR = can_util::constants::Manufacturer::TEAM_USE; // 0x08 (CAN_MFR_SCC)
static constexpr uint8_t BAB_FIRMWARE_DEVICE_ID = 0x00;

// Firmware DATA_SELECT_1 / DATA_SELECT_2 (command payload words, not telemetry indices).
// 0x000F → PDS rail 1 (arm / CH2). 0x00F0 → PDS rail 2 (wheel / CH3).
// CH1 (5 V) is telemetry index 0 only; firmware has no CAN command for it.
static constexpr uint16_t DATA_SELECT_ARM_RAIL = 0x000F;
static constexpr uint16_t DATA_SELECT_WHEEL_RAIL = 0x00F0;

// BAB command TX is off until bench-validated on hardware.
static constexpr bool BAB_COMMAND_TX_ENABLED = false;

// getBMSHealth(): above min but below max → critically low, not a dead sensor.
static constexpr float BMS_CRITICAL_LOW_VOLTAGE_V = 10.0f;
static constexpr float BMS_MIN_VALID_VOLTAGE_V = 0.5f;

// Pack the first N bytes of a CAN payload into a 64-bit integer in
// MSB-first (network) order, matching the bit layouts in BAB-docs.md.
uint64_t packBigEndian(const std::vector<uint8_t>& data, const size_t n) {
    uint64_t out = 0;
    const size_t limit = std::min(n, data.size());
    for (size_t i = 0; i < limit; ++i) {
        out = out << 8 | data[i];
    }
    return out;
}


BAB::BAB(rclcpp::Logger logger, can_util::CANController::SharedPtr& can_controller, const uint32_t device_id)
    : logger(logger.get_child("battery_arbiter_board")),
      can_controller(can_controller),
      device_id(device_id) {
    frame_callback = can_controller->registerFrameCallback(
        [this](const uint32_t frameid, const std::vector<uint8_t>& data) {
            handleFrames(frameid, data);
        }
    );
}

uint32_t BAB::validateFrameID(const uint8_t sev, const uint8_t cmd) {
    return can_util::createCANFrameId(
        BAB_FIRMWARE_DEVTYPE,
        BAB_FIRMWARE_MFR,
        sev,
        cmd,
        BAB_FIRMWARE_DEVICE_ID
    );
}

void BAB::handleFrames(const uint32_t id, const std::vector<uint8_t>& data) {
    // Quick reject: ignore frames whose DeviceType field is not what the BAB
    // firmware emits today (0x00). Saves a few comparisons on a busy bus.
    if (const uint8_t devtype = id >> 24 & 0x1F; devtype != BAB_FIRMWARE_DEVTYPE) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    std::lock_guard lock(mtx);

    // -------------- Automatic PDS rail shutdown (SEV_AUTO, DLC 1) ------------
    if (id == validateFrameID(static_cast<uint8_t>(can_util::constants::Severity::MANUAL_INTERVENTION), AUTOMATIC_RAIL_SHUTDOWN)) {
        if (data.empty()) {
            return;
        }
        const uint8_t reason = data[0] & 0x03;
        // TODO 2026-05-26 (Will Free): this ternary sucks
        const char* reason_str = reason == 0x01
                                     ? "arm rail"
                                     : reason == 0x02
                                     ? "wheel rail"
                                     : "all rails";
        logger.warn("BAB automatic PDS rail shutdown (reason={:#02X}, {})", reason, reason_str);
        return;
    }

    // ---------------------- Battery telemetry (DLC 4) ----------------------
    if (id == validateFrameID(static_cast<uint8_t>(can_util::constants::Severity::STATUS), BATTERY_TELEM)) {
        const uint32_t payload = static_cast<uint32_t>(packBigEndian(data, 4));

        const size_t bat_idx = payload >> 31 & 0x01;
        const float voltage = static_cast<float>(payload >> 20 & 0x7FF) / VOLTAGE_MULTIPLIER;
        const float current = static_cast<float>(payload >> 6 & 0x3FFF) / CURRENT_MULTIPLIER;
        const float temp_c = static_cast<float>(payload & 0x3F) + 20.0f;

        if (bat_idx < BATTERIES_COUNT) {
            auto& bat = battery_telemetry[bat_idx];
            bat.batteries = static_cast<int>(bat_idx) + 1;
            bat.voltage = voltage;
            bat.current = current;
            bat.temperature = temp_c;
            bat.timestamp = now;
            bat.has_received = true;
        }
        return;
    }

    // ----------------------- Rail telemetry (DLC 6) ------------------------
    if (id == validateFrameID(static_cast<uint8_t>(can_util::constants::Severity::STATUS), RAIL_TELEM)) {
        const uint64_t payload = packBigEndian(data, 6);

        const size_t rail_idx = payload >> 42 & 0x03;
        const bool switch_on = (payload >> 41 & 0x01) != 0;
        const float voltage = static_cast<float>(payload >> 30 & 0x7FF) / VOLTAGE_MULTIPLIER;
        const float current = static_cast<float>(payload >> 16 & 0x3FFF) / CURRENT_MULTIPLIER;
        const float power = static_cast<float>(payload & 0xFFFF) / POWER_MULTIPLIER;

        if (rail_idx < RAILS_COUNT) {
            auto& rail = rail_telemetry[rail_idx];
            rail.rail_id = static_cast<int>(rail_idx) + 1;
            rail.active = switch_on;
            rail.voltage = voltage;
            rail.current = current;
            rail.power = power;
            rail.timestamp = now;
            rail.has_received = true;
        }
        return;
    }

    // ------------------------ Relay status (DLC 1) -------------------------
    if (id == validateFrameID(static_cast<uint8_t>(can_util::constants::Severity::STATUS), RELAY_STATUS)) {
        if (data.empty()) {
            return;
        }
        const uint8_t b = data[0];
        const size_t relay_idx = b & 0x01;
        const bool closed = (b >> 1 & 0x01) != 0;

        if (relay_idx < RELAYS_COUNT) {
            relay_telemetry[relay_idx].relay_id = static_cast<int>(relay_idx) + 1;
            relay_telemetry[relay_idx].active = closed;
        }
        return;
    }

    // ----------------------- TCU temperature (DLC 4) -----------------------
    if (id == validateFrameID(static_cast<uint8_t>(can_util::constants::Severity::STATUS), TCU_TELEM)) {
        if (data.size() < sizeof(float)) {
            return;
        }
        float temp;
        std::memcpy(&temp, data.data(), sizeof(float));
        tcu_telemetry.temperature = temp;
        return;
    }

    // ----------------------- TCU fan status (DLC 1) ------------------------
    if (id == validateFrameID(static_cast<uint8_t>(can_util::constants::Severity::STATUS), TCU_STATUS)) {
        if (data.empty()) {
            return;
        }
        tcu_telemetry.fan_enabled = (data[0] & 0x01) != 0;
        return;
    }
}

bool BAB::sendBabControlFrame(const uint8_t inst, uint16_t data_word) const {
    if constexpr (!BAB_COMMAND_TX_ENABLED) {
        logger.warn(
            "BAB control command suppressed (tx disabled until tested): instr={:#02X} word={:#04X}",
            static_cast<uint32_t>(inst), data_word
        );
        return false;
    }
    const uint32_t can_id = validateFrameID(static_cast<uint8_t>(can_util::constants::Severity::CONTROL), inst);
    const std::array payload = {
        static_cast<uint8_t>(data_word >> 8 & 0xFF),
        static_cast<uint8_t>(data_word & 0xFF),
    };
    return can_controller->sendBlockingFrame(can_id, payload);
}

bool BAB::sendBabEmergencyFrame(const uint8_t inst) const {
    if constexpr (!BAB_COMMAND_TX_ENABLED) {
        logger.warn(
            "BAB emergency command suppressed (tx disabled until tested): instr={:#02X}",
            static_cast<uint32_t>(inst)
        );
        return false;
    }
    const uint32_t can_id = validateFrameID(static_cast<uint8_t>(can_util::constants::Severity::MANUAL_INTERVENTION), inst);
    // DATA_EMERG_*_TOKEN = 0x00000000 (big-endian)
    return can_controller->sendBlockingFrame(can_id, std::array<uint8_t, 4>{0, 0, 0, 0});
}

// ---------------------------- Battery getters ----------------------------

float BAB::getBatteryVoltageLevel(const size_t idx) const {
    std::lock_guard lock(mtx);
    return idx < BATTERIES_COUNT ? battery_telemetry[idx].voltage : 0.f;
}

float BAB::getBatteryCurrentLevel(const size_t idx) const {
    std::lock_guard lock(mtx);
    return idx < BATTERIES_COUNT ? battery_telemetry[idx].current : 0.f;
}

float BAB::getBatteryTemp(const size_t idx) const {
    std::lock_guard lock(mtx);
    return idx < BATTERIES_COUNT ? battery_telemetry[idx].temperature : 0.f;
}

bool BAB::batteryFresh(const size_t idx, const std::chrono::milliseconds max_age) const {
    std::lock_guard lock(mtx);
    if (idx >= BATTERIES_COUNT || !battery_telemetry[idx].has_received) {
        return false;
    }
    const auto age = std::chrono::steady_clock::now() - battery_telemetry[idx].timestamp;
    return age <= max_age;
}

bool BAB::batteryEverReceived(const size_t idx) const {
    std::lock_guard lock(mtx);
    return idx < BATTERIES_COUNT && battery_telemetry[idx].has_received;
}

// ------------------------------ Rail getters -----------------------------

float BAB::getRailVoltageLevel(const size_t idx) const {
    std::lock_guard lock(mtx);
    return idx < RAILS_COUNT ? rail_telemetry[idx].voltage : 0.f;
}

float BAB::getRailCurrent(const size_t idx) const {
    std::lock_guard lock(mtx);
    return idx < RAILS_COUNT ? rail_telemetry[idx].current : 0.f;
}

float BAB::getRailPower(const size_t idx) const {
    std::lock_guard lock(mtx);
    return idx < RAILS_COUNT ? rail_telemetry[idx].power : 0.f;
}

float BAB::getRailTemp(const size_t idx) const {
    // Firmware does not currently transmit rail temperature; always 0.
    std::lock_guard lock(mtx);
    return idx < RAILS_COUNT ? rail_telemetry[idx].temperature : 0.f;
}

bool BAB::getRailSwitchOn(const size_t idx) const {
    std::lock_guard lock(mtx);
    return idx < RAILS_COUNT && rail_telemetry[idx].active;
}

bool BAB::railFresh(const size_t idx, const std::chrono::milliseconds max_age) const {
    std::lock_guard lock(mtx);
    if (idx >= RAILS_COUNT || !rail_telemetry[idx].has_received) {
        return false;
    }
    const auto age = std::chrono::steady_clock::now() - rail_telemetry[idx].timestamp;
    return age <= max_age;
}

bool BAB::railEverReceived(const size_t idx) const {
    std::lock_guard lock(mtx);
    return idx < RAILS_COUNT && rail_telemetry[idx].has_received;
}

// ------------------------------ TCU / relay ------------------------------

float BAB::getTCUTemp() const {
    std::lock_guard lock(mtx);
    return tcu_telemetry.temperature;
}

std::string BAB::getTCUStatus() const {
    std::lock_guard lock(mtx);
    return tcu_telemetry.fan_enabled ? "TCU ON" : "TCU OFF";
}

std::string BAB::getBMSHealth() const {
    std::lock_guard lock(mtx);
    const float v = battery_telemetry[0].voltage;
    if (v < BMS_CRITICAL_LOW_VOLTAGE_V && v > BMS_MIN_VALID_VOLTAGE_V) {
        return "CRITICAL LOW VOLTAGE";
    }
    return "NORMAL VOLTAGE (HEALTHY)";
}

std::string BAB::getRelayStatus(const size_t idx) const {
    std::lock_guard lock(mtx);
    if (idx >= RELAYS_COUNT) {
        return "Relay unknown";
    }
    return relay_telemetry[idx].active ? "Relay Closed (ON)" : "Relay OPEN (OFF)";
}

bool BAB::getRelayClosed(const size_t idx) const {
    std::lock_guard lock(mtx);
    return idx < RELAYS_COUNT && relay_telemetry[idx].active;
}

// ------------------------------ Commands ---------------------------------

bool BAB::sendKYSCommand() {
    return sendBabEmergencyFrame(CUT_PDS_OUTPUTS);
}

bool BAB::cutFanPower(uint8_t /*fanID*/) {
    return sendBabControlFrame(TURN_OFF_FAN, 0x0000);
}

bool BAB::CutRelayCommand(uint8_t relayID) {
    const uint16_t select = relayID == DeviceId::ID::JMSB ? DATA_SELECT_ARM_RAIL : DATA_SELECT_WHEEL_RAIL;
    return sendBabControlFrame(TURN_OFF_RELAY, select);
}

bool BAB::sendManualPowerCommands(uint8_t selectRailID, bool turnOn) {
    const Instructions::Inst inst =
        turnOn ? COMMAND_ON : COMMAND_OFF;
    const uint16_t select =
        selectRailID == DeviceId::ID::ARM_EMERGENCY_INTERVENTION
            ? DATA_SELECT_ARM_RAIL
            : DATA_SELECT_WHEEL_RAIL;
    return sendBabControlFrame(inst, select);
}
