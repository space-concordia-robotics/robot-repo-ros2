#include "bab-board/battery_board.hpp"

#include <cstring>

namespace {

// CAN-ID fields used by BAB firmware for both telemetry TX and command RX
// (see src/can-integration/docs/BAB-docs copy.md / Firmware/BAB_MX).
static constexpr uint8_t BAB_FIRMWARE_DEVTYPE   = 0x00;
static constexpr uint8_t BAB_FIRMWARE_MFR       = Manufacturer::TEAM_USE;  // 0x08 (CAN_MFR_SCC)
static constexpr uint8_t BAB_FIRMWARE_DEVICE_ID = 0x00;

// Firmware DATA_SELECT_1 / DATA_SELECT_2 (command payload words, not telemetry indices).
// 0x000F → PDS rail 1 (arm / CH2). 0x00F0 → PDS rail 2 (wheel / CH3).
// CH1 (5 V) is telemetry index 0 only; firmware has no CAN command for it.
static constexpr uint16_t DATA_SELECT_ARM_RAIL   = 0x000F;
static constexpr uint16_t DATA_SELECT_WHEEL_RAIL = 0x00F0;

// BAB command TX is off until bench-validated on hardware.
static constexpr bool BAB_COMMAND_TX_ENABLED = false;

// getBMSHealth(): above min but below max → critically low, not a dead sensor.
static constexpr float BMS_CRITICAL_LOW_VOLTAGE_V = 10.0f;
static constexpr float BMS_MIN_VALID_VOLTAGE_V = 0.5f;

// Pack the first N bytes of a CAN payload into a 64-bit integer in
// MSB-first (network) order, matching the bit layouts in BAB-docs.md.
uint64_t packBigEndian(const std::vector<uint8_t> & data, size_t n) {
    uint64_t out = 0;
    const size_t limit = std::min(n, data.size());
    for (size_t i = 0; i < limit; ++i) {
        out = (out << 8) | data[i];
    }
    return out;
}

}  // namespace


BAB::BAB(rclcpp::Logger logger_,
         can_util::CANController & can_controller_,
         buildAddress::BuildAddress & build_frame_,
         uint32_t deviceID)
    : logger(logger_.get_child("battery_arbiter_board")),
      can_controller(can_controller_),
      build_frame(build_frame_),
      deviceId(deviceID) {

    frame_callback = can_controller.registerFrameCallback(
        [this](uint32_t frameid, const std::vector<uint8_t> & data) {
            handleFrames(frameid, data);
        });
}

uint32_t BAB::validateFrameID(uint32_t sev, Instructions::Inst cmd) const {
    return buildAddress::BuildAddress::buildCANID(
        BAB_FIRMWARE_DEVTYPE,
        BAB_FIRMWARE_MFR,
        sev,
        static_cast<uint32_t>(cmd),
        BAB_FIRMWARE_DEVICE_ID);
}

bool BAB::sendBabControlFrame(Instructions::Inst inst, uint16_t data_word) {
    if (!kBabCommandTxEnabled) {
        logger.warn(
            "BAB control command suppressed (tx disabled until tested): instr={:#02X} word={:#04X}",
            static_cast<uint32_t>(inst), data_word);
        return false;
    }
    const uint32_t can_id = validateFrameID(severity::SEV_CNTRL, inst);
    const std::array<uint8_t, 2> payload = {
        static_cast<uint8_t>((data_word >> 8) & 0xFF),
        static_cast<uint8_t>(data_word & 0xFF),
    };
    return can_controller.sendBlockingFrame(can_id, payload);
}

bool BAB::sendBabEmergencyFrame(Instructions::Inst inst) {
    if (!kBabCommandTxEnabled) {
        logger.warn(
            "BAB emergency command suppressed (tx disabled until tested): instr={:#02X}",
            static_cast<uint32_t>(inst));
        return false;
    }
    const uint32_t can_id = validateFrameID(severity::SEV_MAN_INTERVENTION, inst);
    // DATA_EMERG_*_TOKEN = 0x00000000 (big-endian)
    return can_controller.sendBlockingFrame(can_id, std::array<uint8_t, 4>{0, 0, 0, 0});
}

void BAB::handleFrames(const uint32_t id, const std::vector<uint8_t> & data) {
    // Quick reject: ignore frames whose DeviceType field is not what the BAB
    // firmware emits today (0x00). Saves a few comparisons on a busy bus.
    const uint8_t devtype = (id >> 24) & 0x1F;
    if (devtype != BAB_FIRMWARE_DEVTYPE) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mtx);

    // -------------- Automatic PDS rail shutdown (SEV_AUTO, DLC 1) ------------
    if (id == validateFrameID(severity::SEV_AUTOMATIC_INTERVENTION,
                             Instructions::Inst::AUTOMATIC_RAIL_SHUTDOWN)) {
        if (data.empty()) {
            return;
        }
        const uint8_t reason = data[0] & 0x03;
        const char * reason_str = (reason == 0x01) ? "arm rail"
                                : (reason == 0x02) ? "wheel rail"
                                : "all rails";
        logger.warn("BAB automatic PDS rail shutdown (reason={:#02X}, {})",
                    reason, reason_str);
        return;
    }

    // ---------------------- Battery telemetry (DLC 4) ----------------------
    if (id == validateFrameID(severity::SEV_STATUS, Instructions::Inst::BATTERY_TELEM)) {
        const uint32_t payload = static_cast<uint32_t>(packBigEndian(data, 4));

        const size_t bat_idx = (payload >> 31) & 0x01;
        const float voltage  = static_cast<float>((payload >> 20) & 0x7FF) / VOLTAGE_MULTIPLIER;
        const float current  = static_cast<float>((payload >>  6) & 0x3FFF) / CURRENT_MULTIPLIER;
        const float temp_c   = static_cast<float>(payload & 0x3F) + 20.0f;

        if (bat_idx < NUM_BATTERIES) {
            auto & bat = batteryTelems[bat_idx];
            bat.BatteryNum    = static_cast<int>(bat_idx) + 1;
            bat.voltage       = voltage;
            bat.current       = current;
            bat.temperature   = temp_c;
            bat.timestamp     = now;
            bat.ever_received = true;
        }
        return;
    }

    // ----------------------- Rail telemetry (DLC 6) ------------------------
    if (id == validateFrameID(severity::SEV_STATUS, Instructions::Inst::RAIL_TELEM)) {
        const uint64_t payload = packBigEndian(data, 6);

        const size_t rail_idx = (payload >> 42) & 0x03;
        const bool  switch_on = ((payload >> 41) & 0x01) != 0;
        const float voltage   = static_cast<float>((payload >> 30) & 0x7FF) / VOLTAGE_MULTIPLIER;
        const float current   = static_cast<float>((payload >> 16) & 0x3FFF) / CURRENT_MULTIPLIER;
        const float power     = static_cast<float>(payload & 0xFFFF) / POWER_MULTIPLIER;

        if (rail_idx < NUM_RAILS) {
            auto & rail = railTelems[rail_idx];
            rail.RailNum       = static_cast<int>(rail_idx) + 1;
            rail.status        = switch_on;
            rail.voltage       = voltage;
            rail.current       = current;
            rail.power         = power;
            rail.timestamp     = now;
            rail.ever_received = true;
        }
        return;
    }

    // ------------------------ Relay status (DLC 1) -------------------------
    if (id == validateFrameID(severity::SEV_STATUS, Instructions::Inst::RELAY_STATUS)) {
        if (data.empty()) {
            return;
        }
        const uint8_t b = data[0];
        const size_t relay_idx = b & 0x01;
        const bool closed = ((b >> 1) & 0x01) != 0;

        if (relay_idx < NUM_RELAYS) {
            relayTelems[relay_idx].RelayNum = static_cast<int>(relay_idx) + 1;
            relayTelems[relay_idx].status   = closed;
        }
        return;
    }

    // ----------------------- TCU temperature (DLC 4) -----------------------
    if (id == validateFrameID(severity::SEV_STATUS, Instructions::Inst::TCU_TELEM)) {
        if (data.size() < sizeof(float)) {
            return;
        }
        float temp;
        std::memcpy(&temp, data.data(), sizeof(float));
        tcuTelem.temperature = temp;
        return;
    }

    // ----------------------- TCU fan status (DLC 1) ------------------------
    if (id == validateFrameID(severity::SEV_STATUS, Instructions::Inst::TCU_STATUS)) {
        if (data.empty()) {
            return;
        }
        tcuTelem.fan_status = (data[0] & 0x01) != 0;
        return;
    }
}

// ---------------------------- Battery getters ----------------------------

float BAB::getBatteryVoltageLevel(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_BATTERIES ? batteryTelems[idx].voltage : 0.f;
}

float BAB::getBatteryCurrentLevel(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_BATTERIES ? batteryTelems[idx].current : 0.f;
}

float BAB::getBatteryTemp(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_BATTERIES ? batteryTelems[idx].temperature : 0.f;
}

bool BAB::batteryFresh(size_t idx, std::chrono::milliseconds max_age) const {
    std::lock_guard<std::mutex> lock(mtx);
    if (idx >= NUM_BATTERIES || !batteryTelems[idx].ever_received) {
        return false;
    }
    const auto age = std::chrono::steady_clock::now() - batteryTelems[idx].timestamp;
    return age <= max_age;
}

bool BAB::batteryEverReceived(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_BATTERIES && batteryTelems[idx].ever_received;
}

// ------------------------------ Rail getters -----------------------------

float BAB::getRailVoltageLevel(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_RAILS ? railTelems[idx].voltage : 0.f;
}

float BAB::getRailCurrent(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_RAILS ? railTelems[idx].current : 0.f;
}

float BAB::getRailPower(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_RAILS ? railTelems[idx].power : 0.f;
}

float BAB::getRailTemp(size_t idx) const {
    // Firmware does not currently transmit rail temperature; always 0.
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_RAILS ? railTelems[idx].temperature : 0.f;
}

bool BAB::getRailSwitchOn(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_RAILS && railTelems[idx].status;
}

bool BAB::railFresh(size_t idx, std::chrono::milliseconds max_age) const {
    std::lock_guard<std::mutex> lock(mtx);
    if (idx >= NUM_RAILS || !railTelems[idx].ever_received) {
        return false;
    }
    const auto age = std::chrono::steady_clock::now() - railTelems[idx].timestamp;
    return age <= max_age;
}

bool BAB::railEverReceived(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_RAILS && railTelems[idx].ever_received;
}

// ------------------------------ TCU / relay ------------------------------

float BAB::getTCUTemp() const {
    std::lock_guard<std::mutex> lock(mtx);
    return tcuTelem.temperature;
}

std::string BAB::getTCUStatus() const {
    std::lock_guard<std::mutex> lock(mtx);
    return tcuTelem.fan_status ? "TCU ON" : "TCU OFF";
}

std::string BAB::getBMSHealth() const {
    std::lock_guard<std::mutex> lock(mtx);
    const float v = batteryTelems[0].voltage;
    if (v < BMS_CRITICAL_LOW_VOLTAGE_V && v > BMS_MIN_VALID_VOLTAGE_V) {
        return "CRITICAL LOW VOLTAGE";
    }
    return "NORMAL VOLTAGE (HEALTHY)";
}

std::string BAB::getRelayStatus(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    if (idx >= NUM_RELAYS) {
        return "Relay unknown";
    }
    return relayTelems[idx].status ? "Relay Closed (ON)" : "Relay OPEN (OFF)";
}

bool BAB::getRelayClosed(size_t idx) const {
    std::lock_guard<std::mutex> lock(mtx);
    return idx < NUM_RELAYS && relayTelems[idx].status;
}

// ------------------------------ Commands ---------------------------------

bool BAB::sendKYSCommand() {
    return sendBabEmergencyFrame(Instructions::Inst::CUT_PDS_OUTPUTS);
}

bool BAB::cutFanPower(DeviceId::ID /*fanID*/) {
    return sendBabControlFrame(Instructions::Inst::TURN_OFF_FAN, 0x0000);
}

bool BAB::CutRelayCommand(DeviceId::ID relayID) {
    const uint16_t select =
        (relayID == DeviceId::ID::JMSB) ? DATA_SELECT_ARM_RAIL : DATA_SELECT_WHEEL_RAIL;
    return sendBabControlFrame(Instructions::Inst::TURN_OFF_RELAY, select);
}

bool BAB::sendManualPowerCommands(DeviceId::ID selectRailID, bool turnOn) {
    const Instructions::Inst inst =
        turnOn ? Instructions::Inst::COMMAND_ON : Instructions::Inst::COMMAND_OFF;
    const uint16_t select =
        (selectRailID == DeviceId::ID::ARM_EMERGENCY_INTERVENTION)
            ? DATA_SELECT_ARM_RAIL
            : DATA_SELECT_WHEEL_RAIL;
    return sendBabControlFrame(inst, select);
}
