#include "bab-board/battery_board.hpp"

#include <cstring>

namespace {

// CAN-ID fields actually used by the BAB firmware on the rover (confirmed via
// candump). These differ from BAB-docs.md, which describes an idealised
// protocol with DevType 0x01 / Manufacturer 0x01 (SCC) / DeviceID 0x01.
// Real frames observed on the bus:
//
//   ID 0x00088000  DevType=0x00  Mfr=0x08  Sev=0x02  Instr=0x00  DevID=0x00  Battery
//   ID 0x00088080  DevType=0x00  Mfr=0x08  Sev=0x02  Instr=0x02  DevID=0x00  Rail
//   ID 0x00088200  DevType=0x00  Mfr=0x08  Sev=0x02  Instr=0x08  DevID=0x00  Relay
//
// If firmware is updated to match the doc, change these to 0x01 / SCC / 0x01.
constexpr uint8_t BAB_FIRMWARE_DEVTYPE   = 0x00;                       // not DeviceType::BAB
constexpr uint8_t BAB_FIRMWARE_MFR       = Manufacturer::TEAM_USE;     // 0x08
constexpr uint8_t BAB_FIRMWARE_DEVICE_ID = 0x00;                       // DeviceId::ID::BAB

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
    // Use the field values actually emitted by the BAB firmware (see the
    // constants block above; they do not match BAB-docs.md as of this commit).
    return buildAddress::BuildAddress::buildCANID(
        BAB_FIRMWARE_DEVTYPE,
        BAB_FIRMWARE_MFR,
        sev,
        static_cast<uint32_t>(cmd),
        BAB_FIRMWARE_DEVICE_ID);
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
    if (v < 10.0f && v > 0.5f) {
        return "CRITICAL LOW VOLTAGE";
    }
    return "NORMAL VOLTAGE (HEALTHY)";
}

std::string BAB::getRelayStatus() const {
    std::lock_guard<std::mutex> lock(mtx);
    return relayTelems[0].status ? "Relay Closed (ON)" : "Relay OPEN (OFF)";
}

// ------------------------------ Commands ---------------------------------

bool BAB::sendKYSCommand() {
    uint16_t payload = 0x0000;
    return build_frame.buildAddress(
        static_cast<uint32_t>(deviceType::DeviceType::BAB),
        Manufacturer::TEAM_USE,
        severity::SEV_MAN_INTERVENTION,
        static_cast<uint32_t>(Instructions::Inst::CUT_PDS_OUTPUTS),
        static_cast<uint32_t>(DeviceId::ID::BAB),
        payload);
}

bool BAB::cutFanPower(DeviceId::ID fanID) {
    uint16_t payload = 0x0000;
    return build_frame.buildAddress(
        static_cast<uint32_t>(deviceType::DeviceType::BAB),
        Manufacturer::TEAM_USE,
        severity::SEV_CNTRL,
        static_cast<uint32_t>(Instructions::Inst::TURN_OFF_FAN),
        static_cast<uint32_t>(fanID),
        payload);
}

bool BAB::CutRelayCommand(DeviceId::ID relayID) {
    uint16_t payload = (relayID == DeviceId::ID::JMSB) ? 0x000F : 0x00F0;
    return build_frame.buildAddress(
        static_cast<uint32_t>(deviceType::DeviceType::BAB),
        Manufacturer::TEAM_USE,
        severity::SEV_CNTRL,
        static_cast<uint32_t>(Instructions::Inst::TURN_OFF_RELAY),
        static_cast<uint32_t>(relayID),
        payload);
}

bool BAB::sendManualPowerCommands(DeviceId::ID selectRailID, bool turnOn) {
    Instructions::Inst inst =
        turnOn ? Instructions::Inst::COMMAND_ON : Instructions::Inst::COMMAND_OFF;
    uint16_t payload_val =
        (selectRailID == DeviceId::ID::ARM_EMERGENCY_INTERVENTION) ? 0x000F : 0x00F0;
    return build_frame.buildAddress(
        static_cast<uint32_t>(deviceType::DeviceType::BAB),
        Manufacturer::TEAM_USE,
        severity::SEV_CNTRL,
        static_cast<uint32_t>(inst),
        static_cast<uint32_t>(selectRailID),
        payload_val);
}
