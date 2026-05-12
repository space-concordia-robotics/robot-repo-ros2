#include "wheels_interface/spark/spark_base.hpp"

#include <array>
#include <cstring>
#include <fcntl.h>
#include <map>
#include <stdexcept>


namespace wheels_interface {
    SparkBase::SparkBase(rclcpp::Logger& logger, can_util::CANController& can_controller, const uint8_t deviceId)
        : logger(logger.get_child("spark_max")), can_controller(can_controller), device_id(deviceId) {
        frame_callback = can_controller.registerFrameCallback([this](const uint32_t id, const std::vector<uint8_t>& data) {
            handleFrame(id, data);
        });

        // Ensure deviceId_ is within valid range
        if (deviceId > 62)
            throw std::out_of_range("Invalid CAN bus ID. Must be between 0 and 62.");
    }

    bool SparkBase::sendCanFrame(const APICommand cmd, const std::vector<uint8_t>& data) const {
        return sendCanFrame(createArbId(cmd), data);
    }

    bool SparkBase::sendCanFrame(const uint32_t arbId, const std::vector<uint8_t>& data) const {
        const auto status = can_controller.sendBlockingFrame(arbId, data);

        if (!status)
            logger.error("Could not send CAN frame");

        return status;
    }

    bool SparkBase::sendControlMessage(
        const APICommand cmd,
        const std::string& commandName,
        const float value,
        const std::optional<float> minValue,
        const std::optional<float> maxValue
    ) const {
        if (!std::isfinite(value))
            throw std::invalid_argument(commandName + " must be a finite number.");

        if (minValue && maxValue && (value < *minValue || value > *maxValue))
            throw std::out_of_range(commandName + " must be between " + std::to_string(*minValue) + " and " + std::to_string(*maxValue));

        std::vector<uint8_t> data(8, 0);
        std::memcpy(data.data(), &value, sizeof(value));
        return sendCanFrame(cmd, data);
    }

    uint32_t SparkBase::createArbId(const APICommand cmd) const {
        const uint8_t apiClass = getAPIClass(cmd);
        const uint8_t apiIndex = getAPIIndex(cmd);

        return static_cast<uint32_t>(DEVICE_TYPE) << 24
            | static_cast<uint32_t>(MANUFACTURER) << 16
            | static_cast<uint32_t>(apiClass) << 10
            | static_cast<uint32_t>(apiIndex) << 6
            | static_cast<uint32_t>(device_id);
    }

    uint32_t SparkBase::createParamArbId(Parameter paramId) const {
        return static_cast<uint32_t>(DEVICE_TYPE) << 24
            | static_cast<uint32_t>(MANUFACTURER) << 16
            | static_cast<uint32_t>(48) << 10
            | static_cast<uint32_t>(paramId) << 6
            | static_cast<uint32_t>(device_id);
    }

    uint8_t SparkBase::getAPIClass(APICommand cmd) {
        return static_cast<uint8_t>(static_cast<uint16_t>(cmd) >> 4);
    }

    uint8_t SparkBase::getAPIIndex(APICommand cmd) {
        return static_cast<uint8_t>(static_cast<uint16_t>(cmd) & 0x0F);
    }

    void SparkBase::handleFrame(const uint32_t id, const std::vector<uint8_t>& data) {
        const uint32_t receivedArbId = id;
        uint64_t rawValue = 0;
        for (auto i = 0u; i < data.size(); ++i) {
            rawValue |= static_cast<uint64_t>(data[i]) << (8 * i);
        }
        const auto now = std::chrono::steady_clock::now();

        std::lock_guard lock(mtx);

        if (receivedArbId == createArbId(APICommand::Period0)) {
            period0.dutyCycle = static_cast<float>(static_cast<int16_t>(rawValue & 0xFFFF)) / 32768.0f;
            period0.faults = rawValue >> 16 & 0xFFFF;
            period0.stickyFaults = rawValue >> 32 & 0xFFFF;
            period0.isInverted = rawValue >> 49 & 1;
            period0.idleMode = rawValue >> 57 & 1;
            period0.isFollower = rawValue >> 58 & 1;
            period0.timestamp = now;
        } else if (receivedArbId == createArbId(APICommand::Period1)) {
            const uint32_t velocity = rawValue & 0xFFFFFFFF;
            std::memcpy(&period1.velocity, &velocity, 4);
            period1.temperature = static_cast<float>(rawValue >> 32 & 0xFF);
            period1.voltage = static_cast<float>(rawValue >> 40 & 0xFFFF) / 128.0f;
            period1.current = static_cast<float>(rawValue >> 48 & 0xFFF) / 32.0f;
            period1.timestamp = now;
        } else if (receivedArbId == createArbId(APICommand::Period2)) {
            const uint32_t position = rawValue & 0xFFFFFFFF;
            std::memcpy(&period2.position, &position, 4);
            period2.iAccum = static_cast<float>(rawValue >> 32 & 0xFFFFFFFF) / 1000.0f;
            period2.timestamp = now;
        } else if (receivedArbId == createArbId(APICommand::Period3)) {
            const uint8_t* intVal = reinterpret_cast<uint8_t*>(&rawValue);
            const uint16_t voltage = intVal[0] | (intVal[1] & 3) << 8;
            period3.analogVoltage = static_cast<float>(voltage) / 256.0f;
            const uint32_t velocity = (intVal[1] >> 2 & 0x3F) | static_cast<uint32_t>(intVal[2]) << 6 | static_cast<uint32_t>(intVal[3]) << 14;
            period3.analogVelocity = static_cast<float>(velocity) / 32768.0f;
            const uint32_t position = rawValue >> 32 & 0xFFFFFFFF;
            std::memcpy(&period3.analogPosition, &position, 4);
            period3.timestamp = now;
        } else if (receivedArbId == createArbId(APICommand::Period4)) {
            const uint32_t velocity = rawValue & 0xFFFFFFFF;
            const uint32_t position = rawValue >> 32 & 0xFFFFFFFF;
            std::memcpy(&period4.altEncoderVelocity, &velocity, 4);
            std::memcpy(&period4.altEncoderPosition, &position, 4);
            period4.timestamp = now;
        }
    }

    void SparkBase::setParameter(
        const Parameter parameterId,
        const uint8_t parameterType,
        const std::string& parameterName,
        std::variant<float, uint32_t, uint16_t, uint8_t, bool> value,
        std::optional<float> minValue,
        std::optional<float> maxValue,
        const std::optional<std::string>& customErrorMessage
    ) {
        // Lambda to handle range validation errors
        auto throwRangeError = [&](auto min, auto max) {
            if (customErrorMessage) {
                throw std::out_of_range(*customErrorMessage);
            } else {
                throw std::out_of_range(fmt::format("Parameter '{}' must be between {} and {}", parameterName, min, max));
            }
        };

        // Create CAN data and arbitration ID
        std::vector<uint8_t> data(5, 0);
        const uint32_t arbId = createParamArbId(parameterId);

        // Process the value based on its type and fill CAN data
        std::visit(
            [&](auto&& v) {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, float>) {
                    if (!std::isfinite(v)) {
                        // Ensure float is valid
                        throw std::invalid_argument(fmt::format("Parameter '{}' must be a finite number, but was {}.", parameterName, v));
                    }
                    if (minValue && maxValue && (v < minValue.value() || v > maxValue.value())) {
                        throwRangeError(minValue.value(), maxValue.value());
                    }
                    std::memcpy(data.data(), &v, sizeof(v)); // Copy float to CAN data
                } else if constexpr (std::is_integral_v<T>) {
                    std::memcpy(data.data(), &v, sizeof(v)); // Copy integer to CAN data
                } else if constexpr (std::is_same_v<T, bool>) {
                    data[0] = v ? 1 : 0; // Handle boolean type
                } else {
                    throw std::invalid_argument("Unsupported value type.");
                }
            },
            value
        );

        data[4] = parameterType; // Add parameter type to CAN data
        sendCanFrame(arbId, data); // Send CAN frame with parameter data
    }

    std::optional<std::variant<float, uint32_t, bool>> SparkBase::readParameter(const Parameter parameterId) const {
        const uint32_t requestArbId = createParamArbId(parameterId);
        if (!can_controller.sendBlockingFrame(requestArbId, {})) {
            logger.error("Error sending CAN message");
            return std::nullopt;
        }

        // TODO 2026-02-25 (Will Free): handle case where the next frame read is not the frame with the response
        if (can_frame response = {}; can_controller.readFrameIfAvailable(response)) {
            if (const uint32_t receivedArbId = response.can_id & CAN_EFF_MASK; receivedArbId == requestArbId) {
                switch (const auto type = response.data[4]) {
                case 0x01: {
                    uint32_t val = 0;
                    for (int i = 0; i < 4; ++i) {
                        val |= static_cast<uint32_t>(response.data[i]) << (8 * i);
                    }
                    return val;
                }
                case 0x02: {
                    float val;
                    std::memcpy(&val, response.data, sizeof(float));
                    return val;
                }
                case 0x03: {
                    return response.data[0] != 0;
                }
                default: {
                    logger.error("received bad response type {} while attempting to get value for parameter.", type);
                    return std::nullopt;
                }
                }
            }
        }

        return std::nullopt;
    }

    template <typename T>
    T defaultValue() {
        static_assert(std::is_same_v<T, bool> || std::is_integral_v<T> || std::is_floating_point_v<T>, "Unsupported type");

        if constexpr (std::is_same_v<T, bool>) {
            return false;
        } else if constexpr (std::is_integral_v<T>) {
            return 0;
        } else if constexpr (std::is_floating_point_v<T>) {
            return 0.0;
        } else {
            static_assert(sizeof(T) == 0, "Unsupported type");
            return T(); // for unsupported types, return a default-initialized value
        }
    }

    template <typename T>
    T SparkBase::getParamAs(const Parameter param, const char* name) {
        static_assert(std::is_same_v<T, bool> || std::is_integral_v<T> || std::is_floating_point_v<T>, "Unsupported type");

        const auto result = readParameter(param);
        if (!result.has_value()) {
            logger.error("No response for parameter '{}', using default value.", name);
            if constexpr (std::is_same_v<T, float>) {
                return 0.0f;
            } else if constexpr (std::is_same_v<T, bool>) {
                return false;
            } else {
                return static_cast<T>(0);
            }
        }

        return std::visit([&](auto&& r) -> T {
            using R = std::decay_t<decltype(r)>;

            if constexpr (std::is_convertible_v<R, T>) {
                return static_cast<T>(std::forward<decltype(r)>(r));
            } else {
                logger.error("Wrong type for parameter '{}', expected {} got {}. Using default value.", name, typeid(T).name(), typeid(R).name());
                return defaultValue<T>();
            }
        }, *result);
    }

    template <typename T>
    T SparkBase::getPIDParam(Parameter baseParam, const uint8_t slot, const char* name) {
        if (slot >= 4) {
            throw std::out_of_range("Invalid slot number");
        }
        return getParamAs<T>(static_cast<Parameter>(static_cast<int>(baseParam) + slot), name);
    }

    std::optional<std::tuple<uint8_t, uint8_t, uint8_t, uint8_t, bool>> SparkBase::readFirmwareVersion() const {
        const auto requestArbId = createArbId(APICommand::FirmwareVersion);

        if (!can_controller.sendBlockingFrame(requestArbId, {})) {
            logger.error("Error sending firmware version request");
            return std::nullopt;
        }

        // TODO 2026-02-25 (Will Free): handle case where the next frame read is not the frame with the response
        if (can_frame response = {}; can_controller.readFrameIfAvailable(response)) {
            if (const uint32_t receivedArbId = response.can_id & CAN_EFF_MASK; receivedArbId == requestArbId) {
                uint8_t major = response.data[0];
                uint8_t minor = response.data[1];
                uint8_t patch = response.data[2];
                uint8_t build = response.data[3];
                bool isDebug = response.data[4] != 0;
                return std::make_tuple(major, minor, patch, build, isDebug);
            }
        }

        return std::nullopt;
    }

    bool SparkBase::heartbeat() const {
        const std::vector<uint8_t> data(8, 0xFF);
        return sendCanFrame(APICommand::Heartbeat, data);
    }

    bool SparkBase::resetFaults() const {
        const std::vector<uint8_t> data(8, 0x00);
        return sendCanFrame(APICommand::ClearFaults, data);
    }

    bool SparkBase::clearStickyFaults() const {
        const std::vector<uint8_t> data(8, 0x00);
        return sendCanFrame(APICommand::ClearFaults, data);
    }

    bool SparkBase::burnFlash() const {
        const std::vector<uint8_t> data = {0xA3, 0x3A};
        return sendCanFrame(APICommand::BurnFlash, data);
    }

    bool SparkBase::factoryDefaults() const {
        const std::vector<uint8_t> data = {0x01};
        return sendCanFrame(APICommand::FactoryDefaults, data);
    }

    bool SparkBase::factoryReset() const {
        const std::vector<uint8_t> data = {0x01};
        return sendCanFrame(APICommand::FactoryReset, data);
    }

    bool SparkBase::identify() const {
        const std::vector<uint8_t> data(8, 0x00);
        return sendCanFrame(APICommand::Identify, data);
    }

    // Motor Control //
    bool SparkBase::setSetpoint(const float setpoint) const {
        return sendControlMessage(APICommand::Setpoint, "Setpoint", setpoint);
    }

    bool SparkBase::setDutyCycle(const float dutyCycle) const {
        return sendControlMessage(APICommand::DutyCycle, "Duty Cycle", dutyCycle, -1.0f, 1.0f);
    }

    bool SparkBase::setVelocity(const float velocity) const {
        return sendControlMessage(APICommand::Velocity, "Velocity", velocity);
    }

    bool SparkBase::setSmartVelocity(const float smartVelocity) const {
        return sendControlMessage(APICommand::SmartVelocity, "Smart Velocity", smartVelocity);
    }

    bool SparkBase::setPosition(const float position) const {
        return sendControlMessage(APICommand::Position, "Position", position);
    }

    bool SparkBase::setVoltage(const float voltage) const {
        return sendControlMessage(APICommand::Voltage, "Voltage", voltage);
    }

    bool SparkBase::setCurrent(const float current) const {
        return sendControlMessage(APICommand::Current, "Current", current);
    }

    bool SparkBase::setSmartMotion(const float smartMotion) const {
        return sendControlMessage(APICommand::SmartMotion, "Smart Motion", smartMotion);
    }

    // Status //

    bool SparkBase::setPeriodicStatus0Period(const uint16_t period) const {
        std::vector<uint8_t> data(2, 0x00);
        data[0] = static_cast<uint8_t>(period & 0xFF);
        data[1] = static_cast<uint8_t>(period >> 8 & 0xFF);
        return sendCanFrame(APICommand::Period0, data);
    }

    bool SparkBase::setPeriodicStatus1Period(const uint16_t period) const {
        std::vector<uint8_t> data(2, 0x00);
        data[0] = static_cast<uint8_t>(period & 0xFF);
        data[1] = static_cast<uint8_t>(period >> 8 & 0xFF);
        return sendCanFrame(APICommand::Period1, data);
    }

    bool SparkBase::setPeriodicStatus2Period(const uint16_t period) const {
        std::vector<uint8_t> data(2, 0x00);
        data[0] = static_cast<uint8_t>(period & 0xFF);
        data[1] = static_cast<uint8_t>(period >> 8 & 0xFF);
        return sendCanFrame(APICommand::Period2, data);
    }

    bool SparkBase::setPeriodicStatus3Period(const uint16_t period) const {
        std::vector<uint8_t> data(2, 0x00);
        data[0] = static_cast<uint8_t>(period & 0xFF);
        data[1] = static_cast<uint8_t>(period >> 8 & 0xFF);
        return sendCanFrame(APICommand::Period3, data);
    }

    bool SparkBase::setPeriodicStatus4Period(const uint16_t period) const {
        std::vector<uint8_t> data(2, 0x00);
        data[0] = static_cast<uint8_t>(period & 0xFF);
        data[1] = static_cast<uint8_t>(period >> 8 & 0xFF);
        return sendCanFrame(APICommand::Period4, data);
    }

    // Period 0
    float SparkBase::getDutyCycle() const {
        std::lock_guard lock(mtx);
        return period0.dutyCycle;
    }

    uint16_t SparkBase::getFaults() const {
        std::lock_guard lock(mtx);
        return period0.faults;
    }

    uint16_t SparkBase::getStickyFaults() const {
        std::lock_guard lock(mtx);
        return period0.stickyFaults;
    }

    bool SparkBase::isInverted() const {
        std::lock_guard lock(mtx);
        return period0.isInverted;
    }

    bool SparkBase::getIdleMode() const {
        std::lock_guard lock(mtx);
        return period0.idleMode;
    }

    bool SparkBase::isFollower() const {
        std::lock_guard lock(mtx);
        return period0.isFollower;
    }

    // Period 1
    float SparkBase::getVelocity() const {
        std::lock_guard lock(mtx);
        return period1.velocity;
    }

    float SparkBase::getTemperature() const {
        std::lock_guard lock(mtx);
        return period1.temperature;
    }

    float SparkBase::getVoltage() const {
        std::lock_guard lock(mtx);
        return period1.voltage;
    }

    float SparkBase::getCurrent() const {
        std::lock_guard lock(mtx);
        return period1.current;
    }

    // Period 2
    float SparkBase::getPosition() const {
        std::lock_guard lock(mtx);
        return period2.position;
    }

    float SparkBase::getIAccum() const {
        std::lock_guard lock(mtx);
        return period2.iAccum;
    }

    // Period 3
    float SparkBase::getAnalogVoltage() const {
        std::lock_guard lock(mtx);
        return period3.analogVoltage;
    }

    float SparkBase::getAnalogVelocity() const {
        std::lock_guard lock(mtx);
        return period3.analogVelocity;
    }

    float SparkBase::getAnalogPosition() const {
        std::lock_guard lock(mtx);
        return period3.analogPosition;
    }

    // Period 4
    float SparkBase::getAltEncoderVelocity() const {
        std::lock_guard lock(mtx);
        return period4.altEncoderVelocity;
    }

    float SparkBase::getAltEncoderPosition() const {
        std::lock_guard lock(mtx);
        return period4.altEncoderPosition;
    }

    // Parameter Setters //

    // Basic //

    void SparkBase::setMotorType(MotorType type) {
        setParameter(
            Parameter::kMotorType, PARAM_TYPE_UINT, "Motor Type", static_cast<uint8_t>(type), 0, 1,
            "Invalid motor type. Must be 0 (Brushed) or 1 (Brushless)."
        );
    }

    void SparkBase::setSensorType(SensorType sensor) {
        setParameter(
            Parameter::kSensorType, PARAM_TYPE_UINT, "Sensor Type", static_cast<uint8_t>(sensor), 0, 2,
            "Invalid sensor type. Must be 0 (No Sensor), 1 (Hall Sensor), or 2 (Encoder)."
        );
    }

    void SparkBase::setIdleMode(IdleMode mode) {
        setParameter(
            Parameter::kIdleMode, PARAM_TYPE_UINT, "Idle Mode", static_cast<uint8_t>(mode), 0, 1,
            "Invalid idle mode. Must be 0 (Coast) or 1 (Brake)."
        );
    }

    void SparkBase::setInputDeadband(float deadband) {
        setParameter(Parameter::kInputDeadband, PARAM_TYPE_FLOAT, "Input Deadband", deadband);
    }

    void SparkBase::setInverted(bool inverted) {
        setParameter(Parameter::kInverted, PARAM_TYPE_BOOL, "Inverted", inverted);
    }

    void SparkBase::setRampRate(float rate) {
        setParameter(Parameter::kRampRate, PARAM_TYPE_FLOAT, "Ramp Rate", rate);
    }

    // Advanced //

    void SparkBase::setMotorKv(uint16_t kv) {
        setParameter(Parameter::kMotorKv, PARAM_TYPE_UINT, "Motor Kv", kv);
    }

    void SparkBase::setMotorR(uint16_t r) {
        setParameter(Parameter::kMotorR, PARAM_TYPE_UINT, "Motor Resistance", r);
    }

    void SparkBase::setMotorL(uint16_t l) {
        setParameter(Parameter::kMotorL, PARAM_TYPE_UINT, "Motor Inductance", l);
    }

    // Closed Loop //

    void SparkBase::setCtrlType(CtrlType type) {
        setParameter(
            Parameter::kCtrlType, PARAM_TYPE_UINT, "Control Type", static_cast<uint8_t>(type), 0, 3,
            "Invalid control type. Must be 0 (Duty Cycle), 1 (Velocity), 2 (Voltage), or 3 (Position)."
        );
    }

    void SparkBase::setFeedbackSensorPID0(uint16_t sensor) {
        setParameter(Parameter::kFeedbackSensorPID0, PARAM_TYPE_UINT, "Feedback Sensor PID0", sensor);
    }

    void SparkBase::setClosedLoopVoltageMode(uint8_t mode) {
        setParameter(
            Parameter::kClosedLoopVoltageMode, PARAM_TYPE_UINT, "Closed Loop Voltage Mode", mode, 0, 2,
            "Invalid closed loop voltage mode. Must be 0 (Disabled), 1 (Control Loop Voltage Output Mode) or 2 (Voltage Compensation Mode)."
        );
    }

    void SparkBase::setCompensatedNominalVoltage(float voltage) {
        setParameter(Parameter::kCompensatedNominalVoltage, PARAM_TYPE_FLOAT, "Compensated Nominal Voltage", voltage);
    }

    void SparkBase::setPositionPIDWrapEnable(bool enable) {
        setParameter(Parameter::kPositionPIDWrapEnable, PARAM_TYPE_BOOL, "Position PID Wrap Enable", enable);
    }

    void SparkBase::setPositionPIDMinInput(float minInput) {
        setParameter(Parameter::kPositionPIDMinInput, PARAM_TYPE_FLOAT, "Position PID Min Input", minInput);
    }

    void SparkBase::setPositionPIDMaxInput(float maxInput) {
        setParameter(Parameter::kPositionPIDMaxInput, PARAM_TYPE_FLOAT, "Position PID Max Input", maxInput);
    }

    // Brushless //

    void SparkBase::setPolePairs(uint16_t pairs) {
        setParameter(Parameter::kPolePairs, PARAM_TYPE_UINT, "Pole Pairs", pairs);
    }

    // Current Limit //

    void SparkBase::setCurrentChop(float chop) {
        setParameter(Parameter::kCurrentChop, PARAM_TYPE_FLOAT, "Current Chop", chop, 0, 125);
    }

    void SparkBase::setCurrentChopCycles(uint16_t cycles) {
        setParameter(Parameter::kCurrentChopCycles, PARAM_TYPE_UINT, "Current Chop Cycles", cycles);
    }

    void SparkBase::setSmartCurrentStallLimit(uint16_t limit) {
        setParameter(Parameter::kSmartCurrentStallLimit, PARAM_TYPE_UINT, "Smart Current Stall Limit", limit);
    }

    void SparkBase::setSmartCurrentFreeLimit(uint16_t limit) {
        setParameter(Parameter::kSmartCurrentFreeLimit, PARAM_TYPE_UINT, "Smart Current Free Limit", limit);
    }

    void SparkBase::setSmartCurrentConfig(uint16_t config) {
        setParameter(Parameter::kSmartCurrentConfig, PARAM_TYPE_UINT, "Smart Current Config", config);
    }

    // PIDF //

    void SparkBase::setP(const uint8_t slot, float p) {
        static constexpr std::array params = {
            Parameter::kP_0, Parameter::kP_1,
            Parameter::kP_2, Parameter::kP_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "P", p);
    }

    void SparkBase::setI(const uint8_t slot, float i) {
        static constexpr std::array params = {
            Parameter::kI_0, Parameter::kI_1,
            Parameter::kI_2, Parameter::kI_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "I", i);
    }

    void SparkBase::setD(const uint8_t slot, float d) {
        static constexpr std::array params = {
            Parameter::kD_0, Parameter::kD_1,
            Parameter::kD_2, Parameter::kD_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "D", d);
    }

    void SparkBase::setF(const uint8_t slot, float f) {
        static constexpr std::array params = {
            Parameter::kF_0, Parameter::kF_1,
            Parameter::kF_2, Parameter::kF_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "F", f);
    }

    void SparkBase::setIZone(const uint8_t slot, float iZone) {
        static constexpr std::array params = {
            Parameter::kIZone_0, Parameter::kIZone_1,
            Parameter::kIZone_2, Parameter::kIZone_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "IZone", iZone);
    }

    void SparkBase::setDFilter(const uint8_t slot, float dFilter) {
        static constexpr std::array params = {
            Parameter::kDFilter_0, Parameter::kDFilter_1,
            Parameter::kDFilter_2, Parameter::kDFilter_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "DFilter", dFilter);
    }

    void SparkBase::setOutputMin(const uint8_t slot, float min) {
        static constexpr std::array params = {
            Parameter::kOutputMin_0, Parameter::kOutputMin_1,
            Parameter::kOutputMin_2, Parameter::kOutputMin_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "Output Min", min);
    }

    void SparkBase::setOutputMax(const uint8_t slot, float max) {
        static constexpr std::array params = {
            Parameter::kOutputMax_0, Parameter::kOutputMax_1,
            Parameter::kOutputMax_2, Parameter::kOutputMax_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "Output Max", max);
    }

    // Limits //

    void SparkBase::setHardLimitFwdEn(bool enable) {
        setParameter(Parameter::kHardLimitFwdEn, PARAM_TYPE_BOOL, "Hard Limit Forward Enable", enable);
    }

    void SparkBase::setHardLimitRevEn(bool enable) {
        setParameter(Parameter::kHardLimitRevEn, PARAM_TYPE_BOOL, "Hard Limit Reverse Enable", enable);
    }

    void SparkBase::setLimitSwitchFwdPolarity(bool polarity) {
        setParameter(Parameter::kLimitSwitchFwdPolarity, PARAM_TYPE_BOOL, "Limit Switch Forward Polarity", polarity);
    }

    void SparkBase::setLimitSwitchRevPolarity(bool polarity) {
        setParameter(Parameter::kLimitSwitchRevPolarity, PARAM_TYPE_BOOL, "Limit Switch Reverse Polarity", polarity);
    }

    void SparkBase::setSoftLimitFwdEn(bool enable) {
        setParameter(Parameter::kSoftLimitFwdEn, PARAM_TYPE_BOOL, "Soft Limit Forward Enable", enable);
    }

    void SparkBase::setSoftLimitRevEn(bool enable) {
        setParameter(Parameter::kSoftLimitRevEn, PARAM_TYPE_BOOL, "Soft Limit Reverse Enable", enable);
    }

    void SparkBase::setSoftLimitFwd(float limit) {
        setParameter(Parameter::kSoftLimitFwd, PARAM_TYPE_FLOAT, "Soft Limit Forward", limit);
    }

    void SparkBase::setSoftLimitRev(float limit) {
        setParameter(Parameter::kSoftLimitRev, PARAM_TYPE_FLOAT, "Soft Limit Reverse", limit);
    }

    // Follower //

    void SparkBase::setFollowerID(uint32_t id) {
        setParameter(Parameter::kFollowerID, PARAM_TYPE_UINT, "Follower ID", id);
    }

    void SparkBase::setFollowerConfig(uint32_t config) {
        setParameter(Parameter::kFollowerConfig, PARAM_TYPE_UINT, "Follower Config", config);
    }

    // Encoder Port //

    void SparkBase::setEncoderCountsPerRev(uint16_t counts) {
        setParameter(Parameter::kEncoderCountsPerRev, PARAM_TYPE_UINT, "Encoder Counts Per Revolution", counts);
    }

    void SparkBase::setEncoderAverageDepth(uint8_t depth) {
        setParameter(Parameter::kEncoderAverageDepth, PARAM_TYPE_UINT, "Encoder Average Depth", depth, 1, 64);
    }

    void SparkBase::setEncoderSampleDelta(uint8_t delta) {
        setParameter(Parameter::kEncoderSampleDelta, PARAM_TYPE_UINT, "Encoder Sample Delta", delta, 1, 255);
    }

    void SparkBase::setEncoderInverted(bool inverted) {
        setParameter(Parameter::kEncoderInverted, PARAM_TYPE_BOOL, "Encoder Inverted", inverted);
    }

    void SparkBase::setPositionConversionFactor(float factor) {
        setParameter(Parameter::kPositionConversionFactor, PARAM_TYPE_FLOAT, "Position Conversion Factor", factor);
    }

    void SparkBase::setVelocityConversionFactor(float factor) {
        setParameter(Parameter::kVelocityConversionFactor, PARAM_TYPE_FLOAT, "Velocity Conversion Factor", factor);
    }

    void SparkBase::setClosedLoopRampRate(float rampRate) {
        setParameter(Parameter::kClosedLoopRampRate, PARAM_TYPE_FLOAT, "Closed Loop Ramp Rate", rampRate);
    }

    void SparkBase::setHallSensorSampleRate(float rate) {
        setParameter(Parameter::kHallSensorSampleRate, PARAM_TYPE_FLOAT, "Hall Sensor Sample Rate", rate);
    }

    void SparkBase::setHallSensorAverageDepth(uint16_t depth) {
        setParameter(Parameter::kHallSensorAverageDepth, PARAM_TYPE_UINT, "Hall Sensor Average Depth", depth);
    }

    // Smart Motion //

    void SparkBase::setSmartMotionMaxVelocity(const uint8_t slot, float maxVel) {
        static constexpr std::array params = {
            Parameter::kSmartMotionMaxVelocity_0, Parameter::kSmartMotionMaxVelocity_1,
            Parameter::kSmartMotionMaxVelocity_2, Parameter::kSmartMotionMaxVelocity_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Max Velocity", maxVel);
    }

    void SparkBase::setSmartMotionMaxAccel(const uint8_t slot, float maxAccel) {
        static constexpr std::array params = {
            Parameter::kSmartMotionMaxAccel_0, Parameter::kSmartMotionMaxAccel_1,
            Parameter::kSmartMotionMaxAccel_2, Parameter::kSmartMotionMaxAccel_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Max Accel", maxAccel);
    }

    void SparkBase::setSmartMotionMinVelOutput(const uint8_t slot, float minVel) {
        static constexpr std::array params = {
            Parameter::kSmartMotionMinVelOutput_0, Parameter::kSmartMotionMinVelOutput_1,
            Parameter::kSmartMotionMinVelOutput_2, Parameter::kSmartMotionMinVelOutput_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Min Vel Output", minVel);
    }

    void SparkBase::setSmartMotionAllowedClosedLoopError(const uint8_t slot, float error) {
        static constexpr std::array params = {
            Parameter::kSmartMotionAllowedClosedLoopError_0, Parameter::kSmartMotionAllowedClosedLoopError_1,
            Parameter::kSmartMotionAllowedClosedLoopError_2, Parameter::kSmartMotionAllowedClosedLoopError_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Allowed Close Loop Error", error);
    }

    void SparkBase::setSmartMotionAccelStrategy(const uint8_t slot, float strategy) {
        static constexpr std::array params = {
            Parameter::kSmartMotionAccelStrategy_0, Parameter::kSmartMotionAccelStrategy_1,
            Parameter::kSmartMotionAccelStrategy_2, Parameter::kSmartMotionAccelStrategy_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Accel Strategy", strategy);
    }

    void SparkBase::setIMaxAccum(const uint8_t slot, float maxAccum) {
        static constexpr std::array params = {
            Parameter::kIMaxAccum_0, Parameter::kIMaxAccum_1,
            Parameter::kIMaxAccum_2, Parameter::kIMaxAccum_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "IMaxAccum", maxAccum);
    }

    void SparkBase::setSlot3Placeholder1(const uint8_t slot, float value) {
        static constexpr std::array params = {
            Parameter::kSlot3Placeholder1_0, Parameter::kSlot3Placeholder1_1,
            Parameter::kSlot3Placeholder1_2, Parameter::kSlot3Placeholder1_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 1", value);
    }

    void SparkBase::setSlot3Placeholder2(const uint8_t slot, float value) {
        static constexpr std::array params = {
            Parameter::kSlot3Placeholder2_0, Parameter::kSlot3Placeholder2_1,
            Parameter::kSlot3Placeholder2_2, Parameter::kSlot3Placeholder2_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 2", value);
    }

    void SparkBase::setSlot3Placeholder3(const uint8_t slot, float value) {
        static constexpr std::array params = {
            Parameter::kSlot3Placeholder3_0, Parameter::kSlot3Placeholder3_1,
            Parameter::kSlot3Placeholder3_2, Parameter::kSlot3Placeholder3_3
        };

        if (slot >= params.size())
            throw std::out_of_range(fmt::format("Invalid slot number. Max value is 3, got {}.", slot));

        setParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 3", value);
    }

    // Analog Sensor //

    void SparkBase::setAnalogPositionConversion(float factor) {
        setParameter(Parameter::kAnalogPositionConversion, PARAM_TYPE_FLOAT, "Analog Position Conversion", factor);
    }

    void SparkBase::setAnalogVelocityConversion(float factor) {
        setParameter(Parameter::kAnalogVelocityConversion, PARAM_TYPE_FLOAT, "Analog Velocity Conversion", factor);
    }

    void SparkBase::setAnalogAverageDepth(uint16_t depth) {
        setParameter(Parameter::kAnalogAverageDepth, PARAM_TYPE_UINT, "Analog Average Depth", depth);
    }

    void SparkBase::setAnalogSensorMode(uint8_t mode) {
        setParameter(
            Parameter::kAnalogSensorMode, PARAM_TYPE_UINT, "Analog Sensor Mode", mode, 0, 1,
            "Invalid analog sensor mode. Must be 0 (Absolute) or 1 (Relative)."
        );
    }

    void SparkBase::setAnalogInverted(bool inverted) {
        setParameter(Parameter::kAnalogInverted, PARAM_TYPE_BOOL, "Analog Inverted", inverted);
    }

    void SparkBase::setAnalogSampleDelta(uint16_t delta) {
        setParameter(Parameter::kAnalogSampleDelta, PARAM_TYPE_UINT, "Analog Sample Delta", delta);
    }

    // Alternate Encoder //

    void SparkBase::setDataPortConfig(uint8_t config) {
        setParameter(
            Parameter::kDataPortConfig, PARAM_TYPE_UINT, "Data Port Config", config, 0, 1,
            "Invalid data port config. Must be 0 (Default) or 1 (Alternate Encoder Mode)."
        );
    }

    void SparkBase::setAltEncoderCountsPerRev(uint16_t counts) {
        setParameter(Parameter::kAltEncoderCountsPerRev, PARAM_TYPE_UINT, "Alternate Encoder Counts Per Revolution", counts);
    }

    void SparkBase::setAltEncoderAverageDepth(uint8_t depth) {
        setParameter(Parameter::kAltEncoderAverageDepth, PARAM_TYPE_UINT, "Alternate Encoder Average Depth", depth, 1, 64);
    }

    void SparkBase::setAltEncoderSampleDelta(uint8_t delta) {
        setParameter(Parameter::kAltEncoderSampleDelta, PARAM_TYPE_UINT, "Alternate Encoder Sample Delta", delta, 1, 255);
    }

    void SparkBase::setAltEncoderInverted(bool inverted) {
        setParameter(Parameter::kAltEncoderInverted, PARAM_TYPE_BOOL, "Alternate Encoder Inverted", inverted);
    }

    void SparkBase::setAltEncoderPositionFactor(float factor) {
        setParameter(Parameter::kAltEncoderPositionFactor, PARAM_TYPE_FLOAT, "Alternate Encoder Position Factor", factor);
    }

    void SparkBase::setAltEncoderVelocityFactor(float factor) {
        setParameter(Parameter::kAltEncoderVelocityFactor, PARAM_TYPE_FLOAT, "Alternate Encoder Velocity Factor", factor);
    }

    // Duty Cycle Absolute Encoder //

    void SparkBase::setDutyCyclePositionFactor(float factor) {
        setParameter(Parameter::kDutyCyclePositionFactor, PARAM_TYPE_FLOAT, "Duty Cycle Position Factor", factor);
    }

    void SparkBase::setDutyCycleVelocityFactor(float factor) {
        setParameter(Parameter::kDutyCycleVelocityFactor, PARAM_TYPE_FLOAT, "Duty Cycle Velocity Factor", factor);
    }

    void SparkBase::setDutyCycleInverted(bool inverted) {
        setParameter(Parameter::kDutyCycleInverted, PARAM_TYPE_BOOL, "Duty Cycle Inverted", inverted);
    }

    void SparkBase::setDutyCycleAverageDepth(uint8_t depth) {
        setParameter(
            Parameter::kDutyCycleAverageDepth, PARAM_TYPE_UINT, "Duty Cycle Average Depth", depth, 0, 7,
            "Invalid average depth. Must be 0 (1 bit), 2 (2 bits), 3 (4 bits), 4 (8 bits), 5 (16 bits), 6 (32 bits), or 7 (64 bits)."
        );
    }

    void SparkBase::setDutyCyclePrescalar(uint8_t prescalar) {
        setParameter(Parameter::kDutyCyclePrescalar, PARAM_TYPE_UINT, "Duty Cycle Prescalar", prescalar, 0, 71);
    }

    void SparkBase::setDutyCycleZeroOffset(float offset) {
        setParameter(Parameter::kDutyCycleZeroOffset, PARAM_TYPE_FLOAT, "Duty Cycle Zero Offset", offset, 0.0f, 1.0f);
    }

    // Parameter Getters //

    // Basic //

    uint8_t SparkBase::getMotorType() {
        return getParamAs<uint8_t>(Parameter::kMotorType, "MotorType");
    }

    uint8_t SparkBase::getSensorType() {
        return getParamAs<uint8_t>(Parameter::kSensorType, "SensorType");
    }

    uint8_t SparkBase::getIdleMode() {
        return getParamAs<uint8_t>(Parameter::kIdleMode, "IdleMode");
    }

    float SparkBase::getInputDeadband() {
        return getParamAs<float>(Parameter::kInputDeadband, "InputDeadband");
    }

    bool SparkBase::getInverted() {
        return getParamAs<bool>(Parameter::kInverted, "Inverted");
    }

    float SparkBase::getRampRate() {
        return getParamAs<float>(Parameter::kRampRate, "RampRate");
    }

    // Advanced //

    uint16_t SparkBase::getMotorKv() {
        return getParamAs<uint16_t>(Parameter::kMotorKv, "MotorKv");
    }

    uint16_t SparkBase::getMotorR() {
        return getParamAs<uint16_t>(Parameter::kMotorR, "MotorR");
    }

    uint16_t SparkBase::getMotorL() {
        return getParamAs<uint16_t>(Parameter::kMotorL, "MotorL");
    }

    // Closed Loop //

    uint8_t SparkBase::getCtrlType() {
        return getParamAs<uint8_t>(Parameter::kCtrlType, "CtrlType");
    }

    uint16_t SparkBase::getFeedbackSensorPID0() {
        return getParamAs<uint16_t>(Parameter::kFeedbackSensorPID0, "FeedbackSensorPID0");
    }

    uint8_t SparkBase::getClosedLoopVoltageMode() {
        return getParamAs<uint8_t>(Parameter::kClosedLoopVoltageMode, "ClosedLoopVoltageMode");
    }

    float SparkBase::getCompensatedNominalVoltage() {
        return getParamAs<float>(Parameter::kCompensatedNominalVoltage, "CompensatedNominalVoltage");
    }

    bool SparkBase::getPositionPIDWrapEnable() {
        return getParamAs<bool>(Parameter::kPositionPIDWrapEnable, "PositionPIDWrapEnable");
    }

    float SparkBase::getPositionPIDMinInput() {
        return getParamAs<float>(Parameter::kPositionPIDMinInput, "PositionPIDMinInput");
    }

    float SparkBase::getPositionPIDMaxInput() {
        return getParamAs<float>(Parameter::kPositionPIDMaxInput, "PositionPIDMaxInput");
    }

    // Brushless //

    uint16_t SparkBase::getPolePairs() {
        return getParamAs<uint16_t>(Parameter::kPolePairs, "PolePairs");
    }

    // Current Limit //

    float SparkBase::getCurrentChop() {
        return getParamAs<float>(Parameter::kCurrentChop, "CurrentChop");
    }

    uint16_t SparkBase::getCurrentChopCycles() {
        return getParamAs<uint16_t>(Parameter::kCurrentChopCycles, "CurrentChopCycles");
    }

    uint16_t SparkBase::getSmartCurrentStallLimit() {
        return getParamAs<uint16_t>(Parameter::kSmartCurrentStallLimit, "SmartCurrentStallLimit");
    }

    uint16_t SparkBase::getSmartCurrentFreeLimit() {
        return getParamAs<uint16_t>(Parameter::kSmartCurrentFreeLimit, "SmartCurrentFreeLimit");
    }

    uint16_t SparkBase::getSmartCurrentConfig() {
        return getParamAs<uint16_t>(Parameter::kSmartCurrentConfig, "SmartCurrentConfig");
    }

    // PIDF //

    float SparkBase::getP(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kP_0, slot, "P");
    }

    float SparkBase::getI(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kI_0, slot, "I");
    }

    float SparkBase::getD(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kD_0, slot, "D");
    }

    float SparkBase::getF(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kF_0, slot, "F");
    }

    float SparkBase::getIZone(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kIZone_0, slot, "IZone");
    }

    float SparkBase::getDFilter(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kDFilter_0, slot, "DFilter");
    }

    float SparkBase::getOutputMin(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kOutputMin_0, slot, "OutputMin");
    }

    float SparkBase::getOutputMax(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kOutputMax_0, slot, "OutputMax");
    }

    // Limits //

    bool SparkBase::getHardLimitFwdEn() {
        return getParamAs<bool>(Parameter::kHardLimitFwdEn, "HardLimitFwdEn");
    }

    bool SparkBase::getHardLimitRevEn() {
        return getParamAs<bool>(Parameter::kHardLimitRevEn, "HardLimitRevEn");
    }

    bool SparkBase::getLimitSwitchFwdPolarity() {
        return getParamAs<bool>(Parameter::kLimitSwitchFwdPolarity, "LimitSwitchFwdPolarity");
    }

    bool SparkBase::getLimitSwitchRevPolarity() {
        return getParamAs<bool>(Parameter::kLimitSwitchRevPolarity, "LimitSwitchRevPolarity");
    }

    bool SparkBase::getSoftLimitFwdEn() {
        return getParamAs<bool>(Parameter::kSoftLimitFwdEn, "SoftLimitFwdEn");
    }

    bool SparkBase::getSoftLimitRevEn() {
        return getParamAs<bool>(Parameter::kSoftLimitRevEn, "SoftLimitRevEn");
    }

    float SparkBase::getSoftLimitFwd() {
        return getParamAs<float>(Parameter::kSoftLimitFwd, "SoftLimitFwd");
    }

    float SparkBase::getSoftLimitRev() {
        return getParamAs<float>(Parameter::kSoftLimitRev, "SoftLimitRev");
    }

    // Follower //

    uint32_t SparkBase::getFollowerID() {
        return getParamAs<uint32_t>(Parameter::kFollowerID, "FollowerID");
    }

    uint32_t SparkBase::getFollowerConfig() {
        return getParamAs<uint32_t>(Parameter::kFollowerConfig, "FollowerConfig");
    }

    // Encoder Port //

    uint16_t SparkBase::getEncoderCountsPerRev() {
        return getParamAs<uint16_t>(Parameter::kEncoderCountsPerRev, "EncoderCountsPerRev");
    }

    uint8_t SparkBase::getEncoderAverageDepth() {
        return getParamAs<uint8_t>(Parameter::kEncoderAverageDepth, "EncoderAverageDepth");
    }

    uint8_t SparkBase::getEncoderSampleDelta() {
        return getParamAs<uint8_t>(Parameter::kEncoderSampleDelta, "EncoderSampleDelta");
    }

    bool SparkBase::getEncoderInverted() {
        return getParamAs<bool>(Parameter::kEncoderInverted, "EncoderInverted");
    }

    float SparkBase::getPositionConversionFactor() {
        return getParamAs<float>(Parameter::kPositionConversionFactor, "PositionConversionFactor");
    }

    float SparkBase::getVelocityConversionFactor() {
        return getParamAs<float>(Parameter::kVelocityConversionFactor, "VelocityConversionFactor");
    }

    float SparkBase::getClosedLoopRampRate() {
        return getParamAs<float>(Parameter::kClosedLoopRampRate, "ClosedLoopRampRate");
    }

    float SparkBase::getHallSensorSampleRate() {
        return getParamAs<float>(Parameter::kHallSensorSampleRate, "HallSensorSampleRate");
    }

    uint16_t SparkBase::getHallSensorAverageDepth() {
        return getParamAs<uint16_t>(Parameter::kHallSensorAverageDepth, "HallSensorAverageDepth");
    }

    // Smart Motion //

    float SparkBase::getSmartMotionMaxVelocity(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kSmartMotionMaxVelocity_0, slot, "SmartMotionMaxVelocity");
    }

    float SparkBase::getSmartMotionMaxAccel(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kSmartMotionMaxAccel_0, slot, "SmartMotionMaxAccel");
    }

    float SparkBase::getSmartMotionMinVelOutput(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kSmartMotionMinVelOutput_0, slot, "SmartMotionMinVelOutput");
    }

    float SparkBase::getSmartMotionAllowedClosedLoopError(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kSmartMotionAllowedClosedLoopError_0, slot, "SmartMotionAllowedClosedLoopError");
    }

    float SparkBase::getSmartMotionAccelStrategy(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kSmartMotionAccelStrategy_0, slot, "SmartMotionAccelStrategy");
    }

    float SparkBase::getIMaxAccum(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kIMaxAccum_0, slot, "IMaxAccum");
    }

    float SparkBase::getSlot3Placeholder1(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kSlot3Placeholder1_0, slot, "Slot3Placeholder1");
    }

    float SparkBase::getSlot3Placeholder2(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kSlot3Placeholder2_0, slot, "Slot3Placeholder2");
    }

    float SparkBase::getSlot3Placeholder3(const uint8_t slot) {
        return getPIDParam<float>(Parameter::kSlot3Placeholder3_0, slot, "Slot3Placeholder3");
    }

    // Analog Sensor //

    float SparkBase::getAnalogPositionConversion() {
        return getParamAs<float>(Parameter::kAnalogPositionConversion, "AnalogPositionConversion");
    }

    float SparkBase::getAnalogVelocityConversion() {
        return getParamAs<float>(Parameter::kAnalogVelocityConversion, "AnalogVelocityConversion");
    }

    uint16_t SparkBase::getAnalogAverageDepth() {
        return getParamAs<uint16_t>(Parameter::kAnalogAverageDepth, "AnalogAverageDepth");
    }

    uint8_t SparkBase::getAnalogSensorMode() {
        return getParamAs<uint8_t>(Parameter::kAnalogSensorMode, "AnalogSensorMode");
    }

    bool SparkBase::getAnalogInverted() {
        return getParamAs<bool>(Parameter::kAnalogInverted, "AnalogInverted");
    }

    uint16_t SparkBase::getAnalogSampleDelta() {
        return getParamAs<uint16_t>(Parameter::kAnalogSampleDelta, "AnalogSampleDelta");
    }

    // Alternate Encoder //

    uint8_t SparkBase::getDataPortConfig() {
        return getParamAs<uint8_t>(Parameter::kDataPortConfig, "DataPortConfig");
    }

    uint16_t SparkBase::getAltEncoderCountsPerRev() {
        return getParamAs<uint16_t>(Parameter::kAltEncoderCountsPerRev, "AltEncoderCountsPerRev");
    }

    uint8_t SparkBase::getAltEncoderAverageDepth() {
        return getParamAs<uint8_t>(Parameter::kAltEncoderAverageDepth, "AltEncoderAverageDepth");
    }

    uint8_t SparkBase::getAltEncoderSampleDelta() {
        return getParamAs<uint8_t>(Parameter::kAltEncoderSampleDelta, "AltEncoderSampleDelta");
    }

    bool SparkBase::getAltEncoderInverted() {
        return getParamAs<bool>(Parameter::kAltEncoderInverted, "AltEncoderInverted");
    }

    float SparkBase::getAltEncoderPositionFactor() {
        return getParamAs<float>(Parameter::kAltEncoderPositionFactor, "AltEncoderPositionFactor");
    }

    float SparkBase::getAltEncoderVelocityFactor() {
        return getParamAs<float>(Parameter::kAltEncoderVelocityFactor, "AltEncoderVelocityFactor");
    }

    // Duty Cycle Absolute Encoder //

    float SparkBase::getDutyCyclePositionFactor() {
        return getParamAs<float>(Parameter::kDutyCyclePositionFactor, "DutyCyclePositionFactor");
    }

    float SparkBase::getDutyCycleVelocityFactor() {
        return getParamAs<float>(Parameter::kDutyCycleVelocityFactor, "DutyCycleVelocityFactor");
    }

    bool SparkBase::getDutyCycleInverted() {
        return getParamAs<bool>(Parameter::kDutyCycleInverted, "DutyCycleInverted");
    }

    uint8_t SparkBase::getDutyCycleAverageDepth() {
        return getParamAs<uint8_t>(Parameter::kDutyCycleAverageDepth, "DutyCycleAverageDepth");
    }

    uint8_t SparkBase::getDutyCyclePrescalar() {
        return getParamAs<uint8_t>(Parameter::kDutyCyclePrescalar, "DutyCyclePrescalar");
    }

    float SparkBase::getDutyCycleZeroOffset() {
        return getParamAs<float>(Parameter::kDutyCycleZeroOffset, "DutyCycleZeroOffset");
    }
}
