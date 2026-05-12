#pragma once

#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <thread>
#include <variant>
#include <vector>
#include <rclcpp/logger.hpp>

#include "can_util/can_controller.hpp"

/**
 * This code is largely based on sparkcan
 */
namespace wheels_interface {
    constexpr auto DEVICE_TYPE = 0x02; ///< Device type for SPARK controllers
    constexpr auto MANUFACTURER = 0x05; ///< Manufacturer ID for REV Robotics

    constexpr auto READ_TIMEOUT_US = 20000; ///< Timeout for reading CAN messages in microseconds

    constexpr uint8_t PARAM_TYPE_UINT = 0x01; ///< Parameter type for unsigned integers
    constexpr uint8_t PARAM_TYPE_FLOAT = 0x02; ///< Parameter type for floating-point numbers
    constexpr uint8_t PARAM_TYPE_BOOL = 0x03; ///< Parameter type for boolean values

    /**
     * @brief API commands for the SPARK controller
     *
     * Each command is associated with a specific API class and index
     * The general structure is formatting as (APIClass << 4) | APIIndex
     */
    enum class APICommand : uint16_t {
        ClearFaults = 6 << 4 | 14,
        FactoryDefaults = 7 << 4 | 4,
        FactoryReset = 7 << 4 | 5,
        Identify = 7 << 4 | 6,
        Heartbeat = 11 << 4 | 2,
        BurnFlash = 63 << 4 | 2,
        FirmwareVersion = 9 << 4 | 8,

        Setpoint = 0 << 4 | 1,
        DutyCycle = 0 << 4 | 2,
        Velocity = 1 << 4 | 2,
        SmartVelocity = 1 << 4 | 3,
        Position = 3 << 4 | 2,
        Voltage = 4 << 4 | 2,
        Current = 4 << 4 | 3,
        SmartMotion = 5 << 4 | 2,

        Period0 = 6 << 4 | 0,
        Period1 = 6 << 4 | 1,
        Period2 = 6 << 4 | 2,
        Period3 = 6 << 4 | 3,
        Period4 = 6 << 4 | 4
    };

    /**
     * @brief Parameters for the SPARK controller
     */
    enum class Parameter : uint32_t {
        kInputMode = 1,
        kMotorType = 2,
        kCommAdvance = 3,
        kSensorType = 4,
        kCtrlType = 5,
        kIdleMode = 6,
        kInputDeadband = 7,
        kFeedbackSensorPID0 = 8,
        kFeedbackSensorPID1 = 9,
        kPolePairs = 10,
        kCurrentChop = 11,
        kCurrentChopCycles = 12,
        kP_0 = 13,
        kI_0 = 14,
        kD_0 = 15,
        kF_0 = 16,
        kIZone_0 = 17,
        kDFilter_0 = 18,
        kOutputMin_0 = 19,
        kOutputMax_0 = 20,
        kP_1 = 21,
        kI_1 = 22,
        kD_1 = 23,
        kF_1 = 24,
        kIZone_1 = 25,
        kDFilter_1 = 26,
        kOutputMin_1 = 27,
        kOutputMax_1 = 28,
        kP_2 = 29,
        kI_2 = 30,
        kD_2 = 31,
        kF_2 = 32,
        kIZone_2 = 33,
        kDFilter_2 = 34,
        kOutputMin_2 = 35,
        kOutputMax_2 = 36,
        kP_3 = 37,
        kI_3 = 38,
        kD_3 = 39,
        kF_3 = 40,
        kIZone_3 = 41,
        kDFilter_3 = 42,
        kOutputMin_3 = 43,
        kOutputMax_3 = 44,
        kInverted = 45,
        kOutputRatio = 46,
        kSerialNumberLow = 47,
        kSerialNumberMid = 48,
        kSerialNumberHigh = 49,
        kLimitSwitchFwdPolarity = 50,
        kLimitSwitchRevPolarity = 51,
        kHardLimitFwdEn = 52,
        kHardLimitRevEn = 53,
        kSoftLimitFwdEn = 54,
        kSoftLimitRevEn = 55,
        kRampRate = 56,
        kFollowerID = 57,
        kFollowerConfig = 58,
        kSmartCurrentStallLimit = 59,
        kSmartCurrentFreeLimit = 60,
        kSmartCurrentConfig = 61,
        kMotorKv = 63,
        kMotorR = 64,
        kMotorL = 65,
        kEncoderCountsPerRev = 69,
        kEncoderAverageDepth = 70,
        kEncoderSampleDelta = 71,
        kEncoderInverted = 72,
        kClosedLoopVoltageMode = 74,
        kCompensatedNominalVoltage = 75,
        kSmartMotionMaxVelocity_0 = 76,
        kSmartMotionMaxAccel_0 = 77,
        kSmartMotionMinVelOutput_0 = 78,
        kSmartMotionAllowedClosedLoopError_0 = 79,
        kSmartMotionAccelStrategy_0 = 80,
        kSmartMotionMaxVelocity_1 = 81,
        kSmartMotionMaxAccel_1 = 82,
        kSmartMotionMinVelOutput_1 = 83,
        kSmartMotionAllowedClosedLoopError_1 = 84,
        kSmartMotionAccelStrategy_1 = 85,
        kSmartMotionMaxVelocity_2 = 86,
        kSmartMotionMaxAccel_2 = 87,
        kSmartMotionMinVelOutput_2 = 88,
        kSmartMotionAllowedClosedLoopError_2 = 89,
        kSmartMotionAccelStrategy_2 = 90,
        kSmartMotionMaxVelocity_3 = 91,
        kSmartMotionMaxAccel_3 = 92,
        kSmartMotionMinVelOutput_3 = 93,
        kSmartMotionAllowedClosedLoopError_3 = 94,
        kSmartMotionAccelStrategy_3 = 95,
        kIMaxAccum_0 = 96,
        kSlot3Placeholder1_0 = 97,
        kSlot3Placeholder2_0 = 98,
        kSlot3Placeholder3_0 = 99,
        kIMaxAccum_1 = 100,
        kSlot3Placeholder1_1 = 101,
        kSlot3Placeholder2_1 = 102,
        kSlot3Placeholder3_1 = 103,
        kIMaxAccum_2 = 104,
        kSlot3Placeholder1_2 = 105,
        kSlot3Placeholder2_2 = 106,
        kSlot3Placeholder3_2 = 107,
        kIMaxAccum_3 = 108,
        kSlot3Placeholder1_3 = 109,
        kSlot3Placeholder2_3 = 110,
        kSlot3Placeholder3_3 = 111,
        kPositionConversionFactor = 112,
        kVelocityConversionFactor = 113,
        kClosedLoopRampRate = 114,
        kSoftLimitFwd = 115,
        kSoftLimitRev = 116,
        kAnalogPositionConversion = 119,
        kAnalogVelocityConversion = 120,
        kAnalogAverageDepth = 121,
        kAnalogSensorMode = 122,
        kAnalogInverted = 123,
        kAnalogSampleDelta = 124,
        kDataPortConfig = 127,
        kAltEncoderCountsPerRev = 128,
        kAltEncoderAverageDepth = 129,
        kAltEncoderSampleDelta = 130,
        kAltEncoderInverted = 131,
        kAltEncoderPositionFactor = 132,
        kAltEncoderVelocityFactor = 133,
        kHallSensorSampleRate = 136,
        kHallSensorAverageDepth = 137,
        kDutyCyclePositionFactor = 139,
        kDutyCycleVelocityFactor = 140,
        kDutyCycleInverted = 141,
        kDutyCycleAverageDepth = 143,
        kPositionPIDWrapEnable = 149,
        kPositionPIDMinInput = 150,
        kPositionPIDMaxInput = 151,
        kDutyCyclePrescalar = 153,
        kDutyCycleZeroOffset = 154
    };

    /**
     * @brief Motor type parameter
     */
    enum class MotorType : uint8_t {
        kBrushed = 0,
        kBrushless = 1
    };

    /**
     * @brief Sensor type parameter
     */
    enum class SensorType : uint8_t {
        kNoSensor = 0,
        kHallSensor = 1,
        kEncoder = 2
    };

    /**
     * @brief Control type parameter
     */
    enum class CtrlType : uint8_t {
        kDutyCycle = 0,
        kVelocity = 1,
        kVoltage = 2,
        kPosition = 3
    };

    /**
     * @brief Idle mode parameter
     */
    enum class IdleMode : uint8_t {
        kCoast = 0,
        kBrake = 1
    };

    /**
     * @brief Periodic status 0 structure
     */
    struct Period0Status {
        float dutyCycle;
        uint16_t faults;
        uint16_t stickyFaults;
        bool isInverted;
        bool idleMode;
        bool isFollower;
        std::chrono::steady_clock::time_point timestamp;
    };

    /**
     * @brief Periodic status 1 structure
     */
    struct Period1Status {
        float velocity;
        float temperature;
        float voltage;
        float current;
        std::chrono::steady_clock::time_point timestamp;
    };

    /**
     * @brief Periodic status 2 structure
     */
    struct Period2Status {
        float position;
        float iAccum;
        std::chrono::steady_clock::time_point timestamp;
    };

    /**
     * @brief Periodic status 3 structure
     */
    struct Period3Status {
        float analogVoltage;
        float analogVelocity;
        float analogPosition;
        std::chrono::steady_clock::time_point timestamp;
    };

    /**
     * @brief Periodic status 4 structure
     */
    struct Period4Status {
        float altEncoderVelocity;
        float altEncoderPosition;
        std::chrono::steady_clock::time_point timestamp;
    };

    /**
     * @class SparkBase
     * @brief A base class for controlling REV Robotics SPARK motor controllers via CAN bus
     *
     * This class provides methods to configure, control, and monitor SPARK motor controllers
     * It supports various control modes, parameter settings, and periodic status readings
     */
    class SparkBase {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(SparkBase)

        /**
         * @brief Initializes SparkBase with the specified CAN interface and ID
         *
         * @param interfaceName The name of the CAN interface (e.g., "can0")
         * @param deviceId The CAN ID of the SPARK controller (0-62)
         * @throws std::out_of_range if deviceId_ is greater than 62
         * @throws std::system_error if socket creation fails, with detailed error information
         * @throws std::runtime_error if IOCTL fails or binding to the interface fails, with detailed error information
         *
         * @details This constructor attempts to initialize the CAN bus connection. If it fails, it will throw
         * an exception with a detailed error message that includes possible causes and suggested solutions.
         * Common issues that may cause exceptions include:
         * - Invalid device ID
         * - CAN modules not loaded
         * - System resource limitations
         * - CAN interface not existing
         * - CAN interface not being up
         * - CAN bus not initialized
         * - Interface already bound to another program
         */
        SparkBase(rclcpp::Logger& logger, can_util::CANController& can_controller, uint8_t deviceId);

        virtual ~SparkBase() = default;

    private:
        ros2_fmt_logger::Logger logger;
        can_util::CANController& can_controller;
        uint8_t device_id; ///< Device ID for the SPARK controller on the CAN bus

        std::shared_ptr<can_util::CANFrameCallback> frame_callback;

        mutable std::mutex mtx;

        // Periodic status structures
        Period0Status period0{};
        Period1Status period1{};
        Period2Status period2{};
        Period3Status period3{};
        Period4Status period4{};

        /**
         * @brief Sends a CAN frame
         *
         * @param cmd The API command associated with the CAN frame
         * @param data The data payload to send in the CAN frame
         */
        [[nodiscard]] bool sendCanFrame(APICommand cmd, const std::vector<uint8_t>& data) const;

        /**
         * @brief Sends a CAN frame with a custom arbitration ID
         *
         * @param arbId The full CAN arbitration ID
         * @param data The data payload to send in the CAN frame
         */
        [[nodiscard]] bool sendCanFrame(uint32_t arbId, const std::vector<uint8_t>& data) const;

        /**
         * @brief Sends a control message to the SPARK controller
         *
         * @param cmd The API command associated with the control message
         * @param commandName The control command's name
         * @param value The value associated with the control command
         * @param minValue The minimum allowed value for the command (optional)
         * @param maxValue The maximum allowed value for the command (optional)
         *
         * @throws std::invalid_argument If the command value is not finite
         * @throws std::out_of_range If the value is outside the specified range (will default to min and max of datatype
         * when not provided)
         */
        [[nodiscard]] bool sendControlMessage(
            APICommand cmd,
            const std::string& commandName,
            float value,
            std::optional<float> minValue = std::nullopt,
            std::optional<float> maxValue = std::nullopt
        ) const;

        /**
         * @brief Creates an arbitration ID for the SPARK controller
         *
         * @param cmd The API command for which to create the arbitration ID
         * @return uint32_t The created frame ID
         */
        uint32_t createArbId(APICommand cmd) const;

        /**
         * @brief Creates an arbitration ID for a parameter-specific message
         *
         * @param paramId The parameter ID to encode in the arbitration ID
         * @return uint32_t The created frame ID
         */
        uint32_t createParamArbId(Parameter paramId) const;

        /**
         * @brief Gets the API class from an API command
         *
         * @param cmd The API command
         * @return constexpr uint8_t The API class extracted from the command
         */
        static uint8_t getAPIClass(APICommand cmd);

        /**
         * @brief Gets the API index from an API command
         *
         * @param cmd The API command
         * @return constexpr uint8_t The API index extracted from the command
         */
        static uint8_t getAPIIndex(APICommand cmd);

        /**
         * @brief Continuously reads periodic status data from the SPARK controller
         * Updates the status structures with the received data
         */
        void handleFrame(uint32_t id, const std::vector<uint8_t>& data);

        /**
         * @brief Sets a parameter on the SPARK controller
         *
         * @param parameterId The ID of the parameter to set
         * @param parameterType The type of the parameter (e.g., UINT, FLOAT, BOOL)
         * @param parameterName The name of the parameter
         * @param value The value to set the parameter to (can be float, uint32_t, uint16_t, uint8_t, or bool)
         * @param minValue The minimum allowed value for the parameter (optional)
         * @param maxValue The maximum allowed value for the parameter (optional)
         * @param customErrorMessage A custom error message to use if the value is out of range (optional)
         *
         * @throws std::invalid_argument If the parameter type is invalid or if a float value is not finite
         * @throws std::out_of_range If the value is outside the specified range (will default to min and max of datatype
         * when not provided)
         */
        // TODO 2026-02-25 (Will Free): convert this to a template function?
        void setParameter(
            Parameter parameterId,
            uint8_t parameterType,
            const std::string& parameterName,
            std::variant<float, uint32_t, uint16_t, uint8_t, bool> value,
            std::optional<float> minValue = std::nullopt,
            std::optional<float> maxValue = std::nullopt,
            const std::optional<std::string>& customErrorMessage = std::nullopt
        );

        /**
         * @brief Reads the value of a specified parameter from the device
         *
         * @param parameterId The ID of the parameter to read (enumerated by the Parameter type)
         * @return std::optional<std::variant<float, uint32_t, bool>> The parameter value if received, or std::nullopt if no
         * response
         */
        // TODO 2026-02-25 (Will Free): convert this to a template function?
        std::optional<std::variant<float, uint32_t, bool>> readParameter(Parameter parameterId) const;

        /**
         * @brief Internal helper to get a parameter as a specific type
         *        If no response or wrong type, returns default based on type
         * @tparam T The expected return type
         * @param param The parameter ID
         * @param name A debug name used in error messages
         * @return Value of parameter or type-based default fallback
         */
        template <typename T>
        T getParamAs(Parameter param, const char* name);

        /**
         * @brief Internal helper to get a slot-indexed parameter
         *
         * @tparam T The expected return type
         * @param baseParam The base parameter enum for slot 0
         * @param slot The PID slot index (0–3)
         * @param name A debug name used in error messages
         * @return Value of the slot-specific parameter
         */
        template <typename T>
        T getPIDParam(Parameter baseParam, uint8_t slot, const char* name);

        // TODO 2026-02-20 (Will Free): mark all methods that return something as [[nodiscard]]

    public:
        /**
         * @brief Requests the firmware version from the SPARK controller
         * @return A tuple of (major, minor, patch, build, isDebugBuild), or std::nullopt on failure
         */
        std::optional<std::tuple<uint8_t, uint8_t, uint8_t, uint8_t, bool>> readFirmwareVersion() const;

        uint8_t getDeviceId() const {
            return device_id;
        }

        // System Control Methods //

        /**
         * @brief Sends a heartbeat signal to keep all SPARK controllers active
         */
        bool heartbeat() const;

        /**
         * @brief Resets all faults on the SPARK controller
         */
        bool resetFaults() const;

        /**
         * @brief Clears sticky faults on the SPARK controller
         */
        bool clearStickyFaults() const;

        /**
         * @brief Burns the current configuration to the SPARK controller's flash memory
         */
        bool burnFlash() const;

        /**
         * @brief Resets the SPARK controller to factory default settings
         */
        bool factoryDefaults() const;

        /**
         * @brief Performs a factory reset on the SPARK controller
         */
        bool factoryReset() const;

        /**
         * @brief Triggers the SPARK controller to identify itself
         */
        bool identify() const;

        // Motor Control Methods //

        /**
         * @brief Sets the value for the currently set control type
         * @param setpoint The desired value
         */
        bool setSetpoint(float setpoint) const;

        /**
         * @brief Sets the motor's applied output
         * @param dutyCycle The desired applied output, range: [-1.0, 1.0]
         */
        bool setDutyCycle(float dutyCycle) const;

        /**
         * @brief Sets the motor's velocity
         * @param velocity The desired velocity
         */
        bool setVelocity(float velocity) const;

        /**
         * @brief Sets the motor's smart velocity
         * @param smartVelocity The desired smart velocity
         */
        bool setSmartVelocity(float smartVelocity) const;

        /**
         * @brief Sets the motor's position
         * @param position The desired position
         */
        bool setPosition(float position) const;

        /**
         * @brief Sets the motor's voltage
         * @param voltage The desired voltage
         */
        bool setVoltage(float voltage) const;

        /**
         * @brief Sets the motor's current
         * @param current The desired current
         */
        bool setCurrent(float current) const;

        /**
         * @brief Sets the motor's smart motion
         * @param smartMotion The desired smart motion value
         */
        bool setSmartMotion(float smartMotion) const;

        // Status Methods //

        /**
         * @brief Sets the period for periodic status 0
         * @param period The desired status period in milliseconds
         */
        bool setPeriodicStatus0Period(uint16_t period) const;

        /**
         * @brief Sets the period for periodic status 1
         * @param period The desired status period in milliseconds
         */
        bool setPeriodicStatus1Period(uint16_t period) const;

        /**
         * @brief Sets the period for periodic status 2
         * @param period The desired status period in milliseconds
         */
        bool setPeriodicStatus2Period(uint16_t period) const;

        /**
         * @brief Sets the period for periodic status 3
         * @param period The desired status period in milliseconds
         */
        bool setPeriodicStatus3Period(uint16_t period) const;

        /**
         * @brief Sets the period for periodic status 4
         * @param period The desired status period in milliseconds
         */
        bool setPeriodicStatus4Period(uint16_t period) const;

        /**
         * @brief Retrieves the current applied output
         * @return float The applied output, range: [-1.0, 1.0]
         */
        float getDutyCycle() const;

        /**
         * @brief Retrieves the current faults
         * @return uint16_t A bitfield representing the current faults
         */
        uint16_t getFaults() const;

        /**
         * @brief Retrieves the sticky faults
         * @return uint16_t A bitfield representing the sticky faults
         */
        uint16_t getStickyFaults() const;

        /**
         * @brief Checks if the motor is inverted
         * @return bool True if the motor is inverted, false otherwise
         */
        bool isInverted() const;

        /**
         * @brief Gets the current idle mode
         * @return bool True if in brake mode, false if in coast mode
         */
        bool getIdleMode() const;

        /**
         * @brief Checks if the SPARK controller is in follower mode
         * @return bool True if in follower mode, false otherwise
         */
        bool isFollower() const;

        /**
         * @brief Gets the current velocity
         * @return float The current velocity in RPM
         */
        float getVelocity() const;

        /**
         * @brief Gets the current temperature of the SPARK controller
         * @return float The temperature in degrees Celsius
         */
        float getTemperature() const;

        /**
         * @brief Gets the current voltage of the SPARK controller
         * @return float The voltage in volts
         */
        float getVoltage() const;

        /**
         * @brief Gets the current drawn by the SPARK controller
         * @return float The current in amperes
         */
        float getCurrent() const;

        /**
         * @brief Gets the current position of the motor
         * @return float The position in ticks
         */
        float getPosition() const;

        /**
         * @brief Gets the integral accumulator
         * @return float The integral accumulator
         */
        float getIAccum() const;

        /**
         * @brief Gets the current analog voltage
         * @return float The analog voltage in volts
         */
        float getAnalogVoltage() const;

        /**
         * @brief Gets the current analog velocity
         * @return float The analog velocity in RPM
         */
        float getAnalogVelocity() const;

        /**
         * @brief Gets the current analog position
         * @return float The analog position in ticks
         */
        float getAnalogPosition() const;

        /**
         * @brief Gets the velocity from the alternate encoder
         * @return float The alternate encoder velocity in RPM
         */
        float getAltEncoderVelocity() const;

        /**
         * @brief Gets the position from the alternate encoder
         * @return float The alternate encoder position in ticks
         */
        float getAltEncoderPosition() const;

        // Parameter Setters //

        /**
         * @brief Sets the input mode
         * @param mode The input mode
         */
        void setInputMode(uint8_t mode);

        /**
         * @brief Sets the motor type
         * @param type MotorType::kBrushed for Brushed, MotorType::kBrushless for Brushless
         */
        void setMotorType(MotorType type);

        /**
         * @brief Sets the sensor type
         * @param sensor SensorType::kNoSensor for No Sensor, SensorType::kHallSensor for Hall Sensor, SensorType::kEncoder
         * for Encoder
         */
        void setSensorType(SensorType sensor);

        /**
         * @brief Sets the idle mode
         * @param mode IdleMode::kCoast for Coast, IdleMode::kBrake for Brake
         */
        void setIdleMode(IdleMode mode);

        /**
         * @brief Sets the input deadband
         *
         * @param deadband The deadband value
         */
        void setInputDeadband(float deadband);

        /**
         * @brief Sets whether the motor is inverted
         * @param inverted True to invert the motor, false otherwise
         */
        void setInverted(bool inverted);

        /**
         * @brief Sets the ramp rate
         * @param rate The ramp rate in seconds from neutral to full output
         */
        void setRampRate(float rate);

        // Advanced //

        /**
         * @brief Sets the motor Kv (velocity constant)
         * @param kv The Kv value
         */
        void setMotorKv(uint16_t kv);

        /**
         * @brief Sets the motor resistance
         * @param r The resistance value
         */
        void setMotorR(uint16_t r);

        /**
         * @brief Sets the motor inductance
         * @param l The inductance value
         */
        void setMotorL(uint16_t l);

        // Closed Loop //

        /**
         * @brief Sets the control type
         * @param type CtrlType::kDutyCycle for Duty Cycle, CtrlType::kVelocity for Velocity, CtrlType::kVoltage for
         * Voltage, CtrlType::kPosition for Position
         */
        void setCtrlType(CtrlType type);

        /**
         * @brief Sets the feedback sensor for PID0
         * @param sensor The sensor type
         */
        void setFeedbackSensorPID0(uint16_t sensor);

        /**
         * @brief Sets the closed loop voltage mode
         * @param mode 0 for Disabled, 1 for Control Loop Voltage Output Mode, 2 for Voltage Compensation Mode

         */
        void setClosedLoopVoltageMode(uint8_t mode);

        /**
         * @brief Sets the compensated nominal voltage
         * @param voltage The nominal voltage to compensate for
         */
        void setCompensatedNominalVoltage(float voltage);

        /**
         * @brief Enables or disables position PID wrap
         * @param enable True to enable, false to disable
         */
        void setPositionPIDWrapEnable(bool enable);

        /**
         * @brief Sets the minimum input for position PID
         * @param minInput The minimum input value
         */
        void setPositionPIDMinInput(float minInput);

        /**
         * @brief Sets the maximum input for position PID
         * @param maxInput The maximum input value
         * @throws std::invalid_argument if maxInput is not a finite number
         */
        void setPositionPIDMaxInput(float maxInput);

        // Brushless //

        /**
         * @brief Sets the number of pole pairs for brushless motors
         * @param pairs The number of pole pairs
         */
        void setPolePairs(uint16_t pairs);

        // Current Limit //

        /**
         * @brief Sets the current chop limit
         * @param chop The current chop limit (0-125 amps)
         */
        void setCurrentChop(float chop);

        /**
         * @brief Sets the number of cycles for current chopping
         * @param cycles The number of cycles
         */
        void setCurrentChopCycles(uint16_t cycles);

        /**
         * @brief Sets the smart current stall limit
         * @param limit The stall current limit
         */
        void setSmartCurrentStallLimit(uint16_t limit);

        /**
         * @brief Sets the smart current free limit
         * @param limit The free current limit
         */
        void setSmartCurrentFreeLimit(uint16_t limit);

        /**
         * @brief Sets the smart current configuration
         * @param config The configuration value
         */
        void setSmartCurrentConfig(uint16_t config);

        // PIDF //

        /**
         * @brief Sets the proportional gain for the specified PID slot
         * @param slot The PID slot (0-3)
         * @param p The proportional gain value
         */
        void setP(uint8_t slot, float p);

        /**
         * @brief Sets the integral gain for the specified PID slot
         * @param slot The PID slot (0-3)
         * @param i The integral gain value
         * @throws std::out_of_range if slot is greater than 3
         */
        void setI(uint8_t slot, float i);

        /**
         * @brief Sets the derivative gain for the specified PID slot
         * @param slot The PID slot (0-3)
         * @param d The derivative gain value
         * @throws std::out_of_range if slot is greater than 3
         */
        void setD(uint8_t slot, float d);

        /**
         * @brief Sets the feedforward gain for the specified PID slot
         * @param slot The PID slot (0-3)
         * @param f The feedforward gain value
         * @throws std::out_of_range if slot is greater than 3
         */
        void setF(uint8_t slot, float f);

        /**
         * @brief Sets the integral zone for the specified PID slot
         * @param slot The PID slot (0-3)
         * @param iZone The integral zone value
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setIZone(uint8_t slot, float iZone);

        /**
         * @brief Sets the derivative filter for the specified PID slot
         * @param slot The PID slot (0-3)
         * @param dFilter The derivative filter value
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setDFilter(uint8_t slot, float dFilter);

        /**
         * @brief Sets the output minimum for the specified PID slot
         * @param slot The PID slot (0-3)
         * @param min The minimum output value
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setOutputMin(uint8_t slot, float min);

        /**
         * @brief Sets the output maximum for the specified PID slot
         * @param slot The PID slot (0-3)
         * @param max The maximum output value
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setOutputMax(uint8_t slot, float max);

        // Limits //

        /**
         * @brief Enables or disables the forward hard limit switch
         * @param enable True to enable, false to disable
         */
        void setHardLimitFwdEn(bool enable);

        /**
         * @brief Enables or disables the reverse hard limit switch
         * @param enable True to enable, false to disable
         */
        void setHardLimitRevEn(bool enable);

        /**
         * @brief Sets the polarity of the forward limit switch
         * @param polarity True for normally open, false for normally closed
         */
        void setLimitSwitchFwdPolarity(bool polarity);

        /**
         * @brief Sets the polarity of the reverse limit switch
         * @param polarity True for normally open, false for normally closed
         */
        void setLimitSwitchRevPolarity(bool polarity);

        /**
         * @brief Enables or disables the forward soft limit
         * @param enable True to enable, false to disable
         */
        void setSoftLimitFwdEn(bool enable);

        /**
         * @brief Enables or disables the reverse soft limit
         * @param enable True to enable, false to disable
         */
        void setSoftLimitRevEn(bool enable);

        /**
         * @brief Sets the forward soft limit
         * @param limit The forward soft limit value
         */
        void setSoftLimitFwd(float limit);

        /**
         * @brief Sets the reverse soft limit
         * @param limit The reverse soft limit value
         */
        void setSoftLimitRev(float limit);

        // Follower //

        /**
         * @brief Sets the follower ID for this SparkBase
         * @param id The CAN ID of the SPARK controller to follow
         */
        void setFollowerID(uint32_t id);

        /**
         * @brief Sets the follower configuration
         * @param config The follower configuration value
         */
        void setFollowerConfig(uint32_t config);

        // Encoder Port //

        /**
         * @brief Sets the encoder counts per revolution
         * @param counts The number of counts per revolution
         */
        void setEncoderCountsPerRev(uint16_t counts);

        /**
         * @brief Sets the encoder average depth
         * @param depth The average depth (1-64)
         */
        void setEncoderAverageDepth(uint8_t depth);

        /**
         * @brief Sets the encoder sample delta
         * @param delta The sample delta (1-255)
         */
        void setEncoderSampleDelta(uint8_t delta);

        /**
         * @brief Sets whether the encoder is inverted
         * @param inverted True to invert the encoder, false otherwise
         */
        void setEncoderInverted(bool inverted);

        /**
         * @brief Sets the position conversion factor
         * @param factor The position conversion factor
         */
        void setPositionConversionFactor(float factor);

        /**
         * @brief Sets the velocity conversion factor
         * @param factor The velocity conversion factor
         */
        void setVelocityConversionFactor(float factor);

        /**
         * @brief Sets the closed loop ramp rate
         * @param rampRate The ramp rate in seconds from neutral to full output
         */
        void setClosedLoopRampRate(float rampRate);

        /**
         * @brief Sets the hall sensor sample rate
         * @param rate The sample rate in Hz
         */
        void setHallSensorSampleRate(float rate);

        /**
         * @brief Sets the hall sensor average depth
         * @param depth The average depth
         */
        void setHallSensorAverageDepth(uint16_t depth);

        // Smart Motion //

        /**
         * @brief Sets the maximum velocity for Smart Motion in the specified slot
         * @param slot The Smart Motion slot (0-3)
         * @param maxVel The maximum velocity
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setSmartMotionMaxVelocity(uint8_t slot, float maxVel);

        /**
         * @brief Sets the maximum acceleration for Smart Motion in the specified slot
         * @param slot The Smart Motion slot (0-3)
         * @param maxAccel The maximum acceleration
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setSmartMotionMaxAccel(uint8_t slot, float maxAccel);

        /**
         * @brief Sets the minimum velocity output for Smart Motion in the specified slot
         * @param slot The Smart Motion slot (0-3)
         * @param minVel The minimum velocity
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setSmartMotionMinVelOutput(uint8_t slot, float minVel);

        /**
         * @brief Sets the allowed closed loop error for Smart Motion in the specified slot
         * @param slot The Smart Motion slot (0-3)
         * @param error The allowed closed loop error
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setSmartMotionAllowedClosedLoopError(uint8_t slot, float error);

        /**
         * @brief Sets the acceleration strategy for Smart Motion in the specified slot
         * @param slot The Smart Motion slot (0-3)
         * @param strategy The acceleration strategy
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setSmartMotionAccelStrategy(uint8_t slot, float strategy);

        /**
         * @brief Sets the maximum accumulator value for the I term in the specified slot
         * @param slot The PID slot (0-3)
         * @param maxAccum The maximum accumulator value
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setIMaxAccum(uint8_t slot, float maxAccum);

        /**
         * @brief Sets a placeholder value for slot 3 (purpose undefined)
         * @param slot The PID slot (0-3)
         * @param value The placeholder value
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setSlot3Placeholder1(uint8_t slot, float value);

        /**
         * @brief Sets a placeholder value for slot 3 (purpose undefined)
         * @param slot The PID slot (0-3)
         * @param value The placeholder value
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setSlot3Placeholder2(uint8_t slot, float value);

        /**
         * @brief Sets a placeholder value for slot 3 (purpose undefined)
         * @param slot The PID slot (0-3)
         * @param value The placeholder value
         * @throws std::invalid_argument if slot is greater than 3
         */
        void setSlot3Placeholder3(uint8_t slot, float value);

        // Analog Sensor //

        /**
         * @brief Sets the conversion factor for analog position readings
         * @param factor The conversion factor to apply to raw analog readings
         */
        void setAnalogPositionConversion(float factor);

        /**
         * @brief Sets the conversion factor for analog velocity readings
         * @param factor The conversion factor to apply to raw analog readings
         */
        void setAnalogVelocityConversion(float factor);

        /**
         * @brief Sets the average depth for analog readings
         * @param depth The average depth
         */
        void setAnalogAverageDepth(uint16_t depth);

        /**
         * @brief Sets the analog sensor mode
         * @param mode 0 for Absolute, 1 for Relative
         */
        void setAnalogSensorMode(uint8_t mode);

        /**
         * @brief Sets whether the analog sensor is inverted
         * @param inverted True to invert the sensor, false otherwise
         */
        void setAnalogInverted(bool inverted);

        /**
         * @brief Sets the sample delta for analog readings
         * @param delta The sample delta
         */
        void setAnalogSampleDelta(uint16_t delta);

        // Alternate Encoder  //

        /**
         * @brief Configures the data port
         * @param config 0 for Default, 1 for Alternate Encoder Mode
         */
        void setDataPortConfig(uint8_t config);

        /**
         * @brief Sets the counts per revolution for the alternate encoder
         * @param counts The number of counts per revolution
         */
        void setAltEncoderCountsPerRev(uint16_t counts);

        /**
         * @brief Sets the average depth for the alternate encoder
         * @param depth The average depth (1-64)
         */
        void setAltEncoderAverageDepth(uint8_t depth);

        /**
         * @brief Sets the sample delta for the alternate encoder
         * @param delta The sample delta (1-255)
         */
        void setAltEncoderSampleDelta(uint8_t delta);

        /**
         * @brief Sets whether the alternate encoder is inverted
         * @param inverted True to invert the encoder, false otherwise
         */
        void setAltEncoderInverted(bool inverted);

        /**
         * @brief Sets the position factor for the alternate encoder
         * @param factor The position factor
         */
        void setAltEncoderPositionFactor(float factor);

        /**
         * @brief Sets the velocity factor for the alternate encoder
         * @param factor The velocity factor
         */
        void setAltEncoderVelocityFactor(float factor);

        // Duty Cycle Absolute Encoder //

        /**
         * @brief Sets the position factor for the duty cycle encoder
         * @param factor The position factor
         */
        void setDutyCyclePositionFactor(float factor);

        /**
         * @brief Sets the velocity factor for the duty cycle encoder
         * @param factor The velocity factor
         */
        void setDutyCycleVelocityFactor(float factor);

        /**
         * @brief Sets whether the duty cycle encoder is inverted
         * @param inverted True to invert the encoder, false otherwise
         */
        void setDutyCycleInverted(bool inverted);

        /**
         * @brief Sets the average depth for the duty cycle encoder
         * @param depth The average depth (0-7)
         */
        void setDutyCycleAverageDepth(uint8_t depth);

        /**
         * @brief Sets the prescalar for the duty cycle encoder
         * @param prescalar The prescalar value (0-71)
         */
        void setDutyCyclePrescalar(uint8_t prescalar);

        /**
         * @brief Sets the zero offset for the duty cycle encoder
         * @param offset The zero offset (0-1)
         */
        void setDutyCycleZeroOffset(float offset);

        // Parameter Getters //

        // Basic //
        /**
         * @brief Get the motor type
         * @return Motor type as uint8_t (0 = Brushed, 1 = Brushless)
         */
        uint8_t getMotorType();

        /**
         * @brief Get the sensor type
         * @return Sensor type as uint8_t (0 = No Sensor, 1 = Hall Sensor, 2 = Encoder)
         */
        uint8_t getSensorType();

        /**
         * @brief Get the idle mode
         * @return Idle mode as uint8_t (0 = Coast, 1 = Brake)
         */
        uint8_t getIdleMode();

        /**
         * @brief Get the input deadband
         * @return Input deadband as float
         */
        float getInputDeadband();

        /**
         * @brief Get whether the motor is inverted
         * @return Inverted state as bool
         */
        bool getInverted();

        /**
         * @brief Get the ramp rate
         * @return Ramp rate as float
         */
        float getRampRate();

        // Advanced //

        /**
         * @brief Get the motor Kv rating
         * @return Motor Kv as uint16_t
         */
        uint16_t getMotorKv();

        /**
         * @brief Get the motor resistance
         * @return Motor resistance as uint16_t
         */
        uint16_t getMotorR();

        /**
         * @brief Get the motor inductance
         * @return Motor inductance as uint16_t
         */
        uint16_t getMotorL();

        // Closed Loop //

        /**
         * @brief Get the control type
         * @return Control type as uint8_t (0 = Duty Cycle, 1 = Velocity, 2 = Voltage, 3 = Position)
         */
        uint8_t getCtrlType();

        /**
         * @brief Get the feedback sensor PID0 value
         * @return Feedback sensor PID0 as uint16_t
         */
        uint16_t getFeedbackSensorPID0();

        /**
         * @brief Get the closed loop voltage mode
         * @return Closed loop voltage mode as uint8_t
         */
        uint8_t getClosedLoopVoltageMode();

        /**
         * @brief Get the compensated nominal voltage
         * @return Compensated nominal voltage as float
         */
        float getCompensatedNominalVoltage();

        /**
         * @brief Get the position PID wrap enable state
         * @return Position PID wrap enable state as bool
         */
        bool getPositionPIDWrapEnable();

        /**
         * @brief Get the position PID minimum input
         * @return Position PID minimum input as float
         */
        float getPositionPIDMinInput();

        /**
         * @brief Get the position PID maximum input
         * @return Position PID maximum input as float
         */
        float getPositionPIDMaxInput();

        // Brushless //

        /**
         * @brief Get the number of pole pairs
         * @return Pole pairs as uint16_t
         */
        uint16_t getPolePairs();

        // Current Limit //

        /**
         * @brief Get the current chop value
         * @return Current chop as float
         */
        float getCurrentChop();

        /**
         * @brief Get the current chop cycles
         * @return Current chop cycles as uint16_t
         */
        uint16_t getCurrentChopCycles();

        /**
         * @brief Get the smart current stall limit
         * @return Smart current stall limit as uint16_t
         */
        uint16_t getSmartCurrentStallLimit();

        /**
         * @brief Get the smart current free limit
         * @return Smart current free limit as uint16_t
         */
        uint16_t getSmartCurrentFreeLimit();

        /**
         * @brief Get the smart current configuration
         * @return Smart current config as uint16_t
         */
        uint16_t getSmartCurrentConfig();

        // PIDF //

        /**
         * @brief Get the proportional (P) constant for a given slot
         * @param slot The PID slot (0-3)
         * @return Proportional constant as float
         */
        float getP(uint8_t slot);

        /**
         * @brief Get the integral (I) constant for a given slot
         * @param slot The PID slot (0-3)
         * @return Integral constant as float
         */
        float getI(uint8_t slot);

        /**
         * @brief Get the derivative (D) constant for a given slot
         * @param slot The PID slot (0-3)
         * @return Derivative constant as float
         */
        float getD(uint8_t slot);

        /**
         * @brief Get the feedforward (F) constant for a given slot
         * @param slot The PID slot (0-3)
         * @return Feedforward constant as float
         */
        float getF(uint8_t slot);

        /**
         * @brief Get the IZone value for a given slot
         * @param slot The PID slot (0-3)
         * @return IZone value as float
         */
        float getIZone(uint8_t slot);

        /**
         * @brief Get the DFilter value for a given slot
         * @param slot The PID slot (0-3)
         * @return DFilter value as float
         */
        float getDFilter(uint8_t slot);

        /**
         * @brief Get the output minimum value for a given slot
         * @param slot The PID slot (0-3)
         * @return Output minimum value as float
         */
        float getOutputMin(uint8_t slot);

        /**
         * @brief Get the output maximum value for a given slot
         * @param slot The PID slot (0-3)
         * @return Output maximum value as float
         */
        float getOutputMax(uint8_t slot);

        // Limits //

        /**
         * @brief Get the forward hard limit enable state
         * @return Forward hard limit enable state as bool
         */
        bool getHardLimitFwdEn();

        /**
         * @brief Get the reverse hard limit enable state
         * @return Reverse hard limit enable state as bool
         */
        bool getHardLimitRevEn();

        /**
         * @brief Get the forward limit switch polarity
         * @return Forward limit switch polarity as bool
         */
        bool getLimitSwitchFwdPolarity();

        /**
         * @brief Get the reverse limit switch polarity
         * @return Reverse limit switch polarity as bool
         */
        bool getLimitSwitchRevPolarity();

        /**
         * @brief Get the forward soft limit enable state
         * @return Forward soft limit enable state as bool
         */
        bool getSoftLimitFwdEn();

        /**
         * @brief Get the reverse soft limit enable state
         * @return Reverse soft limit enable state as bool
         */
        bool getSoftLimitRevEn();

        /**
         * @brief Get the forward soft limit value
         * @return Forward soft limit as float
         */
        float getSoftLimitFwd();

        /**
         * @brief Get the reverse soft limit value
         * @return Reverse soft limit as float
         */
        float getSoftLimitRev();

        // Follower //

        /**
         * @brief Get the follower ID
         * @return Follower ID as uint32_t
         */
        uint32_t getFollowerID();

        /**
         * @brief Get the follower configuration
         * @return Follower configuration as uint32_t
         */
        uint32_t getFollowerConfig();

        // Encoder Port //

        /**
         * @brief Get the encoder counts per revolution
         * @return Encoder counts per revolution as uint16_t
         */
        uint16_t getEncoderCountsPerRev();

        /**
         * @brief Get the encoder average depth
         * @return Encoder average depth as uint8_t
         */
        uint8_t getEncoderAverageDepth();

        /**
         * @brief Get the encoder sample delta
         * @return Encoder sample delta as uint8_t
         */
        uint8_t getEncoderSampleDelta();

        /**
         * @brief Get the encoder inverted state
         * @return Encoder inverted state as bool
         */
        bool getEncoderInverted();

        /**
         * @brief Get the position conversion factor
         * @return Position conversion factor as float
         */
        float getPositionConversionFactor();

        /**
         * @brief Get the velocity conversion factor
         * @return Velocity conversion factor as float
         */
        float getVelocityConversionFactor();

        /**
         * @brief Get the closed loop ramp rate
         * @return Closed loop ramp rate as float
         */
        float getClosedLoopRampRate();

        /**
         * @brief Get the Hall sensor sample rate
         * @return Hall sensor sample rate as float
         */
        float getHallSensorSampleRate();

        /**
         * @brief Get the Hall sensor average depth
         * @return Hall sensor average depth as uint16_t
         */
        uint16_t getHallSensorAverageDepth();

        // Smart Motion //

        /**
         * @brief Get the maximum velocity for Smart Motion in a given slot
         * @param slot The Smart Motion slot (0-3)
         * @return Maximum velocity as float
         */
        float getSmartMotionMaxVelocity(uint8_t slot);

        /**
         * @brief Get the maximum acceleration for Smart Motion in a given slot
         * @param slot The Smart Motion slot (0-3)
         * @return Maximum acceleration as float
         */
        float getSmartMotionMaxAccel(uint8_t slot);

        /**
         * @brief Get the minimum velocity output for Smart Motion in a given slot
         * @param slot The Smart Motion slot (0-3)
         * @return Minimum velocity output as float
         */
        float getSmartMotionMinVelOutput(uint8_t slot);

        /**
         * @brief Get the allowed closed-loop error for Smart Motion in a given slot
         * @param slot The Smart Motion slot (0-3)
         * @return Allowed closed-loop error as float
         */
        float getSmartMotionAllowedClosedLoopError(uint8_t slot);

        /**
         * @brief Get the acceleration strategy for Smart Motion in a given slot
         * @param slot The Smart Motion slot (0-3)
         * @return Acceleration strategy as float
         */
        float getSmartMotionAccelStrategy(uint8_t slot);

        /**
         * @brief Get the maximum accumulated integral term for Smart Motion in a given slot
         * @param slot The Smart Motion slot (0-3)
         * @return Maximum integral term accumulation as float
         */
        float getIMaxAccum(uint8_t slot);

        /**
         * @brief Get the value for Slot 3 Placeholder 1 in a given slot
         * @param slot The slot (0-3)
         * @return Placeholder value as float
         */
        float getSlot3Placeholder1(uint8_t slot);

        /**
         * @brief Get the value for Slot 3 Placeholder 2 in a given slot
         * @param slot The slot (0-3)
         * @return Placeholder value as float
         */
        float getSlot3Placeholder2(uint8_t slot);

        /**
         * @brief Get the value for Slot 3 Placeholder 3 in a given slot
         * @param slot The slot (0-3)
         * @return Placeholder value as float
         */
        float getSlot3Placeholder3(uint8_t slot);

        // Analog Sensor //

        /**
         * @brief Get the analog position conversion factor
         * @return Analog position conversion factor as float
         */
        float getAnalogPositionConversion();

        /**
         * @brief Get the analog velocity conversion factor
         * @return Analog velocity conversion factor as float
         */
        float getAnalogVelocityConversion();

        /**
         * @brief Get the analog average depth
         * @return Analog average depth as uint16_t
         */
        uint16_t getAnalogAverageDepth();

        /**
         * @brief Get the analog sensor mode
         * @return Analog sensor mode as uint8_t (0 = Absolute, 1 = Relative)
         */
        uint8_t getAnalogSensorMode();

        /**
         * @brief Get the analog inverted state
         * @return Analog inverted state as bool
         */
        bool getAnalogInverted();

        /**
         * @brief Get the analog sample delta
         * @return Analog sample delta as uint16_t
         */
        uint16_t getAnalogSampleDelta();

        // Alternate Encoder //

        /**
         * @brief Get the data port configuration
         * @return Data port config as uint8_t
         */
        uint8_t getDataPortConfig();

        /**
         * @brief Get the alternate encoder counts per revolution
         * @return Alternate encoder counts per revolution as uint16_t
         */
        uint16_t getAltEncoderCountsPerRev();

        /**
         * @brief Get the alternate encoder average depth
         * @return Alternate encoder average depth as uint8_t
         */
        uint8_t getAltEncoderAverageDepth();

        /**
         * @brief Get the alternate encoder sample delta
         * @return Alternate encoder sample delta as uint8_t
         */
        uint8_t getAltEncoderSampleDelta();

        /**
         * @brief Get the alternate encoder inverted state
         * @return Alternate encoder inverted state as bool
         */
        bool getAltEncoderInverted();

        /**
         * @brief Get the alternate encoder position factor
         * @return Alternate encoder position factor as float
         */
        float getAltEncoderPositionFactor();

        /**
         * @brief Get the alternate encoder velocity factor
         * @return Alternate encoder velocity factor as float
         */
        float getAltEncoderVelocityFactor();

        // Duty Cycle Absolute Encoder //

        /**
         * @brief Get the duty cycle position factor
         * @return Duty cycle position factor as float
         */
        float getDutyCyclePositionFactor();

        /**
         * @brief Get the duty cycle velocity factor
         * @return Duty cycle velocity factor as float
         */
        float getDutyCycleVelocityFactor();

        /**
         * @brief Get the duty cycle inverted state
         * @return Duty cycle inverted state as bool
         */
        bool getDutyCycleInverted();

        /**
         * @brief Get the duty cycle average depth
         * @return Duty cycle average depth as uint8_t
         */
        uint8_t getDutyCycleAverageDepth();

        /**
         * @brief Get the duty cycle prescalar
         * @return Duty cycle prescalar as uint8_t
         */
        uint8_t getDutyCyclePrescalar();

        /**
         * @brief Get the duty cycle zero offset
         * @return Duty cycle zero offset as float
         */
        float getDutyCycleZeroOffset();
    };
}
