//
// Created by nik on 06/02/24.
//
#include "rev_motor_controller.h"

#include <cstring>
#include <linux/can/raw.h>

#include "command_prefixes.h"

void RevMotorController::setDeviceId(const uint8_t id, const uint8_t newId) {
    can_frame frame{};

    frame.len = 5; // NOLINT(*-pro-type-union-access)
    frame.can_id = (COMMAND_PREFIX_SET_DEVCE_ID << 8) | id;

    const uint8_t buf[5] = {newId, 0, 0, 0, 1}; // NOLINT(*-avoid-c-arrays)
    memcpy(frame.data, buf, sizeof(buf)); // NOLINT(*-pro-bounds-array-to-pointer-decay)

    CANController::sendFrame(frame);
}

void RevMotorController::requestStatusFrame() {
    can_frame frame{};
    /*
     * Periodic status messages will be sent over the CAN bus after sending this command once.
     */

    frame.can_id = 0x000502C0;
    frame.len = 1; // NOLINT(*-pro-type-union-access)
    frame.can_id |= CAN_EFF_FLAG;
    frame.data[0] = 1;

    CANController::sendFrame(frame);
}

void RevMotorController::voltagePercentControl(const uint8_t deviceId, const float percent) {
    can_frame frame{};
    /*
     *
     */
    if (s_Devices[deviceId].isRunning) { // NOLINT(*-pro-bounds-constant-array-index)
        const uint64_t buf_data = 1uLL << deviceId;

        frame.can_id = COMMAND_PREFIX_MAINTAIN_SPEED;
        frame.can_id |= CAN_EFF_FLAG;
        frame.len = 8; // NOLINT(*-pro-type-union-access)
        memcpy(frame.data, &buf_data, sizeof(buf_data)); // NOLINT(*-pro-bounds-array-to-pointer-decay)
    } else {
        s_Devices[deviceId].isRunning = true; // NOLINT(*-pro-bounds-constant-array-index)

        // For a start move command, the id field needs to be added with 0x80. This command is only issued once per move.
        frame.can_id = (COMMAND_PREFIX_MOVE_MOTORS << 8) | (deviceId + 0x80);
        frame.can_id |= CAN_EFF_FLAG;
        frame.len = 8; // NOLINT(*-pro-type-union-access)
        memcpy(frame.data, &percent, sizeof(float)); // NOLINT(*-pro-bounds-array-to-pointer-decay)
    }
    CANController::sendFrame(frame);
}

void RevMotorController::stopMotor(const uint8_t device_id) {
    can_frame frame{};
    frame.can_id = (COMMAND_PREFIX_MOVE_MOTORS << 8) | (device_id); // NOLINT(*-redundant-parentheses)
    frame.len = 8; // NOLINT(*-pro-type-union-access)

    CANController::sendFrame(frame);
}

void RevMotorController::velocityControl(const uint8_t deviceID, const float velocity) {
    // if(abs(velocity) < 20){
    //     return;
    // }

    can_frame frame{};
    frame.can_id = (COMMAND_PREFIX_VELOCITY_CONTROL << 8) | (deviceID + 0x80) | CAN_EFF_FLAG;
    frame.len = 8;

    // frame.can_id = (COMMAND_PREFIX_VELOCITY_CONTROL << 8) | (deviceId+0x80);
    // frame.can_id |= CAN_EFF_FLAG;
    // frame.len = 8;
    // frame.data = velocity
    memcpy(frame.data, &velocity, sizeof(float)); // NOLINT(*-pro-bounds-array-to-pointer-decay)

    CANController::sendBlockingFrame(frame);

    // std::cout << "Running  " << (int)deviceId << " on " << velocity << "\n";
    // uint64_t buf_data = (1ULL << deviceId);
    // frame.can_id = COMMAND_PREFIX_MAINTAIN_VELOCITY;
    // frame.can_id |= CAN_EFF_FLAG;
    // frame.len = 8;
    // memcpy(frame.data,&buf_data,sizeof(buf_data));

    // CANController::sendBlockingFrame(frame);

    // std::cout << "Starting run on " << (int)deviceId << "\n";
    // s_Devices.at(deviceId)->isRunning = true;
    // s_Devices[deviceId].isRunning = true;
}

void RevMotorController::startMotor(const uint64_t mask) {
    can_frame frame{};

    // uint64_t buf_data = (1ULL << deviceId);
    frame.can_id = COMMAND_PREFIX_MAINTAIN_VELOCITY;
    frame.can_id |= CAN_EFF_FLAG;
    frame.len = 8; // NOLINT(*-pro-type-union-access)
    memcpy(frame.data, &mask, sizeof(mask)); // NOLINT(*-pro-bounds-array-to-pointer-decay)

    CANController::sendBlockingFrame(frame);
}

void RevMotorController::registerDevice(const uint8_t deviceId) {
    // s_Devices.emplace(deviceId, new Device());
    s_Devices[deviceId].id = deviceId; // NOLINT(*-pro-bounds-constant-array-index)
}

//void RevMotorController::printStatus() {
//
//    snprintf (s_StatusBuffer, STATUS_BUFFER_SIZE - strlen(s_StatusBuffer),
//              "Motor %d status : \n RPM : %f \n Temperature : %i \n Position : %f \n\n",
//             m_deviceID,m_Status.rpm,m_Status.temperature,m_Position);
//}
