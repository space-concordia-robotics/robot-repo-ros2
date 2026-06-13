//
// Created by nik on 08/02/24.
//

#pragma once

#include <cstdint>
#include <cstdio>
#include <linux/can.h>

constexpr auto STATUS_BUFFER_SIZE = 1000;

enum status {
    SUCCESS = 0,
    CAN_ERROR
};

class CANController {
    inline static int s_Socket = 0;

    inline static char* s_StatusBuffer = nullptr;

public:
    static uint8_t configureCAN(const char* fd_path);

    static uint8_t sendFrame(const can_frame& frame);

    static uint8_t readFrame(can_frame& frame);

    static uint8_t sendBlockingFrame(const can_frame& frame);

    static void printStatus() {
        printf("%s \n", s_StatusBuffer);
    }
};
