#pragma once

static constexpr auto COMMAND_PREFIX_SET_DEVICE_ID = 0x205C0;

static constexpr auto COMMAND_PREFIX_RAMP = 0x205CE;

static constexpr auto COMMAND_PREFIX_COAST = 0x205C1;
static constexpr auto COMMAND_PREFIX_BRAKE = 0x205C1;

static constexpr auto COMMAND_PREFIX_HARD_FORWARD_LIMIT = 0x205CD0;
static constexpr auto COMMAND_PREFIX_HARD_REVERSE_LIMIT = 0x205CD4;

static constexpr auto COMMAND_PREFIX_MOVE_MOTORS = 0x20500;

static constexpr auto COMMAND_PREFIX_MAINTAIN_VELOCITY = 0x82052c80;
static constexpr auto COMMAND_PREFIX_MAINTAIN_SPEED = 0x02052C80;
static constexpr auto COMMAND_PREFIX_READ_STATUS = 0x502C0;

static constexpr auto COMMAND_PREFIX_PERIODIC_FRAME = 0x2051;
static constexpr auto COMMAND_PREFIX_VELOCITY_CONTROL = 0x820504;

static constexpr auto STATUS_FRAME_ID_OFFSET = 0x40;
static constexpr auto DEVICE_MAX_ID = 0x3E;
