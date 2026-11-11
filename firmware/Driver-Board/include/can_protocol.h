#pragma once

#include <cstdint>

// IS_LEFT_MOTOR is set via the PlatformIO environment's build_flags
// (see platformio.ini: `left` vs `right` envs), not defined here.

#define BOTH_MOTOR_CAN_ADDR     0x020

#ifdef IS_LEFT_MOTOR
  #define CAN_SEND_TO           0x101
  #define THIS_MOTOR_CAN_ADDR   0x021
#else
  #define CAN_SEND_TO           0x100
  #define THIS_MOTOR_CAN_ADDR   0x022
#endif

#define CAN_BIT_RATE     1'000'000 // 1 Mbit/s classic CAN 2.0B (no FD / bit rate switch used)
#define CAN_SAMPLE_POINT 88        // % ; resolves to an exact 87.5% sample point at this bit rate/clock

enum CAN_CMD : uint8_t {
    CMD_SHAFT_VELOCITY = 0x02,

    CMD_START_FOC = 0x10,

    CMD_ERROR = 0xFF,
};
