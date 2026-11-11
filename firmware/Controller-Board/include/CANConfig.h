#pragma once

#include <cstdint>

// ----------------------------------------------
// Bus timing
// ----------------------------------------------
static const uint32_t CAN_BIT_RATE = 1000UL * 1000UL;      // 1 Mbit/s
static const uint32_t CAN_SAMPLE_POINT_PERMILL = 875;       // 87.5 %

// ACAN_ESP32FD_Settings' auto solver picks 20 time quanta (BRP=4) for an
// exact 1 Mbit/s on the S3's 80 MHz TWAI clock, but PHASE_SEG1 is
// hardware-capped at 16, capping the achievable sample point at 85.0%
// (17/20). These explicit segment values (8 total quanta, BRP=10) still
// give an exact 1 Mbit/s while hitting 87.5% (7/8) exactly.
static const uint32_t CAN_ARBITRATION_BRP = 10;
static const uint32_t CAN_ARBITRATION_PHASE_SEGMENT_1 = 6;
static const uint32_t CAN_ARBITRATION_PHASE_SEGMENT_2 = 1;
static const uint32_t CAN_ARBITRATION_SJW = 1;

// Acceptance filter: accepts 0x100, 0x101, 0x102, 0x103
static const uint32_t CAN_RX_FILTER_ID = 0x100;
static const uint32_t CAN_RX_FILTER_MASK = 0x7FC;

// ----------------------------------------------
// Node addresses
// ----------------------------------------------
#define BOTH_MOTOR_CAN_ADDR     0x020
#define LEFT_MOTOR_CAN_ADDR     0x021
#define RIGHT_MOTOR_CAN_ADDR    0x022

// Shaft velocity broadcast IDs (sent by the motor controllers)
#define RIGHT_MOTOR_VELOCITY_CAN_ID   0x100
#define LEFT_MOTOR_VELOCITY_CAN_ID    0x101

// ----------------------------------------------
// Standard message command byte
// ----------------------------------------------
enum CAN_CMD : uint8_t {
    CMD_SHAFT_VELOCITY = 0x02,

    CMD_START_FOC = 0x10,

    CMD_ERROR = 0xFF,
};
