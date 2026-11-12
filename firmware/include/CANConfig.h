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
// Must match include/can_protocol.h in the bldc_driver firmware.
// ----------------------------------------------
enum CAN_CMD : uint8_t {
    // Driver -> host: [0x02][float rad/s, big-endian]
    CMD_SHAFT_VELOCITY = 0x02,

    // Driver -> host heartbeat, every 100 ms (not sent while a driver is
    // blocked in its FOC start): [0x03][DriverState][DRIVER_FLAG_* bits][0][0]
    CMD_DRIVER_STATUS = 0x03,

    // Host -> driver: [0x10]. Driver -> host reply once the start has
    // finished: [0x10][float 1.0 = ok, 0.0 = failed]
    CMD_START_FOC = 0x10,

    // Driver -> host: [0xFF][float error code (DriverError)]
    CMD_ERROR = 0xFF,
};

enum DriverState : uint8_t {
    DRIVER_STATE_UNCONFIGURED = 0, // never actually sent (driver CAN is off until configured)
    DRIVER_STATE_IDLE         = 1, // waiting for CMD_START_FOC
    DRIVER_STATE_RUNNING      = 2, // FOC running, accepting current commands
    DRIVER_STATE_FAULT        = 3, // FOC start failed; CMD_START_FOC retries
};

enum DriverFlag : uint8_t {
    DRIVER_FLAG_COMMAND_TIMEOUT  = 1 << 0, // driver got no current command for 100 ms -> applying 0 A
    DRIVER_FLAG_CALIBRATION_USED = 1 << 1, // stored alignment used, startup alignment skipped
    DRIVER_FLAG_INVERTED         = 1 << 2, // driver's direction invert setting is on
    DRIVER_FLAG_DRV_FAULT        = 1 << 3, // DRV8316 gate driver reported a fault
};

enum DriverError : uint8_t {
    DRIVER_ERR_MOTOR_INIT    = 1,
    DRIVER_ERR_CURRENT_SENSE = 2,
    DRIVER_ERR_FOC_INIT      = 3,
    DRIVER_ERR_DRV_FAULT     = 4,
};
