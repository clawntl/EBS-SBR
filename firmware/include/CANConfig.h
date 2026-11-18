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

// Acceptance filters (the S3's one hardware filter split in two):
//  - driver telemetry 0x100-0x10F (0x100 itself is where a firmware 1.0/1.1
//    "right" board still reports, so it can be spotted)
//  - CAN module drive requests 0x200-0x20F
static const uint32_t CAN_RX_FILTER_ID = 0x100;
static const uint32_t CAN_RX_FILTER_MASK = 0x7F0;
static const uint32_t CAN_RX_FILTER2_ID = 0x200;
static const uint32_t CAN_RX_FILTER2_MASK = 0x7F0;

// ----------------------------------------------
// CAN modules (see docs/can-modules.md)
// ----------------------------------------------
// Module m (0-15) -> robot, drive request, at least every 250 ms while driving:
//   [int16 speed, 0.01 rad/s of wheel speed, + = forward]
//   [int16 turn, 0.1 deg/s, + = turning right (clockwise seen from above)]
//   [flags: MODULE_FLAG_*][sequence counter, optional]
//   Big-endian. With MODULE_FLAG_FRACTION the two values are instead
//   fractions of the robot's top speed / turn rate, x 1/32767.
#define MODULE_DRIVE_ID_BASE  0x200
#define MODULE_FLAG_DRIVE     0x01   // asking to drive (0 = present, not driving)
#define MODULE_FLAG_FRACTION  0x02   // values are fractions, not units
// Robot -> modules: state, every 20 ms:
//   [BalanceState][drive source: DriveInput::Source | module id << 4]
//   [int16 tilt from set point, 0.01 deg][int16 wheel speed, 0.01 rad/s, + = forward]
//   [int16 turn rate, 0.1 deg/s, + = right]
#define ROBOT_STATE_ID        0x180
// Robot -> modules: limits and battery, every 500 ms:
//   [uint16 battery mV][battery %, 255 = unknown][ROBOT_FLAG_*]
//   [uint16 top speed, 0.01 rad/s][uint16 turn rate, 0.1 deg/s]
#define ROBOT_INFO_ID         0x181
#define ROBOT_FLAG_PAUSED          0x01
#define ROBOT_FLAG_BATTERY_LOW     0x02
#define ROBOT_FLAG_MODULES_ALLOWED 0x04

// ----------------------------------------------
// Node IDs (driver CAN protocol 3, firmware 1.2+)
// ----------------------------------------------
// Each driver board has a node ID n = 1..15 (see BLDC-Driver
// docs/can-protocol.md). Which nodes are this robot's left and right wheels
// is set by the leftNode / rightNode settings (defaults 1 and 2).
//   current frame  0x010 + (n-1)/2, 8 bytes = two floats: odd n bytes 0-3,
//                  even n bytes 4-7; NaN = no command for that node
//   commands       0x020 + n
//   telemetry      0x100 + n
#define CURRENT_FRAME_BASE  0x010
#define COMMAND_ID_BASE     0x020
#define TELEMETRY_ID_BASE   0x100
#define NODE_ID_MIN         1
#define NODE_ID_MAX         15

// ----------------------------------------------
// Standard message command byte
// Must match include/can_protocol.h in the bldc_driver firmware.
// ----------------------------------------------
enum CAN_CMD : uint8_t {
    // Driver -> host: [0x02][float rad/s, big-endian]
    CMD_SHAFT_VELOCITY = 0x02,

    // Driver -> host heartbeat, every 100 ms (not sent while a driver is
    // blocked in its FOC start):
    // [0x03][DriverState][DRIVER_FLAG_* bits][supply mV, uint16 big-endian]
    // Supply mV = 0 means none (older driver firmware sends 0 there).
    CMD_DRIVER_STATUS = 0x03,

    // Host -> driver: [0x10]. Driver -> host reply once the start has
    // finished: [0x10][float 1.0 = ok, 0.0 = failed]
    CMD_START_FOC = 0x10,

    // ---- Configuration (driver CAN protocol 2, firmware 1.1+) -------------
    // Host -> driver on its command ID; the reply comes back on its telemetry
    // ID with the same command byte. See BLDC-Driver docs/can-protocol.md.
    CMD_GET_INFO    = 0x20, // -> [0x20][protocol][major][minor][patch][image KB][0][0]
    CMD_GET_CONFIG  = 0x21, // [0x21][0 active / 1 pending] -> [0x21][which][side][CFG_FLAG_*][zero angle f32]
    CMD_SET_PARAM   = 0x22, // [0x22][param][f32] -> [0x22][param][status][f32 now]
    CMD_CALIBRATION = 0x23, // [0x23][CAL_*] -> [0x23][op][status]
    CMD_SAVE        = 0x24, // -> [0x24][status], then save + reboot
    CMD_REVERT      = 0x25, // -> [0x25][status]
    CMD_REBOOT      = 0x26, // -> [0x26][status], then reboot
    // Board description (protocol 4, firmware 1.3+):
    // [0x27][page] -> page 0:   [0x27][0][capabilities u32][hw rev][0]
    //                 page 1-5: [0x27][page][6 chars of the model name, NUL-padded]
    // The model name (e.g. "BLDC-Driver v0.2") is up to 30 characters.
    CMD_GET_BOARD_INFO = 0x27,
    CMD_NACK        = 0xFE, // driver -> host: [0xFE][command][1] not understood

    // Driver -> host: [0xFF][float error code (DriverError)]
    CMD_ERROR = 0xFF,
};

enum DriverParam : uint8_t {
    PARAM_NODE_ID = 1,   // saved: 1-15 (protocol 2: 1 = left, 2 = right)
    PARAM_INVERT = 2,    // saved: 0 / 1
    PARAM_VELOCITY_INTERVAL_MS = 0x10, // runtime: velocity telemetry period, 1-100 ms
};

enum DriverCalOp : uint8_t { CAL_STORE = 1, CAL_CLEAR = 2 };

enum DriverConfigStatus : uint8_t {
    CONFIG_OK              = 0,
    CONFIG_ERR_UNKNOWN     = 1,
    CONFIG_ERR_BAD_VALUE   = 2,
    CONFIG_ERR_NOT_ALIGNED = 3,
    CONFIG_ERR_FLASH       = 4,
};

enum DriverConfigFlag : uint8_t {
    CFG_FLAG_INVERT         = 1 << 0,
    CFG_FLAG_CALIBRATION    = 1 << 1,
    CFG_FLAG_SENSOR_DIR_CCW = 1 << 2,
    CFG_FLAG_UNSAVED        = 1 << 3,
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
