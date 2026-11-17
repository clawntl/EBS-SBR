#pragma once

#include <stdint.h>

// Tracks the two BLDC motor drivers from their CAN status frames and starts
// FOC on them automatically:
//  - a driver that reports IDLE (or FAULT) gets CMD_START_FOC, one driver at a
//    time, so a driver that resets later (watchdog, brown-out) is restarted
//    instead of sitting idle;
//  - a FAULT start is retried up to MOTOR_MAX_START_ATTEMPTS times, then left
//    alone (it would jolt the wheel on every attempt) until
//    resetMotorStartAttempts() or a reboot.

enum MotorSide : uint8_t { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 };

enum class MotorLink : uint8_t {
    Offline,   // no heartbeat (not powered, not configured, CAN fault...)
    Starting,  // CMD_START_FOC sent, waiting for the reply
    Idle,
    Running,
    Fault,     // FOC start failed
    GaveUp,    // FOC start failed MOTOR_MAX_START_ATTEMPTS times
};

struct MotorDriverInfo {
    MotorLink link;
    uint8_t   flags;         // DRIVER_FLAG_* from the last heartbeat
    uint8_t   lastError;     // DriverError code, 0 = none
    uint32_t  lastErrorAgeMs;
    uint8_t   startAttempts;
    float     supplyVolts;   // from the heartbeat, 0 = not reported
};

// ---- Driver configuration over CAN (driver firmware 1.1+, protocol 2) ----
//
// The web page reads each driver's firmware version and settings and changes
// them through requestDriverOp(). Operations run one at a time in
// Task_motorSupervisor (which also refreshes a driver's details whenever it
// (re)appears on the bus). Anything that changes a setting or reboots a driver
// is refused while the robot is balancing: the driver stops its motor.

struct DriverConfigView {
    bool    infoKnown;      // CMD_GET_INFO answered
    bool    noConfigSupport; // asked, no answer: firmware 1.0 (protocol 1) or offline
    uint8_t protocol;
    uint8_t fwMajor, fwMinor, fwPatch;
    uint8_t imageKB;
    bool    activeKnown;    // settings in use
    uint8_t side;           // 1 = left, 2 = right
    uint8_t flags;          // CFG_FLAG_*
    float   zeroAngle;      // rad, if CFG_FLAG_CALIBRATION
    bool    pendingKnown;   // settings after the next save
    uint8_t pendSide;
    uint8_t pendFlags;
    char    result[72];     // outcome of the last operation, "" = none
};

enum class DriverOp : uint8_t {
    Refresh,      // re-read firmware version and settings
    SetInvert,    // value 0/1, pending until Save
    CalStore,     // keep this boot's alignment, pending until Save
    CalClear,     // forget the stored alignment, pending until Save
    Save,         // write pending settings and reboot the driver
    Revert,       // discard unsaved changes
    Reboot,
    Recalibrate,  // clear + save, let it align on the next start, store + save
};

DriverConfigView getDriverConfig(MotorSide side);
// Incremented whenever a driver's details or an operation's state change.
uint32_t driverConfigVersion();
// Description of the operation in progress ("" = none).
const char *driverJobText();

// Starts an operation. Returns nullptr if started, otherwise why not.
const char *requestDriverOp(MotorSide side, DriverOp op, float value = 0);
// Swaps the two boards' sides (both set + saved together, so their CAN IDs
// swap; doing one at a time would leave both on the same IDs).
const char *requestSwapDriverSides();

// Velocity telemetry period [ms] asked of the drivers (runtime setting on the
// driver, re-sent whenever a driver starts running). 0 = leave at its default.
extern uint8_t wheelTelemetryMs;

// Called by the CAN receive task for every status/reply/error frame.
// supplyMv: the driver's supply voltage from heartbeat bytes 3-4 (0 = none).
void handleDriverStatusFrame(MotorSide side, uint8_t state, uint8_t flags, uint16_t supplyMv);
void handleDriverStartReply(MotorSide side, bool ok);
void handleDriverError(MotorSide side, uint8_t code);
// Configuration replies (CMD_GET_INFO ... CMD_REBOOT, CMD_NACK).
void handleDriverConfigReply(MotorSide side, const uint8_t *data, uint8_t len);

// Snapshot of one driver's status, for the display/serial output.
MotorDriverInfo getMotorDriverInfo(MotorSide side);

// A driver's latest supply voltage [V], or 0 if it hasn't reported one in the
// last maxAgeMs (offline, or firmware without the voltage field).
float getDriverSupplyVoltage(MotorSide side, uint32_t maxAgeMs);

// True when both drivers are running FOC with no gate-driver fault, i.e. the
// motors will actually act on the current commands.
bool motorsReady();

// Blocks until both drivers report running, or the timeout expires.
// Returns motorsReady().
bool waitForMotorsReady(uint32_t timeoutMs);

// Allows a driver that has given up (GaveUp) to be started again.
void resetMotorStartAttempts();

const char *motorLinkName(MotorLink link);
const char *driverErrorName(uint8_t code);

// Supervisor task: issues CMD_START_FOC as described above.
void Task_motorSupervisor(void *pvParams);
