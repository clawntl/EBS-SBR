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
};

// Called by the CAN receive task for every status/reply/error frame.
void handleDriverStatusFrame(MotorSide side, uint8_t state, uint8_t flags);
void handleDriverStartReply(MotorSide side, bool ok);
void handleDriverError(MotorSide side, uint8_t code);

// Snapshot of one driver's status, for the display/serial output.
MotorDriverInfo getMotorDriverInfo(MotorSide side);

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
