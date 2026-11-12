#include "MotorDrivers.h"

#include <Arduino.h>

#include "CANConfig.h"
#include "CanBus.h"

namespace {

// A heartbeat arrives every 100 ms; allow a few to be missed.
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500;
// FOC start (alignment) blocks the driver for ~1-3 s; give up waiting after this.
constexpr uint32_t START_REPLY_TIMEOUT_MS = 8000;
// Minimum gap between start attempts on the same driver.
constexpr uint32_t START_RETRY_INTERVAL_MS = 5000;
constexpr uint8_t  MOTOR_MAX_START_ATTEMPTS = 3;

struct DriverTrack {
    volatile uint8_t  state = DRIVER_STATE_UNCONFIGURED;
    volatile uint8_t  flags = 0;
    volatile uint32_t lastHeartbeatMs = 0;
    volatile bool     everSeen = false;

    volatile bool     startPending = false;
    volatile uint32_t startSentMs = 0;
    volatile uint8_t  startAttempts = 0;
    volatile bool     lastStartFailed = false;

    volatile uint8_t  lastError = 0;
    volatile uint32_t lastErrorMs = 0;
};

DriverTrack drivers[2];

const uint32_t kCommandId[2] = { LEFT_MOTOR_CAN_ADDR, RIGHT_MOTOR_CAN_ADDR };
const char *const kSideName[2] = { "Left", "Right" };

bool heartbeatFresh(const DriverTrack &d, uint32_t now) {
    return d.everSeen && (now - d.lastHeartbeatMs) < HEARTBEAT_TIMEOUT_MS;
}

MotorLink linkOf(const DriverTrack &d, uint32_t now) {
    if (d.startPending) return MotorLink::Starting;
    if (!heartbeatFresh(d, now)) return MotorLink::Offline;
    switch (d.state) {
        case DRIVER_STATE_RUNNING: return MotorLink::Running;
        case DRIVER_STATE_FAULT:
            return (d.startAttempts >= MOTOR_MAX_START_ATTEMPTS) ? MotorLink::GaveUp : MotorLink::Fault;
        case DRIVER_STATE_IDLE:
            // Idle after failed attempts (e.g. a reset in between) still counts as given up.
            return (d.startAttempts >= MOTOR_MAX_START_ATTEMPTS && d.lastStartFailed) ? MotorLink::GaveUp : MotorLink::Idle;
        default: return MotorLink::Offline;
    }
}

} // namespace

void handleDriverStatusFrame(MotorSide side, uint8_t state, uint8_t flags) {
    DriverTrack &d = drivers[side];
    d.state = state;
    d.flags = flags;
    d.lastHeartbeatMs = millis();
    d.everSeen = true;
    if (state == DRIVER_STATE_RUNNING) {
        // Covers a missed start reply.
        d.startPending = false;
        d.startAttempts = 0;
        d.lastStartFailed = false;
    }
}

void handleDriverStartReply(MotorSide side, bool ok) {
    DriverTrack &d = drivers[side];
    d.startPending = false;
    d.lastStartFailed = !ok;
    if (ok) d.startAttempts = 0;
    Serial.printf("%s motor driver: FOC start %s\n", kSideName[side], ok ? "OK" : "FAILED");
}

void handleDriverError(MotorSide side, uint8_t code) {
    DriverTrack &d = drivers[side];
    d.lastError = code;
    d.lastErrorMs = millis();
    Serial.printf("%s motor driver error %u: %s\n", kSideName[side], code, driverErrorName(code));
}

MotorDriverInfo getMotorDriverInfo(MotorSide side) {
    const DriverTrack &d = drivers[side];
    const uint32_t now = millis();
    MotorDriverInfo info;
    info.link = linkOf(d, now);
    info.flags = d.flags;
    info.lastError = d.lastError;
    info.lastErrorAgeMs = d.lastError ? (now - d.lastErrorMs) : 0;
    info.startAttempts = d.startAttempts;
    return info;
}

bool motorsReady() {
    const uint32_t now = millis();
    for (const DriverTrack &d : drivers) {
        if (linkOf(d, now) != MotorLink::Running) return false;
        if (d.flags & DRIVER_FLAG_DRV_FAULT) return false;
    }
    return true;
}

bool waitForMotorsReady(uint32_t timeoutMs) {
    const uint32_t start = millis();
    while (!motorsReady() && (millis() - start) < timeoutMs) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return motorsReady();
}

void resetMotorStartAttempts() {
    for (DriverTrack &d : drivers) {
        d.startAttempts = 0;
        d.lastStartFailed = false;
    }
}

const char *motorLinkName(MotorLink link) {
    switch (link) {
        case MotorLink::Offline:  return "OFFLINE";
        case MotorLink::Starting: return "START..";
        case MotorLink::Idle:     return "IDLE";
        case MotorLink::Running:  return "RUN";
        case MotorLink::Fault:    return "FAULT";
        case MotorLink::GaveUp:   return "FAILED";
    }
    return "?";
}

const char *driverErrorName(uint8_t code) {
    switch (code) {
        case DRIVER_ERR_MOTOR_INIT:    return "motor init";
        case DRIVER_ERR_CURRENT_SENSE: return "current sense";
        case DRIVER_ERR_FOC_INIT:      return "FOC align";
        case DRIVER_ERR_DRV_FAULT:     return "DRV8316 fault";
        default:                       return "unknown";
    }
}

void Task_motorSupervisor(void *pvParams) {
    uint32_t lastStartAttemptMs[2] = { 0, 0 };
    bool startedOnce[2] = { false, false };

    while (1) {
        const uint32_t now = millis();

        // Expire a start that never got a reply (e.g. the driver reset mid-start).
        for (int s = 0; s < 2; s++) {
            DriverTrack &d = drivers[s];
            if (d.startPending && (now - d.startSentMs) > START_REPLY_TIMEOUT_MS) {
                d.startPending = false;
                d.lastStartFailed = true;
                Serial.printf("%s motor driver: no reply to START_FOC\n", kSideName[s]);
            }
        }

        // Only one driver starting at a time: alignment draws current and
        // moves the wheel, so don't do both at once.
        const bool anyPending = drivers[0].startPending || drivers[1].startPending;
        if (!anyPending) {
            for (int s = 0; s < 2; s++) {
                DriverTrack &d = drivers[s];
                const MotorLink link = linkOf(d, now);
                if (link != MotorLink::Idle && link != MotorLink::Fault) continue;
                if (startedOnce[s] && (now - lastStartAttemptMs[s]) < START_RETRY_INTERVAL_MS) continue;

                d.startAttempts = d.startAttempts + 1;
                d.startPending = true;
                d.startSentMs = now;
                lastStartAttemptMs[s] = now;
                startedOnce[s] = true;
                Serial.printf("%s motor driver: sending START_FOC (attempt %u)\n", kSideName[s], d.startAttempts);
                sendStartFOC(kCommandId[s]);
                break; // one at a time
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
