#include "MotorDrivers.h"

#include <Arduino.h>

#include "CANConfig.h"
#include "CanBus.h"
#include "DeviceLog.h"
#include "LQRControl.h"

#include <atomic>
#include <initializer_list>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

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

    volatile uint16_t supplyMv = 0;
    volatile uint32_t supplyMs = 0;

    MotorLink loggedLink = MotorLink::Offline; // for logging link changes
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

void handleDriverStatusFrame(MotorSide side, uint8_t state, uint8_t flags, uint16_t supplyMv) {
    DriverTrack &d = drivers[side];
    const uint32_t now = millis();
    d.state = state;
    d.flags = flags;
    d.lastHeartbeatMs = now;
    if (supplyMv) {
        d.supplyMv = supplyMv;
        d.supplyMs = now;
    }
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
    DeviceLog::printf("[motor] %s driver: FOC start %s\n", kSideName[side], ok ? "OK" : "FAILED");
}

void handleDriverError(MotorSide side, uint8_t code) {
    DriverTrack &d = drivers[side];
    d.lastError = code;
    d.lastErrorMs = millis();
    DeviceLog::printf("[motor] %s driver error %u: %s\n", kSideName[side], code, driverErrorName(code));
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
    info.supplyVolts = getDriverSupplyVoltage(side, 1000);
    return info;
}

float getDriverSupplyVoltage(MotorSide side, uint32_t maxAgeMs) {
    const DriverTrack &d = drivers[side];
    const uint32_t mv = d.supplyMv, at = d.supplyMs;
    if (!mv || millis() - at > maxAgeMs) return 0.0f;
    return mv / 1000.0f;
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

// ===========================================================================
// Driver configuration over CAN
// ===========================================================================

uint8_t wheelTelemetryMs = 2;

namespace {

constexpr uint32_t REPLY_TIMEOUT_MS = 500;
constexpr uint32_t REBOOT_TIMEOUT_MS = 20000; // driver reboot + FOC start (+ alignment)

portMUX_TYPE cfgLock = portMUX_INITIALIZER_UNLOCKED;
DriverConfigView cfg[2] = {};
std::atomic<uint32_t> cfgVersion{0};

// Last configuration reply per driver, for the job steps to wait on.
struct LastReply {
    volatile uint32_t seq = 0;
    volatile uint8_t cmd = 0;
    volatile uint8_t arg = 0;     // param / op / which
    volatile uint8_t status = 0;
};
LastReply lastReply[2];

bool needsRefresh[2] = { false, false };
uint8_t sentInterval[2] = { 0, 0 };

enum class Job : uint8_t { None, Refresh, Simple, Save, Recalibrate, Swap };
enum class Step : uint8_t { Send, WaitReply, WaitInfo,
                            RecalWaitClear, RecalWaitSave1, RecalWaitAligned, RecalWaitStore,
                            RecalWaitSave2, RecalWaitRunning,
                            SwapWaitSet, SwapWaitSave };

struct JobState {
    Job job = Job::None;
    MotorSide side = MOTOR_LEFT;
    DriverOp op = DriverOp::Refresh;
    float value = 0;
    Step step = Step::Send;
    uint32_t stepStartMs = 0;
    uint32_t seqAtSend[2] = { 0, 0 };
    bool sawRestart = false;
    char text[64] = "";
};
JobState job;

void bump() { cfgVersion++; }

void setResult(MotorSide side, const char *fmt, ...) {
    char buf[sizeof(cfg[0].result)];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    portENTER_CRITICAL(&cfgLock);
    strncpy(cfg[side].result, buf, sizeof(cfg[side].result) - 1);
    cfg[side].result[sizeof(cfg[side].result) - 1] = 0;
    portEXIT_CRITICAL(&cfgLock);
    DeviceLog::printf("[driver] %s: %s\n", kSideName[side], buf);
    bump();
}

void setJobText(const char *t) {
    portENTER_CRITICAL(&cfgLock);
    strncpy(job.text, t, sizeof(job.text) - 1);
    job.text[sizeof(job.text) - 1] = 0;
    portEXIT_CRITICAL(&cfgLock);
    bump();
}

void endJob() {
    job.job = Job::None;
    setJobText("");
}

void send(MotorSide side, std::initializer_list<uint8_t> bytes) {
    uint8_t b[8];
    uint8_t n = 0;
    for (uint8_t x : bytes) if (n < 8) b[n++] = x;
    sendDriverCommand(kCommandId[side], b, n);
}

void sendSetParam(MotorSide side, uint8_t param, float v) {
    uint8_t b[6] = { CMD_SET_PARAM, param };
    uint32_t bits;
    memcpy(&bits, &v, 4);
    b[2] = bits >> 24; b[3] = bits >> 16; b[4] = bits >> 8; b[5] = bits;
    sendDriverCommand(kCommandId[side], b, sizeof(b));
}

void sendQueries(MotorSide side) {
    send(side, { CMD_GET_INFO });
    send(side, { CMD_GET_CONFIG, 0 });
    send(side, { CMD_GET_CONFIG, 1 });
}

const char *statusText(uint8_t st) {
    switch (st) {
        case CONFIG_OK:              return "OK";
        case CONFIG_ERR_UNKNOWN:     return "not supported by this firmware";
        case CONFIG_ERR_BAD_VALUE:   return "value out of range";
        case CONFIG_ERR_NOT_ALIGNED: return "no alignment to store (motor not running)";
        case CONFIG_ERR_FLASH:       return "firmware too big - settings page in use";
    }
    return "unknown status";
}

// True once a reply to `cmd` arrived after seqAtSend; status in *st.
bool gotReply(MotorSide side, uint8_t cmd, uint8_t *st) {
    if (lastReply[side].seq == job.seqAtSend[side]) return false;
    if (lastReply[side].cmd != cmd && lastReply[side].cmd != CMD_NACK) return false;
    *st = lastReply[side].cmd == CMD_NACK ? CONFIG_ERR_UNKNOWN : lastReply[side].status;
    return true;
}

void markSent(MotorSide side, uint32_t now) {
    job.seqAtSend[side] = lastReply[side].seq;
    job.stepStartMs = now;
}

void forgetSettings(MotorSide side) {
    portENTER_CRITICAL(&cfgLock);
    cfg[side].activeKnown = false;
    cfg[side].pendingKnown = false;
    portEXIT_CRITICAL(&cfgLock);
    bump();
}

uint8_t simpleCmd(DriverOp op) {
    switch (op) {
        case DriverOp::SetInvert: return CMD_SET_PARAM;
        case DriverOp::CalStore:
        case DriverOp::CalClear:  return CMD_CALIBRATION;
        case DriverOp::Revert:    return CMD_REVERT;
        case DriverOp::Save:      return CMD_SAVE;
        case DriverOp::Reboot:    return CMD_REBOOT;
        default:                  return 0;
    }
}

void runJob(uint32_t now) {
    const MotorSide sd = job.side;
    const uint32_t inStep = now - job.stepStartMs;
    uint8_t st = 0;

    switch (job.job) {
    case Job::None:
        return;

    case Job::Refresh:
        if (job.step == Step::Send) {
            markSent(sd, now);
            sendQueries(sd);
            job.step = Step::WaitInfo;
        } else if (cfg[sd].infoKnown && lastReply[sd].seq != job.seqAtSend[sd] && inStep > 100) {
            endJob();
        } else if (inStep > REPLY_TIMEOUT_MS) {
            if (!cfg[sd].infoKnown) {
                portENTER_CRITICAL(&cfgLock);
                cfg[sd].noConfigSupport = true;
                portEXIT_CRITICAL(&cfgLock);
                bump();
            }
            endJob();
        }
        return;

    case Job::Simple:
    case Job::Save: {
        const uint8_t cmd = simpleCmd(job.op);
        if (job.step == Step::Send) {
            markSent(sd, now);
            switch (job.op) {
                case DriverOp::SetInvert: sendSetParam(sd, PARAM_INVERT, job.value); break;
                case DriverOp::CalStore:  send(sd, { CMD_CALIBRATION, CAL_STORE }); break;
                case DriverOp::CalClear:  send(sd, { CMD_CALIBRATION, CAL_CLEAR }); break;
                default:                  send(sd, { cmd }); break;
            }
            job.step = Step::WaitReply;
        } else if (gotReply(sd, cmd, &st)) {
            const bool restarts = (job.op == DriverOp::Save || job.op == DriverOp::Reboot) && st == CONFIG_OK;
            setResult(sd, "%s: %s%s", job.text, statusText(st), restarts ? " - rebooting" : "");
            if (restarts) forgetSettings(sd);
            else { send(sd, { CMD_GET_CONFIG, 0 }); send(sd, { CMD_GET_CONFIG, 1 }); }
            endJob();
        } else if (inStep > REPLY_TIMEOUT_MS) {
            setResult(sd, "%s: no reply", job.text);
            endJob();
        }
        return;
    }

    case Job::Recalibrate: {
        const MotorLink link = linkOf(drivers[sd], now);
        switch (job.step) {
        case Step::Send:
            markSent(sd, now);
            send(sd, { CMD_CALIBRATION, CAL_CLEAR });
            job.step = Step::RecalWaitClear;
            setJobText("Alignment: clearing the stored one");
            break;
        case Step::RecalWaitClear:
            if (gotReply(sd, CMD_CALIBRATION, &st)) {
                if (st != CONFIG_OK) { setResult(sd, "Alignment failed: clear: %s", statusText(st)); endJob(); break; }
                markSent(sd, now);
                send(sd, { CMD_SAVE });
                job.step = Step::RecalWaitSave1;
            } else if (inStep > REPLY_TIMEOUT_MS) { setResult(sd, "Alignment failed: no reply"); endJob(); }
            break;
        case Step::RecalWaitSave1:
            if (gotReply(sd, CMD_SAVE, &st)) {
                if (st != CONFIG_OK) { setResult(sd, "Alignment failed: save: %s", statusText(st)); endJob(); break; }
                forgetSettings(sd);
                job.sawRestart = false;
                job.stepStartMs = now;
                job.step = Step::RecalWaitAligned;
                setJobText("Alignment: driver rebooting, then aligning (the wheel will move)");
            } else if (inStep > REPLY_TIMEOUT_MS) { setResult(sd, "Alignment failed: no reply to save"); endJob(); }
            break;
        case Step::RecalWaitAligned:
            if (link != MotorLink::Running) job.sawRestart = true;
            if (job.sawRestart && link == MotorLink::Running) {
                if (drivers[sd].flags & DRIVER_FLAG_CALIBRATION_USED) {
                    setResult(sd, "Alignment failed: the driver still used a stored alignment");
                    endJob();
                    break;
                }
                markSent(sd, now);
                send(sd, { CMD_CALIBRATION, CAL_STORE });
                job.step = Step::RecalWaitStore;
                setJobText("Alignment: storing the new one");
            } else if (inStep > REBOOT_TIMEOUT_MS) {
                setResult(sd, "Alignment failed: driver didn't start again (state %s)", motorLinkName(link));
                endJob();
            }
            break;
        case Step::RecalWaitStore:
            if (gotReply(sd, CMD_CALIBRATION, &st)) {
                if (st != CONFIG_OK) { setResult(sd, "Alignment failed: store: %s", statusText(st)); endJob(); break; }
                markSent(sd, now);
                send(sd, { CMD_SAVE });
                job.step = Step::RecalWaitSave2;
            } else if (inStep > REPLY_TIMEOUT_MS) { setResult(sd, "Alignment failed: no reply to store"); endJob(); }
            break;
        case Step::RecalWaitSave2:
            if (gotReply(sd, CMD_SAVE, &st)) {
                if (st != CONFIG_OK) { setResult(sd, "Alignment failed: save: %s", statusText(st)); endJob(); break; }
                forgetSettings(sd);
                job.sawRestart = false;
                job.stepStartMs = now;
                job.step = Step::RecalWaitRunning;
                setJobText("Alignment: driver rebooting with the new alignment");
            } else if (inStep > REPLY_TIMEOUT_MS) { setResult(sd, "Alignment failed: no reply to save"); endJob(); }
            break;
        case Step::RecalWaitRunning:
            if (link != MotorLink::Running) job.sawRestart = true;
            if (job.sawRestart && link == MotorLink::Running) {
                const bool used = drivers[sd].flags & DRIVER_FLAG_CALIBRATION_USED;
                setResult(sd, used ? "Alignment done and saved" : "Alignment saved, but the driver didn't use it on restart");
                endJob();
            } else if (inStep > REBOOT_TIMEOUT_MS) {
                setResult(sd, "Alignment saved, but the driver didn't start again (state %s)", motorLinkName(link));
                endJob();
            }
            break;
        default:
            endJob();
        }
        return;
    }

    case Job::Swap:
        if (job.step == Step::Send) {
            markSent(MOTOR_LEFT, now);
            markSent(MOTOR_RIGHT, now);
            sendSetParam(MOTOR_LEFT, PARAM_SIDE, 2);  // left board becomes right
            sendSetParam(MOTOR_RIGHT, PARAM_SIDE, 1); // right board becomes left
            job.step = Step::SwapWaitSet;
        } else if (job.step == Step::SwapWaitSet) {
            uint8_t a = 0, b = 0;
            const bool ga = gotReply(MOTOR_LEFT, CMD_SET_PARAM, &a), gb = gotReply(MOTOR_RIGHT, CMD_SET_PARAM, &b);
            if (ga && gb) {
                if (a != CONFIG_OK || b != CONFIG_OK) {
                    // Undo whichever one took, so neither is left pending a swap.
                    send(MOTOR_LEFT, { CMD_REVERT });
                    send(MOTOR_RIGHT, { CMD_REVERT });
                    setResult(MOTOR_LEFT, "Swap failed (%s / %s) - nothing changed", statusText(a), statusText(b));
                    endJob();
                    break;
                }
                markSent(MOTOR_LEFT, now);
                markSent(MOTOR_RIGHT, now);
                send(MOTOR_LEFT, { CMD_SAVE });
                send(MOTOR_RIGHT, { CMD_SAVE });
                job.step = Step::SwapWaitSave;
            } else if (inStep > REPLY_TIMEOUT_MS) {
                send(MOTOR_LEFT, { CMD_REVERT });
                send(MOTOR_RIGHT, { CMD_REVERT });
                setResult(MOTOR_LEFT, "Swap failed: no reply from the %s driver - nothing changed", ga ? "right" : "left");
                endJob();
            }
        } else if (job.step == Step::SwapWaitSave) {
            uint8_t a = 0, b = 0;
            const bool ga = gotReply(MOTOR_LEFT, CMD_SAVE, &a), gb = gotReply(MOTOR_RIGHT, CMD_SAVE, &b);
            if ((ga && gb) || inStep > REPLY_TIMEOUT_MS) {
                setResult(MOTOR_LEFT, "Sides swapped: %s / %s - both rebooting",
                          ga ? statusText(a) : "no reply", gb ? statusText(b) : "no reply");
                forgetSettings(MOTOR_LEFT);
                forgetSettings(MOTOR_RIGHT);
                endJob();
            }
        }
        return;
    }
}

// Velocity telemetry rate: (re)sent whenever a driver is running a firmware
// that supports it and hasn't been sent the current value since it started.
void serviceTelemetryRate(uint32_t now) {
    for (int s = 0; s < 2; s++) {
        if (linkOf(drivers[s], now) != MotorLink::Running) { sentInterval[s] = 0; continue; }
        if (!cfg[s].infoKnown || cfg[s].protocol < 2 || wheelTelemetryMs == 0) continue;
        if (sentInterval[s] == wheelTelemetryMs) continue;
        sentInterval[s] = wheelTelemetryMs;
        sendSetParam((MotorSide)s, PARAM_VELOCITY_INTERVAL_MS, wheelTelemetryMs);
    }
}

} // namespace

void handleDriverConfigReply(MotorSide side, const uint8_t *d, uint8_t len) {
    if (len < 2) return;
    DriverConfigView &c = cfg[side];
    portENTER_CRITICAL(&cfgLock);
    switch (d[0]) {
        case CMD_GET_INFO:
            if (len >= 6) {
                c.infoKnown = true;
                c.noConfigSupport = false;
                c.protocol = d[1];
                c.fwMajor = d[2]; c.fwMinor = d[3]; c.fwPatch = d[4];
                c.imageKB = d[5];
            }
            break;
        case CMD_GET_CONFIG:
            if (len >= 8) {
                float angle;
                const uint32_t bits = ((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 8) | d[7];
                memcpy(&angle, &bits, 4);
                if (d[1] == 0) { c.activeKnown = true; c.side = d[2]; c.flags = d[3]; c.zeroAngle = angle; }
                else           { c.pendingKnown = true; c.pendSide = d[2]; c.pendFlags = d[3]; }
            }
            break;
        default:
            break;
    }
    portEXIT_CRITICAL(&cfgLock);
    lastReply[side].cmd = d[0];
    lastReply[side].arg = d[1];
    lastReply[side].status = (d[0] == CMD_SET_PARAM && len >= 3) ? d[2]
                           : (d[0] == CMD_CALIBRATION && len >= 3) ? d[2]
                           : (d[0] == CMD_NACK) ? CONFIG_ERR_UNKNOWN
                           : d[1];
    lastReply[side].seq = lastReply[side].seq + 1;
    bump();
}

DriverConfigView getDriverConfig(MotorSide side) {
    portENTER_CRITICAL(&cfgLock);
    const DriverConfigView v = cfg[side];
    portEXIT_CRITICAL(&cfgLock);
    return v;
}

uint32_t driverConfigVersion() { return cfgVersion; }

const char *driverJobText() { return job.text; }

static const char *opName(DriverOp op) {
    switch (op) {
        case DriverOp::Refresh:     return "Refresh";
        case DriverOp::SetInvert:   return "Invert";
        case DriverOp::CalStore:    return "Store alignment";
        case DriverOp::CalClear:    return "Clear alignment";
        case DriverOp::Save:        return "Save";
        case DriverOp::Revert:      return "Revert";
        case DriverOp::Reboot:      return "Reboot";
        case DriverOp::Recalibrate: return "Re-run alignment";
    }
    return "?";
}

static const char *startJob(Job j, MotorSide side, DriverOp op, float value) {
    if (op != DriverOp::Refresh && balanceState == BalanceState::Balancing)
        return "Not while it's balancing - pause it or lay it down first";
    if (op != DriverOp::Refresh && !cfg[side].infoKnown && j != Job::Swap)
        return "This driver hasn't answered yet (firmware 1.1 or later is needed for settings over CAN)";
    portENTER_CRITICAL(&cfgLock);
    const bool busy = job.job != Job::None;
    if (!busy) {
        job.side = side;
        job.op = op;
        job.value = value;
        job.step = Step::Send;
        job.stepStartMs = millis();
        job.sawRestart = false;
        snprintf(job.text, sizeof(job.text), "%s (%s)", j == Job::Swap ? "Swap sides" : opName(op),
                 j == Job::Swap ? "both" : kSideName[side]);
        job.job = j; // last: the supervisor starts on this
    }
    portEXIT_CRITICAL(&cfgLock);
    if (busy) return "Another driver operation is still running";
    bump();
    return nullptr;
}

const char *requestDriverOp(MotorSide side, DriverOp op, float value) {
    switch (op) {
        case DriverOp::Refresh:     return startJob(Job::Refresh, side, op, value);
        case DriverOp::Save:
        case DriverOp::Reboot:      return startJob(Job::Save, side, op, value);
        case DriverOp::Recalibrate: return startJob(Job::Recalibrate, side, op, value);
        default:                    return startJob(Job::Simple, side, op, value);
    }
}

const char *requestSwapDriverSides() {
    if (!cfg[0].infoKnown || !cfg[1].infoKnown || cfg[0].protocol < 2 || cfg[1].protocol < 2)
        return "Both drivers need firmware 1.1 or later";
    return startJob(Job::Swap, MOTOR_LEFT, DriverOp::Save, 0);
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
                DeviceLog::printf("[motor] %s driver: no reply to START_FOC\n", kSideName[s]);
            }
        }

        // Log each driver's state changes (shows up in the web page's Log tab).
        for (int s = 0; s < 2; s++) {
            const MotorLink link = linkOf(drivers[s], now);
            if (link != drivers[s].loggedLink) {
                DeviceLog::printf("[motor] %s driver: %s -> %s\n", kSideName[s],
                                  motorLinkName(drivers[s].loggedLink), motorLinkName(link));
                // (Re)appeared on the bus: re-read its firmware and settings.
                if (drivers[s].loggedLink == MotorLink::Offline) needsRefresh[s] = true;
                drivers[s].loggedLink = link;
                bump();
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
                DeviceLog::printf("[motor] %s driver: sending START_FOC (attempt %u)\n", kSideName[s], d.startAttempts);
                sendStartFOC(kCommandId[s]);
                break; // one at a time
            }
        }

        // Driver configuration: one operation at a time; refreshes when idle.
        if (job.job == Job::None) {
            for (int s = 0; s < 2; s++) {
                if (needsRefresh[s] && linkOf(drivers[s], now) != MotorLink::Offline) {
                    needsRefresh[s] = false;
                    startJob(Job::Refresh, (MotorSide)s, DriverOp::Refresh, 0);
                    break;
                }
            }
        }
        runJob(now);
        serviceTelemetryRate(now);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
