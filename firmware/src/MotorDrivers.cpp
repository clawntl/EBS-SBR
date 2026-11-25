#include "MotorDrivers.h"

#include <Arduino.h>

#include "CANConfig.h"
#include "CanBus.h"
#include "DeviceLog.h"
#include "LQRControl.h"
#include "Settings.h"

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

// Set by allowMotorStarts(): until then no START_FOC is sent.
volatile bool startsAllowed = false;

// Node IDs in use for left / right (see driverNode()).
volatile uint8_t activeNode[2] = { 1, 2 };

uint32_t commandId(int side) { return COMMAND_ID_BASE + activeNode[side]; }

// Last heartbeat from each node 0..15 (0 = old firmware "right" board).
volatile uint32_t heardMs[NODE_ID_MAX + 1] = {};
const char *const kSideName[2] = { "Left", "Right" };

bool heartbeatFresh(const DriverTrack &d, uint32_t now) {
    // Signed: the CAN task (other core) may store a heartbeat time later than
    // the caller's `now`; unsigned it would wrap and read as offline.
    return d.everSeen && (int32_t)(now - d.lastHeartbeatMs) < (int32_t)HEARTBEAT_TIMEOUT_MS;
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

void allowMotorStarts() {
    startsAllowed = true;
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
uint8_t leftNodeSetting = 1, rightNodeSetting = 2;

uint8_t driverNode(MotorSide side) { return activeNode[side]; }

void noteNodeHeard(uint8_t node, uint8_t) {
    if (node <= NODE_ID_MAX) heardMs[node] = millis() | 1;
}

uint16_t nodesHeard() {
    const uint32_t now = millis();
    uint16_t bits = 0;
    for (int n = 0; n <= NODE_ID_MAX; n++) {
        const uint32_t t = heardMs[n];
        if (t && (int32_t)(now - t) < 2000) bits |= 1u << n;
    }
    return bits;
}

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
// Rejected PARAM_CURRENT_* writes per driver during a CurrentLoop job.
volatile uint8_t loopErrors[2] = { 0, 0 };
uint8_t sentInterval[2] = { 0, 0 };

enum class Job : uint8_t { None, Refresh, Simple, Save, Recalibrate, ChangeNode, CurrentLoop, Cogging, CogRead };
enum class Step : uint8_t { Send, WaitReply, WaitInfo, WaitBoard,
                            RecalWaitClear, RecalWaitSave1, RecalWaitAligned, RecalWaitStore,
                            RecalWaitSave2, RecalWaitRunning,
                            NodeWaitSet, NodeWaitSave, LoopWait, CogWait, CogRun,
                            ReadInfo, ReadBins, ReadWait };

// Anti-cogging calibration: how long both drivers may take (the driver's
// sweep is ~52 s at its default speed), and how often they're polled.
constexpr uint32_t COG_CAL_TIMEOUT_MS = 150000;
constexpr uint32_t COG_POLL_MS = 500;
volatile bool cogAbortRequested = false;
volatile bool cogCalibrating = false;

// Anti-cogging tables read from the drivers (requestCogTableRead). Frames of
// 4 bins; cogGot marks which frames arrived.
constexpr int COG_FRAMES = DRIVER_COG_BINS / 4;
int8_t cogTab[2][DRIVER_COG_BINS];
volatile float cogScale[2] = { 0, 0 };     // A per count, 0 = no table
volatile bool cogInfoSeen[2] = { false, false };
volatile uint32_t cogGot[2][(COG_FRAMES + 31) / 32];
bool cogReady[2] = { false, false };
std::atomic<uint32_t> cogTabVersion{0};
uint16_t cogReadNext = 0;   // next frame to request in this round
uint8_t cogReadRound = 0;
constexpr int COG_READ_BURST = 4;          // requests per driver per supervisor pass (5 ms apart while reading)
constexpr uint8_t COG_READ_ROUNDS = 4;     // first pass + re-requests of missing frames

bool cogFrameGot(int s, int f) { return cogGot[s][f >> 5] & (1u << (f & 31)); }
int cogFramesGot(int s) { int n = 0; for (int f = 0; f < COG_FRAMES; f++) n += cogFrameGot(s, f); return n; }
void send(MotorSide side, std::initializer_list<uint8_t> bytes);
void requestCogFrame(int s, int f) {
    const uint16_t bin = (uint16_t)(f * 4);
    send((MotorSide)s, { CMD_COGGING, COG_OP_READ, (uint8_t)(bin >> 8), (uint8_t)bin });
}

struct JobState {
    Job job = Job::None;
    MotorSide side = MOTOR_LEFT;
    DriverOp op = DriverOp::Refresh;
    float value = 0;
    float value2 = 0, value3 = 0; // CurrentLoop: I and filter (value = P)
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
    sendDriverCommand(commandId(side), b, n);
}

void sendSetParam(MotorSide side, uint8_t param, float v) {
    uint8_t b[6] = { CMD_SET_PARAM, param };
    uint32_t bits;
    memcpy(&bits, &v, 4);
    b[2] = bits >> 24; b[3] = bits >> 16; b[4] = bits >> 8; b[5] = bits;
    sendDriverCommand(commandId(side), b, sizeof(b));
}

void sendQueries(MotorSide side) {
    send(side, { CMD_GET_INFO });
    send(side, { CMD_GET_CONFIG, 0 });
    send(side, { CMD_GET_CONFIG, 1 });
}

// Current loop and FOC loop rate (protocol 5+); the replies fill cfg.
void sendParamQueries(MotorSide side) {
    for (uint8_t p : { (uint8_t)PARAM_CURRENT_P, (uint8_t)PARAM_CURRENT_I,
                       (uint8_t)PARAM_CURRENT_TF, (uint8_t)PARAM_FOC_LOOP_HZ })
        send(side, { CMD_GET_PARAM, p });
}

// Anti-cogging status and last result (protocol 7+).
void sendCogQueries(MotorSide side) {
    send(side, { CMD_COGGING, COG_OP_STATUS });
    send(side, { CMD_COGGING, COG_OP_RESULT });
}

const char *cogFailureText(uint8_t code) {
    switch (code) {
        case 1: return "FOC stopped";
        case 2: return "a current command arrived";
        case 3: return "the wheel didn't follow (blocked?)";
        case 4: return "aborted";
        case 5: return "angles missed";
        case 6: return "no table to refine";
    }
    return "?";
}

// Model name pages 1-5 (protocol 4+). The replies carry their page number, so
// they can all be sent at once; the driver answers them in order.
void sendBoardInfoQueries(MotorSide side) {
    for (uint8_t page = 1; page <= 5; page++) send(side, { CMD_GET_BOARD_INFO, page });
}

const char *statusText(uint8_t st) {
    switch (st) {
        case CONFIG_OK:              return "OK";
        case CONFIG_ERR_UNKNOWN:     return "not supported by this firmware";
        case CONFIG_ERR_BAD_VALUE:   return "value out of range";
        case CONFIG_ERR_NOT_ALIGNED: return "no alignment to store (motor not running)";
        case CONFIG_ERR_FLASH:       return "firmware too big - settings page in use";
        case CONFIG_ERR_NOT_RUNNING: return "the motor isn't running";
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
            portENTER_CRITICAL(&cfgLock);
            cfg[sd].modelKnown = false;
            cfg[sd].model[0] = 0;
            portEXIT_CRITICAL(&cfgLock);
            markSent(sd, now);
            sendQueries(sd);
            job.step = Step::WaitInfo;
        } else if (job.step == Step::WaitBoard) {
            // Older firmware NACKs CMD_GET_BOARD_INFO; it isn't asked (protocol < 4).
            if (cfg[sd].modelKnown || inStep > REPLY_TIMEOUT_MS) endJob();
        } else if (cfg[sd].infoKnown && lastReply[sd].seq != job.seqAtSend[sd] && inStep > 100) {
            if (cfg[sd].protocol >= 4) {
                markSent(sd, now);
                sendBoardInfoQueries(sd);
                if (cfg[sd].protocol >= 5) sendParamQueries(sd);
                if (cfg[sd].protocol >= 7) {
                    sendCogQueries(sd);
                    send(sd, { CMD_GET_BOARD_INFO, 6 }); // serial number
                    send(sd, { CMD_GET_BOARD_INFO, 7 });
                }
                job.step = Step::WaitBoard;
            } else {
                endJob();
            }
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
            else {
                send(sd, { CMD_GET_CONFIG, 0 });
                send(sd, { CMD_GET_CONFIG, 1 });
                if (cfg[sd].protocol >= 5) sendParamQueries(sd); // a revert also restores the current loop
                if (cfg[sd].protocol >= 7) sendCogQueries(sd);   // ... and the anti-cogging table
            }
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

    case Job::CurrentLoop: {
        // Both drivers, three SET_PARAMs each; done when both have answered
        // three times (or on the timeout).
        if (job.step == Step::Send) {
            for (int s = 0; s < 2; s++) {
                loopErrors[s] = 0;
                job.seqAtSend[s] = lastReply[s].seq;
                sendSetParam((MotorSide)s, PARAM_CURRENT_P, job.value);
                sendSetParam((MotorSide)s, PARAM_CURRENT_I, job.value2);
                sendSetParam((MotorSide)s, PARAM_CURRENT_TF, job.value3);
            }
            job.stepStartMs = now;
            job.step = Step::LoopWait;
        } else {
            bool all = true;
            for (int s = 0; s < 2; s++) all &= (uint32_t)(lastReply[s].seq - job.seqAtSend[s]) >= 3;
            if (!all && inStep <= REPLY_TIMEOUT_MS) break;
            for (int s = 0; s < 2; s++) {
                const MotorSide ms = (MotorSide)s;
                const bool answered = (uint32_t)(lastReply[s].seq - job.seqAtSend[s]) >= 3;
                if (!answered)         setResult(ms, "Current loop: no reply");
                else if (loopErrors[s]) setResult(ms, "Current loop: rejected (value out of range?)");
                else                   setResult(ms, "Current loop applied - Save & reboot to keep it");
                send(ms, { CMD_GET_CONFIG, 0 }); // the unsaved flag
                send(ms, { CMD_GET_CONFIG, 1 });
                sendParamQueries(ms);
            }
            endJob();
        }
        break;
    }

    case Job::Cogging: {
        // Both drivers. job.value = CogAction.
        const CogAction a = (CogAction)lroundf(job.value);
        const bool measuring = a == CogAction::Calibrate || a == CogAction::Refine;
        if (job.step == Step::Send) {
            for (int s = 0; s < 2; s++) {
                const MotorSide ms = (MotorSide)s;
                job.seqAtSend[s] = lastReply[s].seq;
                switch (a) {
                    case CogAction::Calibrate: send(ms, { CMD_COGGING, COG_OP_CALIBRATE }); break;
                    case CogAction::Refine:    send(ms, { CMD_COGGING, COG_OP_REFINE }); break;
                    case CogAction::Abort:     send(ms, { CMD_COGGING, COG_OP_ABORT }); break;
                    case CogAction::Clear:     send(ms, { CMD_COGGING, COG_OP_CLEAR }); break;
                    case CogAction::CompOn:    sendSetParam(ms, PARAM_COG_COMP, 1); break;
                    case CogAction::CompSlots: sendSetParam(ms, PARAM_COG_COMP, 2); break;
                    case CogAction::CompOff:   sendSetParam(ms, PARAM_COG_COMP, 0); break;
                }
            }
            if (measuring) cogCalibrating = true; // from now on the controller sends 0 A
            job.stepStartMs = now;
            job.step = Step::CogWait;
        } else if (job.step == Step::CogWait) {
            bool all = true;
            for (int s = 0; s < 2; s++) all &= lastReply[s].seq != job.seqAtSend[s];
            if (!all && inStep <= REPLY_TIMEOUT_MS) break;
            bool ok = true;
            for (int s = 0; s < 2; s++) {
                const MotorSide ms = (MotorSide)s;
                const bool answered = lastReply[s].seq != job.seqAtSend[s];
                const uint8_t st = lastReply[s].cmd == CMD_NACK ? CONFIG_ERR_UNKNOWN : lastReply[s].status;
                if (!answered)            { setResult(ms, "Anti-cogging: no reply"); ok = false; }
                else if (st != CONFIG_OK) { setResult(ms, "Anti-cogging: %s", statusText(st)); ok = false; }
                else if (!measuring)      setResult(ms, a == CogAction::Abort ? "Anti-cogging: stopped"
                                                    : a == CogAction::Clear ? "Anti-cogging table cleared - Save & reboot to keep that"
                                                    : "Anti-cogging compensation %s - Save & reboot to keep it",
                                                    a == CogAction::CompOn ? "on" : a == CogAction::CompSlots ? "slots only" : "off");
                send(ms, { CMD_GET_CONFIG, 0 });
                send(ms, { CMD_GET_CONFIG, 1 }); // (the page shows the pending copy's unsaved flag)
                sendCogQueries(ms);
            }
            if (measuring && ok) {
                job.step = Step::CogRun;
                job.stepStartMs = now;
                job.seqAtSend[0] = now; // reused: last poll time
                setJobText("Anti-cogging: measuring both wheels (keep them free)");
            } else {
                if (measuring) {
                    // One started, the other didn't: stop the one that did.
                    for (int s = 0; s < 2; s++) send((MotorSide)s, { CMD_COGGING, COG_OP_ABORT });
                }
                cogCalibrating = false;
                endJob();
            }
        } else { // CogRun: poll both until neither is calibrating
            const DriverConfigView l = getDriverConfig(MOTOR_LEFT), r = getDriverConfig(MOTOR_RIGHT);
            auto running = [](const DriverConfigView &c) {
                return c.cogState == COG_STATE_SETTLING || c.cogState == COG_STATE_FORWARD || c.cogState == COG_STATE_BACKWARD;
            };
            const bool fresh = (now - job.stepStartMs) > 1000; // replies to the polls since the start
            if (cogAbortRequested) {
                cogAbortRequested = false;
                for (int s = 0; s < 2; s++) send((MotorSide)s, { CMD_COGGING, COG_OP_ABORT });
            }
            const bool lost = linkOf(drivers[0], now) != MotorLink::Running || linkOf(drivers[1], now) != MotorLink::Running;
            if (fresh && !running(l) && !running(r)) {
                for (int s = 0; s < 2; s++) {
                    const DriverConfigView &c = s ? r : l;
                    if (c.cogState == COG_STATE_DONE)
                        setResult((MotorSide)s, "Anti-cogging measured: %u mA peak-to-peak - turn it on, then Save & reboot to keep it",
                                  c.cogAmpMa);
                    else
                        setResult((MotorSide)s, "Anti-cogging calibration failed: %s",
                                  c.cogState == COG_STATE_FAILED ? cogFailureText(c.cogDetail) : "?");
                    sendCogQueries((MotorSide)s);
                    send((MotorSide)s, { CMD_GET_CONFIG, 0 });
                    send((MotorSide)s, { CMD_GET_CONFIG, 1 });
                }
                cogCalibrating = false;
                endJob();
            } else if (inStep > COG_CAL_TIMEOUT_MS || lost) {
                for (int s = 0; s < 2; s++) {
                    send((MotorSide)s, { CMD_COGGING, COG_OP_ABORT });
                    setResult((MotorSide)s, lost ? "Anti-cogging calibration stopped: a driver stopped running"
                                                 : "Anti-cogging calibration timed out");
                }
                cogCalibrating = false;
                endJob();
            } else if (now - job.seqAtSend[0] >= COG_POLL_MS) {
                job.seqAtSend[0] = now;
                for (int s = 0; s < 2; s++) send((MotorSide)s, { CMD_COGGING, COG_OP_STATUS });
                char t[64];
                snprintf(t, sizeof(t), "Anti-cogging: measuring (left %u%%, right %u%%) - keep the wheels free",
                         running(l) || l.cogState == COG_STATE_DONE ? l.cogDetail : 0,
                         running(r) || r.cogState == COG_STATE_DONE ? r.cogDetail : 0);
                setJobText(t);
            }
        }
        break;
    }

    case Job::CogRead: {
        if (job.step == Step::Send) {
            for (int s = 0; s < 2; s++) {
                cogInfoSeen[s] = false;
                cogScale[s] = 0;
                cogReady[s] = false;
                for (auto &w : cogGot[s]) w = 0;
                send((MotorSide)s, { CMD_COGGING, COG_OP_INFO });
            }
            job.stepStartMs = now;
            job.step = Step::ReadInfo;
        } else if (job.step == Step::ReadInfo) {
            if ((cogInfoSeen[0] && cogInfoSeen[1]) || inStep > REPLY_TIMEOUT_MS) {
                cogReadNext = 0;
                cogReadRound = 0;
                job.step = Step::ReadBins;
                job.stepStartMs = now;
            }
        } else if (job.step == Step::ReadBins) {
            // A burst per pass: the next COG_READ_BURST frames that either
            // driver (with a table) still lacks; re-request rounds skip the
            // frames already received.
            int burst = 0;
            while (cogReadNext < COG_FRAMES && burst < COG_READ_BURST) {
                bool asked = false;
                for (int s = 0; s < 2; s++) {
                    if (cogScale[s] > 0 && !cogFrameGot(s, cogReadNext)) { requestCogFrame(s, cogReadNext); asked = true; }
                }
                cogReadNext++;
                if (asked) burst++;
            }
            if (cogReadNext >= COG_FRAMES) { job.step = Step::ReadWait; job.stepStartMs = now; }
        } else { // ReadWait: let the last replies arrive, then another round for any missing
            if (inStep < 60) break;
            bool missing = false;
            for (int s = 0; s < 2; s++) if (cogScale[s] > 0 && cogFramesGot(s) < COG_FRAMES) missing = true;
            if (missing && ++cogReadRound < COG_READ_ROUNDS) {
                cogReadNext = 0;
                job.step = Step::ReadBins;
                break;
            }
            for (int s = 0; s < 2; s++) {
                const int got = cogFramesGot(s);
                cogReady[s] = cogScale[s] > 0 && got == COG_FRAMES;
                if (!cogInfoSeen[s])        setResult((MotorSide)s, "Anti-cogging table: no reply (firmware too old?)");
                else if (!(cogScale[s] > 0)) setResult((MotorSide)s, "Anti-cogging table: this driver has none");
                else if (!cogReady[s])      setResult((MotorSide)s, "Anti-cogging table: only %d of %d parts arrived", got, COG_FRAMES);
            }
            cogTabVersion++;
            endJob();
        }
        break;
    }

    case Job::ChangeNode: {
        const uint8_t newId = (uint8_t)lroundf(job.value);
        if (job.step == Step::Send) {
            markSent(sd, now);
            sendSetParam(sd, PARAM_NODE_ID, newId);
            job.step = Step::NodeWaitSet;
        } else if (job.step == Step::NodeWaitSet) {
            if (gotReply(sd, CMD_SET_PARAM, &st)) {
                if (st != CONFIG_OK) { setResult(sd, "Node ID change failed: %s", statusText(st)); endJob(); break; }
                markSent(sd, now);
                send(sd, { CMD_SAVE });
                job.step = Step::NodeWaitSave;
            } else if (inStep > REPLY_TIMEOUT_MS) { setResult(sd, "Node ID change failed: no reply"); endJob(); }
        } else if (job.step == Step::NodeWaitSave) {
            if (gotReply(sd, CMD_SAVE, &st)) {
                if (st != CONFIG_OK) {
                    send(sd, { CMD_REVERT });
                    setResult(sd, "Node ID change failed: save: %s", statusText(st));
                    endJob();
                    break;
                }
                // The board reboots as node newId: follow it.
                Settings::set(sd == MOTOR_LEFT ? "leftNode" : "rightNode", newId);
                forgetSettings(sd);
                setResult(sd, "Now node %u - rebooting", newId);
                endJob();
            } else if (inStep > REPLY_TIMEOUT_MS) {
                send(sd, { CMD_REVERT });
                setResult(sd, "Node ID change failed: no reply to save");
                endJob();
            }
        }
        break;
    }
    }
}

// Takes the wanted left/right node IDs into use, only while not balancing
// (switching mid-balance would send one wheel's current to another board).
void serviceNodeMapping() {
    const uint8_t l = leftNodeSetting, r = rightNodeSetting;
    if (l == activeNode[0] && r == activeNode[1]) return;
    if (l < NODE_ID_MIN || l > NODE_ID_MAX || r < NODE_ID_MIN || r > NODE_ID_MAX || l == r) return;
    if (balanceState == BalanceState::Balancing) return;
    for (int s = 0; s < 2; s++) {
        const uint8_t n = s ? r : l;
        if (activeNode[s] == n) continue;
        DeviceLog::printf("[driver] %s driver: node %u -> %u\n", kSideName[s], activeNode[s], n);
        activeNode[s] = n;
        drivers[s] = DriverTrack{};
        portENTER_CRITICAL(&cfgLock);
        cfg[s] = DriverConfigView{};
        portEXIT_CRITICAL(&cfgLock);
        sentInterval[s] = 0;
        needsRefresh[s] = true;
    }
    // Remember it (only these two keys; other unsaved settings stay unsaved).
    if (Settings::savedValue("leftNode") != l) Settings::saveKey("leftNode");
    if (Settings::savedValue("rightNode") != r) Settings::saveKey("rightNode");
    bump();
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
        case CMD_GET_BOARD_INFO:
            // Pages 1-5: 6 characters each of the model name, NUL-padded.
            if (len >= 8 && (d[1] == 6 || d[1] == 7)) {
                // Serial number (MCU unique ID), 6 bytes per page.
                static const char hex[] = "0123456789ABCDEF";
                const uint8_t at = (d[1] - 6) * 12;
                for (uint8_t i = 0; i < 6; i++) {
                    c.serial[at + 2 * i]     = hex[d[2 + i] >> 4];
                    c.serial[at + 2 * i + 1] = hex[d[2 + i] & 15];
                }
                c.serial[24] = 0;
                c.serialPages |= d[1] == 6 ? 1 : 2;
                c.serialKnown = c.serialPages == 3;
            }
            if (len >= 8 && d[1] >= 1 && d[1] <= 5) {
                const uint8_t at = (d[1] - 1) * 6;
                bool end = false;
                for (uint8_t i = 0; i < 6; i++) {
                    c.model[at + i] = (char)d[2 + i];
                    if (d[2 + i] == 0) end = true;
                }
                c.model[sizeof(c.model) - 1] = 0;
                if (end || d[1] == 5) c.modelKnown = true;
            }
            break;
        case CMD_SET_PARAM:
        case CMD_GET_PARAM:
            // [cmd][param][status][f32 value]
            if (len >= 7) {
                const uint8_t param = d[1];
                const bool ok = d[2] == CONFIG_OK;
                float v;
                const uint32_t bits = ((uint32_t)d[3] << 24) | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 8) | d[6];
                memcpy(&v, &bits, 4);
                const bool loopParam = param == PARAM_CURRENT_P || param == PARAM_CURRENT_I || param == PARAM_CURRENT_TF;
                if (d[0] == CMD_SET_PARAM && loopParam && !ok) loopErrors[side] = loopErrors[side] + 1;
                if (ok) {
                    if (param == PARAM_CURRENT_P)   { c.curP = v;  c.loopKnown = true; }
                    if (param == PARAM_CURRENT_I)   { c.curI = v;  c.loopKnown = true; }
                    if (param == PARAM_CURRENT_TF)  { c.curTf = v; c.loopKnown = true; }
                    if (param == PARAM_FOC_LOOP_HZ) c.focHz = v;
                }
            }
            break;
        case CMD_COGGING:
            if (len >= 8 && d[1] == COG_OP_READ) {
                if (d[2] != 0xFF) {
                    const int bin = (d[2] << 8) | d[3];
                    if (bin >= 0 && bin + 3 < DRIVER_COG_BINS && (bin & 3) == 0) {
                        for (int i = 0; i < 4; i++) cogTab[side][bin + i] = (int8_t)d[4 + i];
                        cogGot[side][(bin / 4) >> 5] |= 1u << ((bin / 4) & 31);
                    }
                }
                break;
            }
            if (len >= 8 && d[1] == COG_OP_INFO) {
                float sc;
                const uint32_t bits = ((uint32_t)d[3] << 24) | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 8) | d[6];
                memcpy(&sc, &bits, 4);
                cogScale[side] = (d[2] == CONFIG_OK && d[7] * 16 == DRIVER_COG_BINS && sc > 0) ? sc : 0.0f;
                cogInfoSeen[side] = true;
                break;
            }
            if (len >= 8 && d[2] == CONFIG_OK) {
                if (d[1] == COG_OP_RESULT) {
                    c.cogResultKnown = true;
                    c.cogCoverage = d[3]; c.cogQuality = d[4]; c.cogMaxErr = d[5]; c.cogNoise = d[6];
                } else {
                    c.cogKnown = true;
                    c.cogState = d[3]; c.cogDetail = d[4]; c.cogFlags = d[5];
                    c.cogAmpMa = d[6] * 4; c.cogFricMa = d[7] * 2;
                }
            } else if (len >= 8 && d[1] != COG_OP_RESULT) {
                // Refused (e.g. not running): the status bytes still describe it.
                c.cogKnown = true;
                c.cogState = d[3]; c.cogDetail = d[4]; c.cogFlags = d[5];
                c.cogAmpMa = d[6] * 4; c.cogFricMa = d[7] * 2;
            }
            break;
        case CMD_GET_CONFIG:
            if (len >= 8) {
                float angle;
                const uint32_t bits = ((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 8) | d[7];
                memcpy(&angle, &bits, 4);
                if (d[1] == 0) { c.activeKnown = true; c.node = d[2]; c.flags = d[3]; c.zeroAngle = angle; }
                else           { c.pendingKnown = true; c.pendNode = d[2]; c.pendFlags = d[3]; }
            }
            break;
        default:
            break;
    }
    portEXIT_CRITICAL(&cfgLock);
    lastReply[side].cmd = d[0];
    lastReply[side].arg = d[1];
    lastReply[side].status = ((d[0] == CMD_SET_PARAM || d[0] == CMD_GET_PARAM) && len >= 3) ? d[2]
                           : ((d[0] == CMD_CALIBRATION || d[0] == CMD_COGGING) && len >= 3) ? d[2]
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
        case DriverOp::SetNodeId:   return "Change node ID";
        case DriverOp::CurrentLoop: return "Current loop";
        case DriverOp::Cogging:     return "Anti-cogging";
        case DriverOp::CogRead:     return "Read anti-cogging tables";
    }
    return "?";
}

static const char *startJob(Job j, MotorSide side, DriverOp op, float value, float value2 = 0, float value3 = 0) {
    // (Anti-cogging on/off is allowed while balancing: it applies at once,
    // like the current loop, and comparing with/without is the point.)
    const bool cogToggle = op == DriverOp::Cogging && (value == (float)CogAction::CompOn || value == (float)CogAction::CompOff
                                                       || value == (float)CogAction::CompSlots);
    if (op != DriverOp::Refresh && !cogToggle && balanceState == BalanceState::Balancing)
        return "Not while it's balancing - pause it or lay it down first";
    if (op != DriverOp::Refresh && !cfg[side].infoKnown)
        return "This driver hasn't answered yet (firmware 1.1 or later is needed for settings over CAN)";
    portENTER_CRITICAL(&cfgLock);
    const bool busy = job.job != Job::None;
    if (!busy) {
        job.side = side;
        job.op = op;
        job.value = value;
        job.value2 = value2;
        job.value3 = value3;
        job.step = Step::Send;
        job.stepStartMs = millis();
        job.sawRestart = false;
        if (j == Job::CurrentLoop || j == Job::Cogging || j == Job::CogRead) snprintf(job.text, sizeof(job.text), "%s (both drivers)", opName(op));
        else snprintf(job.text, sizeof(job.text), "%s (%s)", opName(op), kSideName[side]);
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
        case DriverOp::SetNodeId: {
            const int id = (int)lroundf(value);
            if (id < NODE_ID_MIN || id > NODE_ID_MAX) return "Node IDs are 1-15";
            if (id == activeNode[side]) return "It's already that node";
            if (id == activeNode[side ? 0 : 1]) return "That's the other wheel's node";
            if (nodesHeard() & (1u << id)) return "Another board on the bus already uses that node ID";
            if (cfg[side].infoKnown && cfg[side].protocol < 3) return "This driver's firmware has no node IDs (needs 1.2)";
            return startJob(Job::ChangeNode, side, op, value);
        }
        default:                    return startJob(Job::Simple, side, op, value);
    }
}

const char *requestCurrentLoop(float p, float i, float tf) {
    if (!(p >= 0 && p <= 50) || !(i >= 0 && i <= 100000) || !(tf >= 0 && tf <= 0.01f))
        return "Out of range: P 0-50 V/A, I 0-100000 V/(A s), filter 0-10 ms";
    for (int s = 0; s < 2; s++) {
        if (!cfg[s].infoKnown || cfg[s].protocol < 5)
            return "Both drivers need firmware 1.4 or later for the current loop settings";
        if (linkOf(drivers[s], millis()) == MotorLink::Offline) return "A driver is offline";
    }
    return startJob(Job::CurrentLoop, MOTOR_LEFT, DriverOp::CurrentLoop, p, i, tf);
}

const char *requestCogging(CogAction a) {
    if (a == CogAction::Abort) {
        if (job.job == Job::Cogging && job.step == Step::CogRun) { cogAbortRequested = true; return nullptr; }
        // Not ours (e.g. started over USB): ask the drivers anyway.
        for (int s = 0; s < 2; s++) send((MotorSide)s, { CMD_COGGING, COG_OP_ABORT });
        return nullptr;
    }
    for (int s = 0; s < 2; s++) {
        if (!cfg[s].infoKnown || cfg[s].protocol < 7) return "Both drivers need firmware 1.7 or later for anti-cogging";
        if (linkOf(drivers[s], millis()) == MotorLink::Offline) return "A driver is offline";
    }
    if (a == CogAction::Calibrate || a == CogAction::Refine) {
        if (balanceState != BalanceState::Tipped)
            return "Lay the robot down first (past the cut-out angle), with both wheels free to turn";
        if (!motorsReady()) return "Both motors must be running";
        if (a == CogAction::Refine) {
            for (int s = 0; s < 2; s++)
                if (!(cfg[s].cogFlags & COG_FLAG_TABLE) || (cfg[s].cogFlags & COG_FLAG_MISMATCH))
                    return "Refine needs a table on both drivers - calibrate first";
        }
    }
    return startJob(Job::Cogging, MOTOR_LEFT, DriverOp::Cogging, (float)a);
}

bool driverCalibrating() { return cogCalibrating; }

const char *requestCogTableRead() {
    for (int s = 0; s < 2; s++) {
        if (!cfg[s].infoKnown || cfg[s].protocol < 7) return "Both drivers need firmware 1.7 or later for anti-cogging";
        if (linkOf(drivers[s], millis()) == MotorLink::Offline) return "A driver is offline";
    }
    // (startJob refuses while balancing: the read adds ~3000 frames a second to the bus.)
    return startJob(Job::CogRead, MOTOR_LEFT, DriverOp::CogRead, 0);
}

bool cogTableReady(MotorSide side) { return cogReady[side]; }
float cogTableAmps(MotorSide side, int bin) {
    if (!cogReady[side] || bin < 0 || bin >= DRIVER_COG_BINS) return 0;
    return cogTab[side][bin] * cogScale[side];
}
uint32_t cogTablesVersion() { return cogTabVersion; }

const char *requestDriverNodes(uint8_t l, uint8_t r) {
    if (balanceState == BalanceState::Balancing) return "Not while it's balancing - pause it or lay it down first";
    if (l < NODE_ID_MIN || l > NODE_ID_MAX || r < NODE_ID_MIN || r > NODE_ID_MAX) return "Node IDs are 1-15";
    if (l == r) return "Left and right must be different nodes";
    Settings::set("leftNode", l);
    Settings::set("rightNode", r);
    return nullptr;
}

const char *requestSwapDriverSides() {
    return requestDriverNodes(activeNode[1], activeNode[0]);
}

void Task_motorSupervisor(void *pvParams) {
    uint32_t lastStartAttemptMs[2] = { 0, 0 };
    bool startedOnce[2] = { false, false };
    bool alignHintLogged[2] = { false, false };

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
                // Started: the FOC loop rate (and current loop) can be read now.
                if (link == MotorLink::Running && cfg[s].infoKnown && cfg[s].protocol >= 5)
                    sendParamQueries((MotorSide)s);
                drivers[s].loggedLink = link;
                bump();
            }
        }

        // A driver without a stored alignment aligns at every start: it turns
        // its wheel and draws more current for a few seconds, so it starts on
        // its own. Drivers with a stored alignment (only a short current-sense
        // check, no wheel turn) start together.
        auto quickStart = [](const DriverTrack &d) { return (d.flags & DRIVER_FLAG_CALIBRATION_USED) != 0; };
        const bool slowPending = (drivers[0].startPending && !quickStart(drivers[0])) ||
                                 (drivers[1].startPending && !quickStart(drivers[1]));
        if (startsAllowed && !slowPending) {
            for (int s = 0; s < 2; s++) {
                DriverTrack &d = drivers[s];
                const MotorLink link = linkOf(d, now);
                if (link != MotorLink::Idle && link != MotorLink::Fault) continue;
                if (startedOnce[s] && (now - lastStartAttemptMs[s]) < START_RETRY_INTERVAL_MS) continue;
                const bool quick = quickStart(d);
                if (!quick && (drivers[0].startPending || drivers[1].startPending)) continue;

                d.startAttempts = d.startAttempts + 1;
                d.startPending = true;
                d.startSentMs = now;
                lastStartAttemptMs[s] = now;
                startedOnce[s] = true;
                DeviceLog::printf("[motor] %s driver: sending START_FOC (attempt %u)\n", kSideName[s], d.startAttempts);
                if (!quick && !alignHintLogged[s]) {
                    alignHintLogged[s] = true;
                    DeviceLog::printf("[motor] %s driver has no stored alignment: it aligns (wheel turns) at every "
                                      "start. Re-run alignment (Drivers tab) stores one; boots faster.\n", kSideName[s]);
                }
                sendStartFOC(commandId(s));
                if (!quick) break; // starts on its own
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
        if (job.job == Job::None) serviceNodeMapping();
        serviceTelemetryRate(now);

        // Faster while reading the anti-cogging tables (a burst per pass).
        vTaskDelay(pdMS_TO_TICKS(job.job == Job::CogRead ? 5 : 100));
    }
}
