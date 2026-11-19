#include "Capture.h"

#include <atomic>
#include <esp_heap_caps.h>
#include <vector>

#include "Battery.h"
#include "DriveInput.h"
#include "RobotParams.h"
#include "Settings.h"

#include "WebInterface.h"

namespace Capture {
namespace {

// Up to 60 s at 500 Hz, if there's room in PSRAM (see start()). At a higher
// control rate the maximum duration shrinks; recording every Nth step
// ("recEvery") stretches it.
constexpr uint32_t MAX_SAMPLES = 30000;
// PSRAM left free for everything else (Wi-Fi/web buffers can live there).
constexpr size_t PSRAM_RESERVE = 256 * 1024;
// Without PSRAM (shouldn't happen on the ESP32-S3 Zero) fall back to this.
constexpr uint32_t FALLBACK_SAMPLES = 1000;

// Fixed point, 60 bytes per sample (30 000 samples = 1.8 MB of PSRAM).
struct Sample {
    uint32_t tUs;      // since the trigger
    int16_t tilt;      // deg x 100
    int16_t tiltRate;  // deg/s x 10
    int16_t setpoint;  // deg x 100
    int16_t phi;       // rad x 10
    int16_t phiDot;    // rad/s x 100
    int16_t yawRate;   // deg/s x 10
    int16_t wheelL;    // rad/s x 100
    int16_t wheelR;    // rad/s x 100
    int16_t lqr;       // A x 1000
    int16_t curL;      // A x 1000
    int16_t curR;      // A x 1000
    int8_t joyX;       // x 100
    int8_t joyY;       // x 100
    uint8_t state;     // BalanceState
    uint8_t source;    // DriveInput::Source (bits 0-3) | module id << 4
    int16_t posErr;    // rad x 100
    int16_t speedRef;  // rad/s x 100
    int16_t heading;   // deg x 10
    int16_t headErr;   // deg x 100
    int16_t yawRateRef;// deg/s x 10
    int16_t yawCur;    // A x 1000
    int16_t pitch;     // deg x 100
    int16_t gyro[3];   // deg/s x 10 (+-2000 deg/s range)
    int16_t accel[3];  // g x 10000 (+-2 g range)
    uint8_t lifted;    // 0 none, 1 left, 2 right
    uint8_t hold;      // holdScale x 100
};
static_assert(sizeof(Sample) == 60, "Sample should pack to 60 bytes");

Sample *buffer = nullptr;
uint32_t allocated = 0;
std::atomic<State> current{State::Idle};
std::atomic<uint32_t> count{0};
uint32_t cap = 0;
uint32_t startUs = 0;
uint32_t every = 1;   // record every Nth control step
uint32_t skip = 0;
Trigger trigger = Trigger::Now;
std::atomic<int> readers{0};

// Context at start, written into the CSV header.
float rateAtStart = 0;
float batteryAtStart = 0;
std::vector<float> settingsAtStart;

int16_t fixed(float v, float scale) {
    const float x = v * scale;
    return (int16_t)constrain(x, -32767.0f, 32767.0f);
}

int8_t fixed8(float v, float scale) {
    const float x = v * scale;
    return (int8_t)constrain(x, -127.0f, 127.0f);
}

void beginRecording(uint32_t nowUs) {
    startUs = nowUs;
    count = 0;
    current = State::Recording;
}

const char *stateCsvName(uint8_t s) {
    switch ((BalanceState)s) {
    case BalanceState::MotorsNotReady: return "motors-off";
    case BalanceState::ImuStalled:     return "imu-stall";
    case BalanceState::Paused:         return "paused";
    case BalanceState::Tipped:         return "tipped";
    case BalanceState::BatteryLow:     return "battery-low";
    case BalanceState::Balancing:      return "balancing";
    case BalanceState::Waiting:        return "waiting";
    case BalanceState::Runaway:        return "runaway";
    }
    return "?";
}

} // namespace

const char *stateName(State s) {
    switch (s) {
    case State::Idle:      return "idle";
    case State::Armed:     return "armed";
    case State::Recording: return "recording";
    case State::Done:      return "done";
    }
    return "?";
}

bool start(uint8_t seconds, Trigger trig) {
    if (readers > 0) return false; // a download is reading the buffer
    if (!buffer) {
        // As much as PSRAM allows, up to MAX_SAMPLES, leaving some free.
        const size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        const size_t usable = largest > PSRAM_RESERVE ? largest - PSRAM_RESERVE : 0;
        allocated = (uint32_t)min((size_t)MAX_SAMPLES, usable / sizeof(Sample));
        if (allocated >= FALLBACK_SAMPLES) {
            buffer = (Sample *)heap_caps_malloc(allocated * sizeof(Sample), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        }
        if (!buffer) {
            buffer = (Sample *)heap_caps_malloc(FALLBACK_SAMPLES * sizeof(Sample), MALLOC_CAP_8BIT);
            allocated = FALLBACK_SAMPLES;
        }
        if (!buffer) {
            allocated = 0;
            return false;
        }
    }
    current = State::Idle; // stop Task_LQR writing while we set up
    every = (uint32_t)constrain(lroundf(Settings::value("recEvery")), 1L, 20L);
    skip = 0;
    rateAtStart = (IMUsampleRate ? IMUsampleRate : 500) / (float)every;
    const uint32_t wanted = (uint32_t)(constrain((int)seconds, 1, (int)CAPTURE_MAX_SECONDS) * rateAtStart);
    cap = min(wanted, allocated);
    count = 0;
    trigger = trig;
    batteryAtStart = Battery::voltage();
    size_t paramCount;
    Settings::params(paramCount);
    settingsAtStart.resize(paramCount);
    for (size_t i = 0; i < paramCount; i++) settingsAtStart[i] = Settings::valueAt(i);
    if (trig == Trigger::Now) beginRecording(micros());
    else current = State::Armed;
    return true;
}

void cancel() {
    if (current == State::Armed) current = State::Idle;
    else if (current == State::Recording) current = count ? State::Done : State::Idle;
}

State state() { return current; }
uint32_t samples() { return count; }
uint32_t capacity() { return cap; }
float rateHz() { return rateAtStart; }

void record(const ControlSample &s, BalanceState previousState) {
    State st = current;
    if (st == State::Armed) {
        if (s.state != BalanceState::Balancing || previousState == BalanceState::Balancing) return;
        beginRecording(s.timeUs);
        st = State::Recording;
    }
    if (st != State::Recording) return;
    if (skip) { skip--; return; }
    skip = every - 1;

    const uint32_t i = count;
    Sample &d = buffer[i];
    d.tUs = s.timeUs - startUs;
    d.tilt = fixed(s.tilt, 100);
    d.tiltRate = fixed(s.tiltRate, 10);
    d.setpoint = fixed(s.setpoint, 100);
    d.phi = fixed(s.phi, 10);
    d.phiDot = fixed(s.phiDot, 100);
    d.yawRate = fixed(s.yawRate, 10);
    d.wheelL = fixed(s.wheelL, 100);
    d.wheelR = fixed(s.wheelR, 100);
    d.lqr = fixed(s.lqr, 1000);
    d.curL = fixed(s.curL, 1000);
    d.curR = fixed(s.curR, 1000);
    d.joyX = fixed8(s.joyX, 100);
    d.joyY = fixed8(s.joyY, 100);
    d.state = (uint8_t)s.state;
    d.source = s.source;
    d.posErr = fixed(s.posErr, 100);
    d.speedRef = fixed(s.speedRef, 100);
    d.heading = fixed(s.heading, 10);
    d.headErr = fixed(s.headErr, 100);
    d.yawRateRef = fixed(s.yawRateRef, 10);
    d.yawCur = fixed(s.yawCur, 1000);
    d.pitch = fixed(s.pitch, 100);
    for (int k = 0; k < 3; k++) {
        d.gyro[k] = fixed(s.gyro[k], 10);
        d.accel[k] = fixed(s.accel[k], 10000);
    }
    d.lifted = s.lifted;
    d.hold = (uint8_t)lroundf(constrain(s.holdScale, 0.0f, 1.0f) * 100.0f);
    count = i + 1;
    if (i + 1 >= cap) current = State::Done;
}

// ---------------------------------------------------------------------------
// CSV output
// ---------------------------------------------------------------------------

CsvReader::CsvReader() { readers++; }
CsvReader::~CsvReader() { readers--; }

bool CsvReader::nextLine() {
    const State st = current;
    const uint32_t n = (st == State::Recording || st == State::Armed) ? 0 : count.load();
    char line[400];

    if (row < 0) {
        // Header block: everything needed to interpret the run.
        pending = "# Self-balancing robot capture\n";
        snprintf(line, sizeof(line), "# robot: %s, build: %s %s\n", robotName(), __DATE__, __TIME__);
        pending += line;
        snprintf(line, sizeof(line), "# rate_hz: %.0f (every %lu control step(s)), samples: %lu, trigger: %s, battery_v: %.2f\n",
                 rateAtStart, (unsigned long)every, (unsigned long)n, trigger == Trigger::Now ? "now" : "next-balance", batteryAtStart);
        pending += line;
        pending += "# controller: lqr = -(K0*pos_err + K1*(tilt-setpoint) + K3*(phi_dot-speed_ref) + K4*tilt_rate) [A, total]; "
                   "yaw_cur = turnGain*joy_x - (K2*head_err + K5*(yaw_rate-yaw_rate_ref)) [A]; "
                   "cur_l/cur_r = lqr/2 +- yaw_cur, each within +-maxCurrent (balance first), 0 unless balancing; "
                   "hold = share of the K0/K3 terms used (balPriority limits them to 1.5x the total current limit); "
                   "lifted = wheel detected off the ground / slipping (phi_dot then from the other wheel)\n";
        pending += "# units: deg, deg/s, rad, rad/s, A, g; tilt = roll from the IMU filter, setpoint = set point + learned trim, phi = wheel angle (avg of both), "
                   "heading/pos_err/head_err reset when balancing starts; gx..az = raw IMU body axes (x up, y forward, z along the axle)\n";
        size_t paramCount;
        const ParamDef *defs = Settings::params(paramCount);
        pending += "# settings:";
        for (size_t i = 0; i < paramCount && i < settingsAtStart.size(); i++) {
            snprintf(line, sizeof(line), " %s=%g", defs[i].key, settingsAtStart[i]);
            pending += line;
        }
        pending += " (at the start of the recording)\n";
        pending += "t_ms,state,tilt,tilt_rate,setpoint,phi,phi_dot,yaw_rate,wheel_l,wheel_r,lqr,cur_l,cur_r,joy_x,joy_y,"
                   "pos_err,speed_ref,heading,head_err,yaw_rate_ref,yaw_cur,pitch,gx,gy,gz,ax,ay,az,drive_src,hold,lifted\n";
        row = 0;
        return true;
    }
    if (row >= (int32_t)n) return false;
    const Sample &s = buffer[row++];
    char src[12];
    const uint8_t srcKind = s.source & 0x0F;
    if (srcKind == (uint8_t)DriveInput::Source::Module) snprintf(src, sizeof(src), "module%u", s.source >> 4);
    else snprintf(src, sizeof(src), "%s", DriveInput::sourceName((DriveInput::Source)srcKind));
    const int len = snprintf(line, sizeof(line),
                             "%.3f,%s,%.2f,%.1f,%.2f,%.1f,%.2f,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,"
                             "%.2f,%.2f,%.1f,%.2f,%.1f,%.3f,%.2f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%s,%.2f,%s\n",
                             s.tUs / 1000.0f, stateCsvName(s.state), s.tilt / 100.0f, s.tiltRate / 10.0f,
                             s.setpoint / 100.0f, s.phi / 10.0f, s.phiDot / 100.0f, s.yawRate / 10.0f,
                             s.wheelL / 100.0f, s.wheelR / 100.0f, s.lqr / 1000.0f, s.curL / 1000.0f,
                             s.curR / 1000.0f, s.joyX / 100.0f, s.joyY / 100.0f,
                             s.posErr / 100.0f, s.speedRef / 100.0f, s.heading / 10.0f, s.headErr / 100.0f,
                             s.yawRateRef / 10.0f, s.yawCur / 1000.0f, s.pitch / 100.0f,
                             s.gyro[0] / 10.0f, s.gyro[1] / 10.0f, s.gyro[2] / 10.0f,
                             s.accel[0] / 10000.0f, s.accel[1] / 10000.0f, s.accel[2] / 10000.0f, src,
                             s.hold / 100.0f, s.lifted == 1 ? "L" : s.lifted == 2 ? "R" : "-");
    pending = String(line, len);
    return true;
}

size_t CsvReader::read(uint8_t *buf, size_t maxLen) {
    size_t n = 0;
    while (n < maxLen) {
        if (pendingPos >= pending.length()) {
            pendingPos = 0;
            pending = "";
            if (!nextLine()) break;
        }
        const size_t take = min(maxLen - n, pending.length() - pendingPos);
        memcpy(buf + n, pending.c_str() + pendingPos, take);
        n += take;
        pendingPos += take;
    }
    return n;
}

} // namespace Capture
