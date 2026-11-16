#include "Capture.h"

#include <atomic>
#include <esp_heap_caps.h>
#include <vector>

#include "Battery.h"
#include "RobotParams.h"
#include "Settings.h"

#include "WebInterface.h"

namespace Capture {
namespace {

// 60 s at 500 Hz. At a higher control rate the maximum duration shrinks.
constexpr uint32_t MAX_SAMPLES = 30000;
// Without PSRAM (shouldn't happen on the ESP32-S3 Zero) fall back to this.
constexpr uint32_t FALLBACK_SAMPLES = 2500;

// Fixed point, 32 bytes per sample, so 30 000 samples are ~1 MB of PSRAM.
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
};
static_assert(sizeof(Sample) <= 32, "Sample should pack to 32 bytes");

Sample *buffer = nullptr;
uint32_t allocated = 0;
std::atomic<State> current{State::Idle};
std::atomic<uint32_t> count{0};
uint32_t cap = 0;
uint32_t startUs = 0;
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
        buffer = (Sample *)heap_caps_malloc(MAX_SAMPLES * sizeof(Sample), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        allocated = MAX_SAMPLES;
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
    rateAtStart = IMUsampleRate ? IMUsampleRate : 500;
    const uint32_t wanted = (uint32_t)constrain((int)seconds, 1, (int)CAPTURE_MAX_SECONDS) * (uint32_t)rateAtStart;
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

    const uint32_t i = count;
    buffer[i] = {
        s.timeUs - startUs,
        fixed(s.tilt, 100), fixed(s.tiltRate, 10), fixed(s.setpoint, 100),
        fixed(s.phi, 10), fixed(s.phiDot, 100), fixed(s.yawRate, 10),
        fixed(s.wheelL, 100), fixed(s.wheelR, 100),
        fixed(s.lqr, 1000), fixed(s.curL, 1000), fixed(s.curR, 1000),
        fixed8(s.joyX, 100), fixed8(s.joyY, 100),
        (uint8_t)s.state,
    };
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
    char line[200];

    if (row < 0) {
        // Header block: everything needed to interpret the run.
        pending = "# Self-balancing robot capture\n";
        snprintf(line, sizeof(line), "# robot: %s, build: %s %s\n", robotName(), __DATE__, __TIME__);
        pending += line;
        snprintf(line, sizeof(line), "# rate_hz: %.0f (one sample per control step), samples: %lu, trigger: %s, battery_v: %.2f\n",
                 rateAtStart, (unsigned long)n, trigger == Trigger::Now ? "now" : "next-balance", batteryAtStart);
        pending += line;
        pending += "# controller: lqr = -(K1*(tilt-setpoint) + K3*phi_dot + K4*tilt_rate) [A, total]; "
                   "cur_l/cur_r = lqr/2 +- joystick terms, clamped to +-1 A, 0 unless balancing\n";
        pending += "# units: deg, deg/s, rad, rad/s, A; tilt = roll from the IMU filter, phi = wheel angle (avg of both)\n";
        size_t paramCount;
        const ParamDef *defs = Settings::params(paramCount);
        pending += "# settings:";
        for (size_t i = 0; i < paramCount && i < settingsAtStart.size(); i++) {
            snprintf(line, sizeof(line), " %s=%g", defs[i].key, settingsAtStart[i]);
            pending += line;
        }
        pending += " (at the start of the recording)\n";
        pending += "t_ms,state,tilt,tilt_rate,setpoint,phi,phi_dot,yaw_rate,wheel_l,wheel_r,lqr,cur_l,cur_r,joy_x,joy_y\n";
        row = 0;
        return true;
    }
    if (row >= (int32_t)n) return false;
    const Sample &s = buffer[row++];
    const int len = snprintf(line, sizeof(line),
                             "%.3f,%s,%.2f,%.1f,%.2f,%.1f,%.2f,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f\n",
                             s.tUs / 1000.0f, stateCsvName(s.state), s.tilt / 100.0f, s.tiltRate / 10.0f,
                             s.setpoint / 100.0f, s.phi / 10.0f, s.phiDot / 100.0f, s.yawRate / 10.0f,
                             s.wheelL / 100.0f, s.wheelR / 100.0f, s.lqr / 1000.0f, s.curL / 1000.0f,
                             s.curR / 1000.0f, s.joyX / 100.0f, s.joyY / 100.0f);
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
