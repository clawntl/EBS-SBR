#include "Battery.h"

#include <atomic>

#include "Config.h"
#include "DeviceLog.h"
#include "MotorDrivers.h"

namespace {

// Smoothing (per 1 s reading): ~5 s for the lockout so a brief sag while the
// motors work hard doesn't trip it, ~30 s for the charge estimate.
constexpr float FAST_ALPHA = 0.2f;
constexpr float SLOW_ALPHA = 1.0f / 30.0f;

// Runtime estimate: the level is logged every HISTORY_INTERVAL_MS and the
// discharge rate is a least-squares fit over the last HISTORY_LEN entries
// (10 minutes). Needs MIN_HISTORY entries (2 minutes) before estimating.
constexpr uint32_t HISTORY_INTERVAL_MS = 10000;
constexpr uint8_t HISTORY_LEN = 60;
constexpr uint8_t MIN_HISTORY = 12;
// Ignore slopes flatter than this [%/min]: idle or on a bench supply.
constexpr float MIN_DISCHARGE_RATE = 0.02f;
// A jump up of this much means charging or a fresh battery: start over.
constexpr float RESET_RISE_PERCENT = 3.0f;

// Typical Li-ion (NMC 18650) cell voltage vs state of charge at light load.
// Per cell; the pack voltage is divided by BATTERY_CELLS first.
struct CurvePoint { float volts; float percent; };
const CurvePoint CURVE[] = {
    {3.00f, 0},  {3.30f, 5},  {3.45f, 10}, {3.55f, 20}, {3.62f, 30}, {3.68f, 40},
    {3.74f, 50}, {3.80f, 60}, {3.87f, 70}, {3.95f, 80}, {4.06f, 90}, {4.20f, 100},
};

std::atomic<float> fastVoltage{0.0f};
std::atomic<float> leftV{0.0f}, rightV{0.0f};
std::atomic<uint8_t> level{0};
std::atomic<int16_t> minutesLeft{-1};
std::atomic<bool> low{false};
std::atomic<bool> haveReading{false};

float slowVoltage = 0;
float levelExact = 0;
uint32_t lastCheckMs = 0;
bool firstReading = true;

float history[HISTORY_LEN];
uint8_t historyCount = 0, historyHead = 0;
uint32_t lastHistoryMs = 0;

float percentFromVoltage(float packVolts) {
    const float v = packVolts / BATTERY_CELLS;
    constexpr size_t n = sizeof(CURVE) / sizeof(CURVE[0]);
    if (v <= CURVE[0].volts) return 0;
    if (v >= CURVE[n - 1].volts) return 100;
    for (size_t i = 1; i < n; i++) {
        if (v < CURVE[i].volts) {
            const CurvePoint &a = CURVE[i - 1], &b = CURVE[i];
            return a.percent + (v - a.volts) / (b.volts - a.volts) * (b.percent - a.percent);
        }
    }
    return 100;
}

void clearHistory() { historyCount = historyHead = 0; }

void updateEstimate(uint32_t now) {
    if (historyCount && now - lastHistoryMs < HISTORY_INTERVAL_MS) return;
    lastHistoryMs = now;

    // Charging or battery swapped: the old trend is meaningless.
    if (historyCount) {
        const float newest = history[(historyHead + HISTORY_LEN - 1) % HISTORY_LEN];
        if (levelExact > newest + RESET_RISE_PERCENT) clearHistory();
    }

    history[historyHead] = levelExact;
    historyHead = (historyHead + 1) % HISTORY_LEN;
    if (historyCount < HISTORY_LEN) historyCount++;

    if (historyCount < MIN_HISTORY) {
        minutesLeft = -1;
        return;
    }
    // Least-squares slope of level vs time, oldest sample at x = 0.
    const uint8_t oldest = (historyHead + HISTORY_LEN - historyCount) % HISTORY_LEN;
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (uint8_t i = 0; i < historyCount; i++) {
        const float x = i * (HISTORY_INTERVAL_MS / 60000.0f); // minutes
        const float y = history[(oldest + i) % HISTORY_LEN];
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    const float n = historyCount;
    const float slope = (n * sxy - sx * sy) / (n * sxx - sx * sx); // %/min
    if (slope > -MIN_DISCHARGE_RATE) {
        minutesLeft = -2;
        return;
    }
    minutesLeft = (int16_t)constrain(levelExact / -slope, 0.0f, 999.0f);
}

} // namespace

namespace Battery {

bool update(float cutoffVolts) {
    const uint32_t now = millis();
    if (now - lastCheckMs < BATTERY_CHECK_INTERVAL_MS) return false;
    lastCheckMs = now;

    const float l = getDriverSupplyVoltage(MOTOR_LEFT, BATTERY_READING_MAX_AGE_MS);
    const float r = getDriverSupplyVoltage(MOTOR_RIGHT, BATTERY_READING_MAX_AGE_MS);
    leftV = l;
    rightV = r;
    const int n = (l > 0) + (r > 0);
    if (n == 0) {
        // Drivers off, offline or on old firmware: nothing to go on. Don't
        // hold the robot off on a stale reading; the next reading restarts
        // the smoothing.
        if (haveReading) DeviceLog::println("[battery] no voltage reading from the motor drivers");
        haveReading = false;
        firstReading = true;
        fastVoltage = 0;
        low = false;
        minutesLeft = -1;
        clearHistory();
        return false;
    }
    const float v = (l + r) / n;

    float fast = fastVoltage;
    if (firstReading) {
        fast = slowVoltage = v;
    } else {
        fast += FAST_ALPHA * (v - fast);
        slowVoltage += SLOW_ALPHA * (v - slowVoltage);
    }
    fastVoltage = fast;
    haveReading = true;

    levelExact = percentFromVoltage(slowVoltage);
    level = (uint8_t)roundf(levelExact);
    updateEstimate(now);

    const bool wasLow = low;
    bool isLowNow = wasLow;
    if (cutoffVolts <= 0) {
        isLowNow = false; // lockout disabled
    } else if (!wasLow && fast < cutoffVolts) {
        isLowNow = true;
    } else if (wasLow && fast >= cutoffVolts + BATTERY_HYSTERESIS_V) {
        isLowNow = false;
    }
    low = isLowNow;

    if (isLowNow != wasLow || firstReading) {
        DeviceLog::printf("[battery] %.2f V (L %.2f, R %.2f), %u%%%s\n", fast, l, r, (unsigned)level,
                          isLowNow ? " - LOW, balancing stopped" : (wasLow ? " - recovered" : ""));
    }
    firstReading = false;
    return true;
}

bool known() { return haveReading; }
float voltage() { return fastVoltage; }
float leftVoltage() { return leftV; }
float rightVoltage() { return rightV; }
uint8_t percent() { return level; }
int16_t minutesRemaining() { return minutesLeft; }
bool isLow() { return low; }

} // namespace Battery
