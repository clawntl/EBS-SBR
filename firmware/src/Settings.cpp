#include "Settings.h"

#include <Preferences.h>
#include <math.h>
#include <string.h>

#include "Config.h"
#include "RobotParams.h"

namespace {

// Single source of truth for every tunable parameter: defaults, ranges,
// labels, grouping and the help text shown on the web page. The page builds
// its controls from this table (sent as a "schema" message), so adding a
// parameter only needs an entry here plus wherever the firmware uses it.
//
// Defaults are the values that were hard-coded in RobotParams.cpp.
const ParamDef kParams[] = {
    // ---- Balance gains (LQR) -------------------------------------------------
    {"K1", "Tilt angle (K1)", "Balance gains (LQR)", -10.0f, -30.0f, 30.0f, 0.01f, false,
     "How hard the wheels push back when the robot leans away from the angle set point: the main "
     "'stiffness'. More negative pushes harder; too much makes it oscillate quickly or buzz, too "
     "little and it can't catch itself.",
     [](float v) { K[1] = v; }, 0},
    {"K4", "Tilt rate (K4)", "Balance gains (LQR)", -0.6f, -3.0f, 3.0f, 0.001f, false,
     "Damping: how hard the wheels react to how fast the robot is tipping (the gyro rate). Too "
     "little and it overshoots and wobbles; too much makes it twitchy and noisy, because it "
     "amplifies gyro noise.",
     [](float v) { K[4] = v; }, 0},
    {"K3", "Wheel speed (K3)", "Balance gains (LQR)", -0.05f, -2.0f, 2.0f, 0.001f, false,
     "Feedback on the wheels' own speed (the average reported by both motor drivers). Stops the "
     "robot rolling away at a steady speed; too much fights the balancing and makes it slow and "
     "swaying.",
     [](float v) { K[3] = v; }, 0},
    // ---- Balance ----------------------------------------------------------------
    {"setpoint", "Angle set point (deg)", "Balance", 2.0f, -20.0f, 20.0f, 0.01f, false,
     "The tilt reading at which the robot is exactly balanced. It isn't 0 because the IMU isn't "
     "mounted perfectly level and the weight isn't exactly over the axle. If it balances but keeps "
     "creeping in one direction, nudge this until it stays put; use the fine-adjust slider for "
     "small steps.",
     [](float v) { angleSetPoint = v * DEG_TO_RAD; }, PARAM_FLAG_FINE},
    {"deactAngle", "Cut-out angle (deg)", "Balance", 42.0f, 0.0f, 90.0f, 1.0f, false,
     "If the robot tilts more than this from upright it has fallen over, and the motors are set to "
     "0 A. Balancing starts again by itself once it's stood back up within this angle.",
     [](float v) { deactivationAngle = v; }, 0},
    {"alpha", "Gyro trust (filter alpha)", "Balance", 0.998f, 0.9f, 1.0f, 0.0001f, false,
     "The tilt angle combines the gyro (smooth and fast, but drifts) with the accelerometer (no "
     "drift, but upset by movement and vibration). Higher trusts the gyro more. At 500 Hz, 0.998 "
     "averages the accelerometer over about 1 s; the same alpha at a higher control rate averages "
     "over a shorter time.",
     [](float v) { alpha = v; }, 0},
    {"imuRate", "Control rate (Hz)", "Balance", 500.0f, 200.0f, 1000.0f, 50.0f, false,
     "How often the IMU is read, the controller runs and the motor currents are sent over CAN. "
     "Applies immediately. The period is a whole number of milliseconds, so some rates round "
     "(e.g. 300 Hz runs at 333 Hz); the Tune tab shows the real loop rate.",
     [](float v) { IMUsampleRate = (uint16_t)lroundf(v); }, 0},

    // ---- Position & heading hold ---------------------------------------------------
    {"K0", "Wheel position (K0)", "Position & heading hold", 0.0f, -2.0f, 2.0f, 0.001f, false,
     "Holds the robot's position: current per radian of wheel rotation away from where it should "
     "be (where it started balancing, moved along by the drive input). Same sign as K3. 0 = off "
     "(it then only resists speed, so it creeps). Start small (e.g. -0.005) and increase until it "
     "returns to its spot after a nudge without a slow sway; too much makes it rock back and "
     "forth.",
     [](float v) { K[0] = v; }, 0},
    {"K2", "Heading (K2)", "Position & heading hold", 0.0f, -2.0f, 2.0f, 0.01f, false,
     "Holds the heading: differential current [A] per radian the robot has turned away from the "
     "heading it should have. Stops the slow spin from left/right motor differences. Needs 'Yaw "
     "direction' set correctly first; positive values hold, negative ones push it further round. "
     "0 = off. Start around 0.05.",
     [](float v) { K[2] = v; }, 0},
    {"K5", "Turn rate (K5)", "Position & heading hold", 0.0f, -1.0f, 1.0f, 0.001f, false,
     "Damps turning: differential current [A] per rad/s of yaw rate away from the requested turn "
     "rate. Positive damps; too much buzzes or wobbles in yaw. 0 = off. Start around 0.02, then "
     "add K2.",
     [](float v) { K[5] = v; }, 0},
    {"yawInvert", "Yaw direction flip (0/1)", "Position & heading hold", 0.0f, 0.0f, 1.0f, 1.0f, false,
     "Matches the gyro's yaw sign to the wheels. Check it with K2 and K5 at 0: turn right with "
     "the joystick and watch 'Yaw rate' on the chart. It should go positive; if it goes negative "
     "set this to 1. With it wrong the heading hold makes the robot spin.",
     [](float v) { yawInvert = v > 0.5f; }, 0},
    {"maxPosErr", "Max position error (rad)", "Position & heading hold", 10.0f, 0.5f, 100.0f, 0.5f, false,
     "The most wheel rotation (rad) the position hold acts on. If the robot is pushed further than "
     "this the target moves with it, so it doesn't run back hard. One wheel revolution is 6.28 "
     "rad.",
     [](float v) { maxPositionError = v; }, 0},
    {"maxHeadErr", "Max heading error (deg)", "Position & heading hold", 30.0f, 1.0f, 180.0f, 1.0f, false,
     "The most heading error the heading hold acts on. Turn the robot further than this by hand "
     "and the target heading turns with it.",
     [](float v) { maxHeadingError = v * DEG_TO_RAD; }, 0},

    // ---- Driving ------------------------------------------------------------------
    {"maxSpeed", "Top speed (wheel rad/s)", "Driving", 10.0f, 0.0f, 40.0f, 0.5f, false,
     "Wheel speed requested at full forward/back on the joystick or gamepad. The controller leans "
     "the robot to reach it, so too much can tip it past the cut-out angle.",
     [](float v) { maxWheelSpeed = v; }, 0},
    {"maxAccel", "Acceleration (wheel rad/s²)", "Driving", 20.0f, 1.0f, 200.0f, 1.0f, false,
     "How quickly the requested speed may change. Lower is gentler starts and stops; very high "
     "makes it lurch (it has to roll backwards briefly to lean into a start).",
     [](float v) { maxWheelAccel = v; }, 0},
    {"maxTurn", "Turn rate (deg/s)", "Driving", 90.0f, 0.0f, 360.0f, 5.0f, false,
     "Turn rate requested at full left/right.",
     [](float v) { maxTurnRate = v * DEG_TO_RAD; }, 0},
    {"turnGain", "Turn feed-forward (A)", "Driving", 0.15f, 0.0f, 1.0f, 0.01f, false,
     "Differential current added straight away at full left/right (taken from one wheel, added "
     "to the other), so turns start without waiting for the turn-rate error to build. The "
     "heading-hold gains (K2, K5) then correct it to the requested rate. With K2 and K5 at 0 this "
     "is the only turning current, as before.",
     [](float v) { turnFeedForward = v; }, 0},

    // ---- System ---------------------------------------------------------------------
    {"battCutoff", "Battery cutoff (V, 0 = off)", "System", 9.6f, 0.0f, 12.6f, 0.1f, false,
     "Below this pack voltage (average of the two motor drivers' readings, smoothed over about "
     "5 s) the robot stops balancing to protect the 3S Li-ion pack. It's allowed again once the "
     "voltage has recovered to 0.3 V above the cutoff. 9.6 V is 3.2 V per cell. 0 disables the "
     "cutoff.",
     [](float) {}, 0},
    {"recEvery", "Record every Nth step", "System", 1.0f, 1.0f, 20.0f, 1.0f, false,
     "Recordings normally keep every control step (up to about 60 s at 500 Hz, 30 s at 1000 Hz). "
     "2 keeps every other step and doubles the longest recording, and so on. Applies to the next "
     "recording.",
     [](float) {}, 0},
    {"telemetryMs", "Telemetry period (ms)", "System", 50.0f, 20.0f, 1000.0f, 10.0f, false,
     "How often live data is sent to this page. For a detailed log of every control step use "
     "Record instead.",
     [](float) {}, 0},
};
constexpr size_t kParamCount = sizeof(kParams) / sizeof(kParams[0]);

constexpr const char *NVS_NAMESPACE = "sbr";

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
float current[kParamCount];
float saved[kParamCount];
volatile uint32_t changeCounter = 0;

int indexOf(const char *key) {
    if (!key) return -1;
    for (size_t i = 0; i < kParamCount; i++) {
        if (!strcmp(kParams[i].key, key)) return (int)i;
    }
    return -1;
}

float clampToParam(const ParamDef &p, float v) {
    return v < p.min ? p.min : (v > p.max ? p.max : v);
}

// Replaces every value (and optionally the saved copy) and applies them.
void replaceAll(const float *values, bool alsoSaved) {
    portENTER_CRITICAL(&lock);
    for (size_t i = 0; i < kParamCount; i++) {
        current[i] = values[i];
        if (alsoSaved) saved[i] = values[i];
    }
    changeCounter = changeCounter + 1;
    portEXIT_CRITICAL(&lock);
    for (size_t i = 0; i < kParamCount; i++) kParams[i].apply(values[i]);
}

} // namespace

namespace Settings {

const ParamDef *params(size_t &count) {
    count = kParamCount;
    return kParams;
}

const ParamDef *find(const char *key) {
    const int i = indexOf(key);
    return i < 0 ? nullptr : &kParams[i];
}

float value(const char *key) {
    const int i = indexOf(key);
    return i < 0 ? NAN : valueAt((size_t)i);
}

float valueAt(size_t index) {
    if (index >= kParamCount) return NAN;
    portENTER_CRITICAL(&lock);
    const float v = current[index];
    portEXIT_CRITICAL(&lock);
    return v;
}

uint32_t version() { return changeCounter; }

bool set(const char *key, float v, float *applied) {
    const int i = indexOf(key);
    if (i < 0 || !isfinite(v)) return false;
    const ParamDef &p = kParams[i];
    v = clampToParam(p, v);

    portENTER_CRITICAL(&lock);
    current[i] = v;
    changeCounter = changeCounter + 1;
    portEXIT_CRITICAL(&lock);
    p.apply(v);

    if (applied) *applied = v;
    return true;
}

void resetToDefaults() {
    float values[kParamCount];
    for (size_t i = 0; i < kParamCount; i++) values[i] = kParams[i].defaultValue;
    replaceAll(values, false);
}

void load() {
    // Each parameter is stored under its own key, so adding or removing a
    // parameter in a future version keeps the others (new ones get their
    // default) instead of invalidating the whole saved set.
    float values[kParamCount];
    for (size_t i = 0; i < kParamCount; i++) values[i] = kParams[i].defaultValue;
    Preferences prefs;
    // Read-only begin() fails until the namespace has been written once;
    // that's the normal first-boot case and the defaults apply.
    if (prefs.begin(NVS_NAMESPACE, true)) {
        for (size_t i = 0; i < kParamCount; i++) {
            const ParamDef &p = kParams[i];
            const float v = prefs.getFloat(p.key, p.defaultValue);
            values[i] = isfinite(v) ? clampToParam(p, v) : p.defaultValue;
        }
        prefs.end();
    }
    replaceAll(values, true);
}

bool save() {
    float values[kParamCount];
    portENTER_CRITICAL(&lock);
    memcpy(values, current, sizeof(values));
    portEXIT_CRITICAL(&lock);

    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return false;
    bool ok = true;
    for (size_t i = 0; i < kParamCount; i++) {
        ok &= prefs.putFloat(kParams[i].key, values[i]) == sizeof(float);
    }
    prefs.end();
    if (ok) {
        portENTER_CRITICAL(&lock);
        memcpy(saved, values, sizeof(saved));
        changeCounter = changeCounter + 1; // dirty flag changed
        portEXIT_CRITICAL(&lock);
    }
    return ok;
}

bool isDirty() {
    portENTER_CRITICAL(&lock);
    const bool dirty = memcmp(current, saved, sizeof(current)) != 0;
    portEXIT_CRITICAL(&lock);
    return dirty;
}

} // namespace Settings
