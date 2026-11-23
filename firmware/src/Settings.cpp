#include "Settings.h"

#include <Preferences.h>
#include <math.h>
#include <string.h>

#include "Config.h"
#include "DriveInput.h"
#include "Gamepad.h"
#include "MotorDrivers.h"
#include "RobotParams.h"

namespace {

// Single source of truth for every tunable parameter: defaults, ranges,
// labels, grouping and the help text shown on the web page. The page builds
// its controls from this table (sent as a "schema" message), so adding a
// parameter only needs an entry here plus wherever the firmware uses it.
//
// Defaults: the tune from the 2026-11-16 tests (gains, set point, hold and turn);
// the rest are the values that were hard-coded in RobotParams.cpp.
const ParamDef kParams[] = {
    // ---- Balance gains (LQR) -------------------------------------------------
    {"K1", "Tilt angle (K1)", "Balance gains (LQR)", -20.0f, -30.0f, 30.0f, 0.01f, false,
     "How hard the wheels push back when the robot leans away from the angle set point: the main "
     "'stiffness'. More negative pushes harder; too much makes it oscillate quickly or buzz, too "
     "little and it can't catch itself.",
     [](float v) { K[1] = v; }, 0},
    {"K4", "Tilt rate (K4)", "Balance gains (LQR)", -2.0f, -3.0f, 3.0f, 0.001f, false,
     "Damping: how hard the wheels react to how fast the robot is tipping (the gyro rate). Too "
     "little and it overshoots and wobbles; too much makes it twitchy and noisy, because it "
     "amplifies gyro noise.",
     [](float v) { K[4] = v; }, 0},
    {"K3", "Wheel speed (K3)", "Balance gains (LQR)", -0.24f, -2.0f, 2.0f, 0.001f, false,
     "Feedback on the wheels' own speed (the average reported by both motor drivers). Stops the "
     "robot rolling away at a steady speed; too much fights the balancing and makes it slow and "
     "swaying.",
     [](float v) { K[3] = v; }, 0},
    // ---- Balance ----------------------------------------------------------------
    {"setpoint", "Angle set point (deg)", "Balance", 1.5f, -20.0f, 20.0f, 0.01f, false,
     "The tilt reading at which the robot is exactly balanced. It isn't 0 because the IMU isn't "
     "mounted perfectly level and the weight isn't exactly over the axle. If it balances but keeps "
     "creeping in one direction, nudge this until it stays put; use the fine-adjust slider for "
     "small steps.",
     [](float v) { angleSetPoint = v * DEG_TO_RAD; }, PARAM_FLAG_FINE},
    {"learnTrim", "Learn set point trim (0/1)", "Balance", 1.0f, 0.0f, 1.0f, 1.0f, false,
     "When on, the robot works out the exact balance angle itself while it stands still (no "
     "drive input, not being pushed): if it keeps leaning one way, the trim below creeps towards "
     "that lean, so the position hold doesn't have to keep pulling against it. It takes about "
     "half a minute to settle and follows slow changes (battery position, a slope). The learned "
     "trim saves itself once the robot stops balancing.",
     [](float v) { learnTrim = v > 0.5f; }, 0},
    {"spTrim", "Set point trim (deg)", "Balance", 0.0f, -5.0f, 5.0f, 0.01f, false,
     "Learned correction added to the angle set point (shown live as the set point on the "
     "charts). Set it to 0 to start learning again from scratch, e.g. after moving the battery.",
     [](float v) { setPointTrim = v * DEG_TO_RAD; }, 0},
    {"deactAngle", "Cut-out angle (deg)", "Balance", 42.0f, 0.0f, 90.0f, 1.0f, false,
     "If the robot tilts more than this from upright it has fallen over, and the motors are set to "
     "0 A. To start again, stand it up within the start angle below and hold it still.",
     [](float v) { deactivationAngle = v; }, 0},
    {"maxCurrent", "Current limit (A per wheel)", "Balance", 1.5f, 0.5f, 2.0f, 0.05f, false,
     "The most current the controller asks of each motor (balancing and turning together; balancing "
     "comes first). More gives more torque to catch a bump or a big lean, but it heats the motors "
     "and draws harder on the battery: the gimbal motors take short peaks well, but sustained "
     "high current makes them hot. The drivers themselves limit at 2 A. Was a fixed 1 A.",
     [](float v) { maxWheelCurrent = v; }, 0},
    {"engageAngle", "Start angle (deg)", "Balance", 10.0f, 2.0f, 40.0f, 1.0f, false,
     "Balancing only starts once the robot is within this angle of the set point AND held still "
     "(not swinging, wheels stopped) for 0.3 s. Then it keeps going until it passes the cut-out "
     "angle. Stops it starting mid-lift with the wheels in the air, which spins them up to full "
     "speed before it touches the ground.",
     [](float v) { engageAngle = v; }, 0},
    {"runawaySpeed", "Runaway wheel speed (rad/s)", "Balance", 60.0f, 20.0f, 200.0f, 5.0f, false,
     "If the wheels spin faster than this for 0.3 s while balancing (it has been lifted, or a crash "
     "left the wheels spinning freely) the motors stop until it's stood up and held still again. "
     "Never lower than 1.5x the top speed, whatever this says.",
     [](float v) { runawaySpeed = v; }, 0},
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
    {"balPriority", "Balance first at the current limit (0/1)", "Balance", 1.0f, 0.0f, 1.0f, 1.0f, false,
     "After a hard push the tilt terms (K1, K4) and the position/speed terms (K0, K3) can ask for "
     "current in opposite directions, and their sum can push the wrong way just as the motors run "
     "out of current: after a hard push the spinning wheels can make the speed term ask for 10 A "
     "or more. With this on, the position/speed terms are limited to 1.5x the total current limit "
     "(both wheels), so once the tilt terms ask for more, catching the fall comes first and getting "
     "back to its spot second. Ordinary pushes and driving rarely reach the limit.",
     [](float v) { balancePriority = v > 0.5f; }, 0},
    {"liftDetect", "Lifted wheel detection (0/1)", "Balance", 1.0f, 0.0f, 1.0f, 1.0f, false,
     "Going over a bump, one wheel can leave the ground and spin up, which makes the robot think "
     "it's rolling fast and react wrongly. With this on, when the two wheels' speeds differ by "
     "more than turning explains (Lift threshold), the controller uses the slower, grounded "
     "wheel's speed until they agree again. Uses the wheel radius and track below.",
     [](float v) { liftDetect = v > 0.5f; }, 0},
    {"liftSpeed", "Lift threshold (rad/s)", "Balance", 8.0f, 3.0f, 50.0f, 0.5f, false,
     "How far the wheels' speed difference may be from what the measured turn rate explains "
     "before a wheel counts as lifted or slipping. Normal driving and turning stay within about "
     "2-5 rad/s; a wheel off the ground passes 30 rad/s within 50 ms.",
     [](float v) { liftSpeed = v; }, 0},
    {"wheelDia", "Wheel diameter (mm)", "Balance", 60.0f, 20.0f, 300.0f, 0.5f, false,
     "Outside diameter of the tyres. Used by Lifted wheel detection to work out how much the "
     "wheel speeds should differ in a turn.",
     [](float v) { wheelRadius = v / 2000.0f; }, 0},
    {"trackWidth", "Track width (mm)", "Balance", 110.0f, 30.0f, 500.0f, 0.5f, false,
     "Distance between the two tyres' centre lines. Used by Lifted wheel detection.",
     [](float v) { trackWidth = v / 1000.0f; }, 0},

    // ---- Position & heading hold ---------------------------------------------------
    {"K0", "Wheel position (K0)", "Position & heading hold", -0.23f, -2.0f, 2.0f, 0.001f, false,
     "Holds the robot's position: current per radian of wheel rotation away from where it should "
     "be (where it started balancing, moved along by the drive input). Same sign as K3. 0 = off "
     "(it then only resists speed, so it creeps). Start small (e.g. -0.005) and increase until it "
     "returns to its spot after a nudge without a slow sway; too much makes it rock back and "
     "forth.",
     [](float v) { K[0] = v; }, 0},
    {"K2", "Heading (K2)", "Position & heading hold", 0.3f, -2.0f, 2.0f, 0.01f, false,
     "Holds the heading: differential current [A] per radian the robot has turned away from the "
     "heading it should have. Stops the slow spin from left/right motor differences. Needs 'Yaw "
     "direction' set correctly first; positive values hold, negative ones push it further round. "
     "0 = off. Start around 0.05.",
     [](float v) { K[2] = v; }, 0},
    {"K5", "Turn rate (K5)", "Position & heading hold", 0.2f, -1.0f, 1.0f, 0.001f, false,
     "Damps turning: differential current [A] per rad/s of yaw rate away from the requested turn "
     "rate. Positive damps; too much buzzes or wobbles in yaw. 0 = off. Start around 0.02, then "
     "add K2.",
     [](float v) { K[5] = v; }, 0},
    {"yawInvert", "Yaw direction flip (0/1)", "Position & heading hold", 0.0f, 0.0f, 1.0f, 1.0f, false,
     "Matches the gyro's yaw sign to the wheels. Check it with K2 and K5 at 0: turn right with "
     "the joystick and watch 'Yaw rate' on the chart. It should go positive; if it goes negative "
     "set this to 1. With it wrong the heading hold makes the robot spin.",
     [](float v) { yawInvert = v > 0.5f; }, 0},
    {"maxPosErr", "Max position error (rad)", "Position & heading hold", 2.0f, 0.5f, 100.0f, 0.5f, false,
     "The most wheel rotation (rad) the position hold acts on. If the robot is pushed further than "
     "this the target moves with it, so it doesn't run back hard. One wheel revolution is 6.28 "
     "rad.",
     [](float v) { maxPositionError = v; }, 0},
    {"maxHeadErr", "Max heading error (deg)", "Position & heading hold", 30.0f, 1.0f, 180.0f, 1.0f, false,
     "The most heading error the heading hold acts on. Turn the robot further than this by hand "
     "and the target heading turns with it.",
     [](float v) { maxHeadingError = v * DEG_TO_RAD; }, 0},

    // ---- Driving ------------------------------------------------------------------
    {"maxSpeed", "Top speed (wheel rad/s)", "Driving", 40.0f, 0.0f, 40.0f, 0.5f, false,
     "Wheel speed requested at full forward/back on the joystick or gamepad. The controller leans "
     "the robot to reach it, so too much can tip it past the cut-out angle.",
     [](float v) { maxWheelSpeed = v; }, 0},
    {"maxAccel", "Acceleration (wheel rad/s²)", "Driving", 35.0f, 1.0f, 200.0f, 1.0f, false,
     "How quickly the requested speed may change. Lower is gentler starts and stops; very high "
     "makes it lurch (it has to roll backwards briefly to lean into a start).",
     [](float v) { maxWheelAccel = v; }, 0},
    {"speedLead", "Max speed lead (wheel rad/s)", "Driving", 8.0f, 0.0f, 50.0f, 0.5f, false,
     "How far the requested speed may run ahead of the wheels' actual speed while speeding up. On "
     "rough ground the robot can't keep up with the ramp; without a limit the growing speed error "
     "makes it lean further and further forward until a bump tips it over. With a limit the "
     "request waits for the wheels. No effect where the wheels keep up. 0 = no limit.",
     [](float v) { maxSpeedLead = v; }, 0},
    {"maxTurn", "Turn rate (deg/s)", "Driving", 120.0f, 0.0f, 360.0f, 5.0f, false,
     "Turn rate requested at full left/right.",
     [](float v) { maxTurnRate = v * DEG_TO_RAD; }, 0},
    {"modules", "Allow CAN modules (0/1)", "Driving", 1.0f, 0.0f, 1.0f, 1.0f, false,
     "Let modules on the CAN bus (e.g. a line follower or a Raspberry Pi) drive the robot. The web "
     "joystick and the Xbox controller always take over while their stick is off-centre; a module "
     "drives when neither is in use. See docs/can-modules.md for the CAN frames.",
     [](float v) { DriveInput::modulesAllowed = v > 0.5f; }, 0},
    {"turnGain", "Turn feed-forward (A)", "Driving", 0.1f, 0.0f, 1.0f, 0.01f, false,
     "Differential current added straight away at full left/right (taken from one wheel, added "
     "to the other), so turns start without waiting for the turn-rate error to build. The "
     "heading-hold gains (K2, K5) then correct it to the requested rate. With K2 and K5 at 0 this "
     "is the only turning current, as before.",
     [](float v) { turnFeedForward = v; }, 0},

    // ---- Xbox controller ---------------------------------------------------------------
    {"gpEnable", "Xbox controller over BLE (0/1)", "Xbox controller", 1.0f, 0.0f, 1.0f, 1.0f, false,
     "Pair an Xbox controller directly with the robot (Control tab). Off disconnects the controller "
     "and stops all Bluetooth radio use, leaving the radio to Wi-Fi (applies straight away).",
     [](float v) { Gamepad::enabled = v > 0.5f; }, 0},
    {"gpLayout", "Stick layout (0/1)", "Xbox controller", 1.0f, 0.0f, 1.0f, 1.0f, false,
     "0: left stick does both (forward/back and turning). 1: left stick forward/back, right stick "
     "turning, like an RC car.",
     [](float v) { Gamepad::layout = v > 0.5f ? 1 : 0; }, 0},
    {"gpDeadzone", "Stick dead zone", "Xbox controller", 0.12f, 0.0f, 0.5f, 0.01f, false,
     "Stick movement ignored around the centre (fraction of full travel), so a stick that doesn't "
     "centre exactly doesn't creep the robot or hold off a CAN module.",
     [](float v) { Gamepad::deadzone = v; }, 0},

    // ---- System ---------------------------------------------------------------------
    {"battCutoff", "Battery cutoff (V, 0 = off)", "System", 9.6f, 0.0f, 12.6f, 0.1f, false,
     "Below this pack voltage (average of the two motor drivers' readings, smoothed over about "
     "5 s) the robot stops balancing to protect the 3S Li-ion pack. It's allowed again once the "
     "voltage has recovered to 0.3 V above the cutoff. 9.6 V is 3.2 V per cell. 0 disables the "
     "cutoff.",
     [](float) {}, 0},
    {"leftNode", "Left driver node", "Motor drivers", 1.0f, 1.0f, 15.0f, 1.0f, false,
     "CAN node ID of the left wheel's driver board. Set on the Drivers tab.",
     [](float v) { leftNodeSetting = (uint8_t)lroundf(v); }, PARAM_FLAG_HIDDEN},
    {"rightNode", "Right driver node", "Motor drivers", 2.0f, 1.0f, 15.0f, 1.0f, false,
     "CAN node ID of the right wheel's driver board. Set on the Drivers tab.",
     [](float v) { rightNodeSetting = (uint8_t)lroundf(v); }, PARAM_FLAG_HIDDEN},
    {"wheelTlmMs", "Wheel speed reports (ms)", "System", 2.0f, 1.0f, 20.0f, 1.0f, false,
     "How often each motor driver reports its wheel speed. The driver firmware's own default is 10 ms "
     "(100 Hz), so the wheel-speed and position terms only saw a new value every 5th control step; "
     "2 ms matches the 500 Hz control rate. The CAN bus is then about 20% busy. Needs driver "
     "firmware 1.1 or later (older firmware keeps 10 ms); applies when a driver (re)starts or this "
     "changes.",
     [](float v) { wheelTelemetryMs = (uint8_t)lroundf(v); }, 0},
    {"recSecs", "Recording length (s)", "System", 10.0f, 1.0f, 60.0f, 1.0f, false,
     "Length of a recording: the Record panel's choice, also used by the Xbox controller's Y button.",
     [](float) {}, PARAM_FLAG_HIDDEN},
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

float savedValue(const char *key) {
    const int i = indexOf(key);
    if (i < 0) return NAN;
    portENTER_CRITICAL(&lock);
    const float v = saved[i];
    portEXIT_CRITICAL(&lock);
    return v;
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

bool saveKey(const char *key) {
    const int i = indexOf(key);
    if (i < 0) return false;
    const float v = valueAt((size_t)i);
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return false;
    const bool ok = prefs.putFloat(kParams[i].key, v) == sizeof(float);
    prefs.end();
    if (ok) {
        portENTER_CRITICAL(&lock);
        saved[i] = v;
        changeCounter = changeCounter + 1; // dirty flag may have changed
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
