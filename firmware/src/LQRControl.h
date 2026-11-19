#pragma once

#include <stdint.h>

// Target currents [A] for the left/right motors, produced and sent over CAN
// by Task_LQR (also read by the display).
extern float leftTargetCurrent;
extern float rightTargetCurrent;

// (Drive input - joystick, gamepad, CAN modules - comes from DriveInput.h.)

// What Task_LQR is currently doing (for the display).
enum class BalanceState : uint8_t {
    MotorsNotReady, // one or both drivers not running FOC -> 0 A
    ImuStalled,     // no IMU sample for IMU_SAMPLE_TIMEOUT_MS -> 0 A
    Paused,         // `paused` set from the web page -> 0 A
    Tipped,         // tilt beyond deactivationAngle -> 0 A
    BatteryLow,     // battery below the cutoff (Battery::isLow()) -> 0 A
    Balancing,
    Waiting,        // within the cut-out angle but not started: hold it upright and still -> 0 A
    Runaway,        // stopped: wheels spun faster than runawaySpeed (lifted?) -> 0 A until restarted
};
extern volatile BalanceState balanceState;

const char *balanceStateName(BalanceState s); // "balancing", "tipped", ...

// One control step, as computed by Task_LQR. Used for the web page's live
// data (getControlSnapshot) and for recordings (Capture).
struct ControlSample {
    uint32_t timeUs;      // micros() at the step
    float tilt;           // [deg] theta[2] (0 = upright by the IMU)
    float tiltRate;       // [deg/s]
    float setpoint;       // [deg] set point in use (angleSetPoint + learned trim)
    float phi;            // [rad] wheel angle, integrated from phiDot
    float phiDot;         // [rad/s] wheel speed used: average of both wheels, or the grounded one if a wheel is lifted (negative = rolling forward)
    float yawRate;        // [deg/s] positive = the way a positive yawCur turns it
    float wheelL, wheelR; // [rad/s] as reported by the drivers
    float lqr;            // [A] total balancing current from the LQR (before split/joystick/gating)
    float curL, curR;     // [A] currents actually sent
    float joyX, joyY;     // joystick input actually used (-1..1, 0 if stale)
    BalanceState state;
    uint8_t source;       // DriveInput::Source in bits 0-3, module id in bits 4-7
    uint8_t lifted;       // lifted/slipping wheel: 0 none, 1 left, 2 right (liftDetect)
    float holdScale;      // 0-1: how much of the K0/K3 terms was used (balancePriority)

    // Position & heading hold (0 unless balancing).
    float posErr;         // [rad] wheel angle minus its reference (clamped)
    float speedRef;       // [rad/s] ramped wheel speed set point
    float heading;        // [deg] heading since balancing started
    float headErr;        // [deg] heading minus its reference (clamped)
    float yawRateRef;     // [deg/s] requested turn rate
    float yawCur;         // [A] differential current (+ left, - right)

    // Raw IMU, body frame (see IMUDriver.h).
    float pitch;          // [deg] theta[1]
    float gyro[3];        // [deg/s] x, y, z, offsets removed
    float accel[3];       // [g] x, y, z
};

// Latest step plus the measured control loop rate [Hz].
ControlSample getControlSnapshot(uint16_t *loopHz = nullptr);

// Wheel spin test for tuning (measures motor torque per amp against wheel
// inertia): with the robot lying down and its wheels free, drives both wheels
// at +amps, 0, -amps, 0 for phaseMs each, and records it. Returns nullptr if
// started, otherwise why not.
const char *requestSpinTest(float amps, uint32_t phaseMs);

// Publishes the learned set point trim to Settings (about once a second)
// and saves it to flash once the robot stops balancing. Call regularly from
// a core-0 task (the network task does).
void serviceSetPointTrim();

// Runs once per IMU sample (woken by Task_BMI160), computes the wheel
// currents and sends them over CAN immediately.
void Task_LQR(void *pvParam);
