#pragma once

#include <stdint.h>

// Target currents [A] for the left/right motors, produced and sent over CAN
// by Task_LQR (also read by the display).
extern float leftTargetCurrent;
extern float rightTargetCurrent;

// Joystick drive input, written by WebInterface from the latest WebSocket
// message. Range -1..1. Task_LQR treats this as stale (and zeros it) if it
// hasn't been refreshed within JOYSTICK_TIMEOUT_MS, so a dropped connection
// can't leave the robot driving blind.
extern volatile float joystickAxisX; // turn rate set point, x maxTurnRate
extern volatile float joystickAxisY; // +1 = full forward (wheel speed set point, x maxWheelSpeed)
extern volatile unsigned long joystickLastUpdateMs;

// What Task_LQR is currently doing (for the display).
enum class BalanceState : uint8_t {
    MotorsNotReady, // one or both drivers not running FOC -> 0 A
    ImuStalled,     // no IMU sample for IMU_SAMPLE_TIMEOUT_MS -> 0 A
    Paused,         // `paused` set from the web page -> 0 A
    Tipped,         // tilt beyond deactivationAngle -> 0 A
    BatteryLow,     // battery below the cutoff (Battery::isLow()) -> 0 A
    Balancing,
};
extern volatile BalanceState balanceState;

const char *balanceStateName(BalanceState s); // "balancing", "tipped", ...

// One control step, as computed by Task_LQR. Used for the web page's live
// data (getControlSnapshot) and for recordings (Capture).
struct ControlSample {
    uint32_t timeUs;      // micros() at the step
    float tilt;           // [deg] theta[2] (0 = upright by the IMU)
    float tiltRate;       // [deg/s]
    float setpoint;       // [deg] angleSetPoint
    float phi;            // [rad] wheel angle, integrated from phiDot
    float phiDot;         // [rad/s] average of both wheels' speed (negative = rolling forward)
    float yawRate;        // [deg/s] positive = the way a positive yawCur turns it
    float wheelL, wheelR; // [rad/s] as reported by the drivers
    float lqr;            // [A] total balancing current from the LQR (before split/joystick/gating)
    float curL, curR;     // [A] currents actually sent
    float joyX, joyY;     // joystick input actually used (-1..1, 0 if stale)
    BalanceState state;

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

// Runs once per IMU sample (woken by Task_BMI160), computes the wheel
// currents and sends them over CAN immediately.
void Task_LQR(void *pvParam);
