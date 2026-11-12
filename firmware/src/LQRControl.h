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
extern volatile float joystickAxisX; // turn
extern volatile float joystickAxisY; // forward/back
extern volatile unsigned long joystickLastUpdateMs;

// What Task_LQR is currently doing (for the display).
enum class BalanceState : uint8_t {
    MotorsNotReady, // one or both drivers not running FOC -> 0 A
    ImuStalled,     // no IMU sample for IMU_SAMPLE_TIMEOUT_MS -> 0 A
    Paused,         // `paused` set from the web page -> 0 A
    Tipped,         // tilt beyond deactivationAngle -> 0 A
    Balancing,
};
extern volatile BalanceState balanceState;

// Runs once per IMU sample (woken by Task_BMI160), computes the wheel
// currents and sends them over CAN immediately.
void Task_LQR(void *pvParam);
