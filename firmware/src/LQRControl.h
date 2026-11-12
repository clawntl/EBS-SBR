#pragma once

#include <stdint.h>

// Target currents [A] for the left/right motors, produced by Task_LQR and
// consumed by Task_sendMotorCurrents.
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
    Paused,         // `paused` set from the web page -> 0 A
    Tipped,         // tilt beyond deactivationAngle -> 0 A
    Balancing,
};
extern volatile BalanceState balanceState;

void Task_LQR(void *pvParam);
