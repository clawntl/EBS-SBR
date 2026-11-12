#pragma once

#include <Arduino.h>

// K0: Wheel displacement gain
// K1: Tilt angle gain
// K2: Yaw angle gain
// K3: Wheel velocity gain
// K4: Tilt velocity gain
// K5: Yaw velocity gain
extern float K[6];

extern float angleSetPoint;     // Unit: rad from upright
extern float deactivationAngle; // Unit: deg

extern float alpha; // Complementary filter coefficient
// IMU sample rate [Hz]. Also the LQR rate and the motor current CAN send
// rate: Task_LQR runs and sends once per IMU sample.
extern uint16_t IMUsampleRate;

extern bool paused;

// Joystick drive gains: how much current [A] full joystick deflection adds
// on top of the balancing current.
extern float turnCurrentGain;
extern float moveCurrentGain;
