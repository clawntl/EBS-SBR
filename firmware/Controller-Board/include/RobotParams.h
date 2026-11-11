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
extern uint16_t IMUsampleRate;  // Hz
extern uint16_t motorSendRate;  // Hz

extern bool paused;
