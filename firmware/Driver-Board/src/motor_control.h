#pragma once

#include <SimpleFOC.h>

extern BLDCMotor motor;

// Target current commanded over CAN (broadcast BOTH_MOTOR_CAN_ADDR message), in Amps.
extern float target_current;

// True once startFOC() has completed and motorControlLoop() is actively driving the motor.
extern bool enabled;

// One-time driver/sensor/FOC configuration. Call once from setup(), before startFOC().
void setupFOC();

// Initializes current sense and runs FOC calibration, then enables closed-loop
// control. Call once, e.g. in response to CMD_START_FOC over CAN.
void startFOC();

// Runs one FOC control iteration and periodic CAN velocity telemetry.
// Call once per loop() iteration; a no-op until startFOC() has been called.
void motorControlLoop();
