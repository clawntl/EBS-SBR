#pragma once

#include <Arduino.h>

// Live tuning values used by the control code. They are set at boot from the
// saved settings (or the defaults) and changed through Settings (src/Settings.h),
// which the web page and serial commands use. Defaults live in Settings.cpp.

// K0: Wheel displacement gain
// K1: Tilt angle gain
// K2: Yaw angle gain
// K3: Wheel velocity gain
// K4: Tilt velocity gain
// K5: Yaw velocity gain
extern float K[6];

extern float angleSetPoint;     // Unit: rad from upright
// Learned correction added to angleSetPoint (rad). Learned while the robot
// stands still (see learnTrim / LQRControl.cpp) and saved by itself.
extern float setPointTrim;
extern bool learnTrim;
extern float deactivationAngle; // Unit: deg
extern float engageAngle;       // [deg] from the set point, to start balancing
extern float sideCutoutAngle;   // [deg] sideways lean (theta[1]) beyond which it won't balance
extern float runawaySpeed;      // [rad/s] wheel speed that stops balancing
extern float maxWheelCurrent;   // [A] per-wheel current limit (drivers: 2 A)
// When the motors are at their limit, the tilt terms (K1, K4) get priority
// over the position/speed terms (K0, K3): see LQRControl.cpp.
extern bool balancePriority;
// Lifted / slipping wheel detection: if the two wheels' speeds differ by more
// than the turning explains (by liftSpeed rad/s), the wheel speed used by the
// controller comes from the slower (grounded) wheel only.
extern bool liftDetect;
extern float liftSpeed;         // [rad/s]
extern float wheelRadius;       // [m]
extern float trackWidth;        // [m] tyre centre to centre

extern float alpha; // Complementary filter coefficient
// IMU sample rate [Hz]. Also the LQR rate and the motor current CAN send
// rate: Task_LQR runs and sends once per IMU sample.
extern uint16_t IMUsampleRate;

extern bool paused;

// Driving. The drive inputs (web joystick / gamepad, later other sources)
// are -1..1 and are turned into SETPOINTS for the balance controller rather
// than extra current: a wheel-speed set point (forward/back) and a yaw-rate
// set point (turning). The controller then holds position and heading when
// the input is released (see LQRControl.cpp).
extern float maxWheelSpeed;  // [rad/s] wheel speed at full forward/back
extern float maxWheelAccel;  // [rad/s^2] how fast the speed set point may change
extern float maxSpeedLead;   // [rad/s] how far the speed set point may run ahead of the wheels (0 = no limit)
extern float maxTurnRate;    // [rad/s] yaw rate at full left/right
extern float turnFeedForward; // [A] differential current at full turn (feed-forward)
extern bool  yawInvert;      // flip the gyro's yaw sign to match the wheels
// Limits on the position / heading error the hold loops act on. Beyond
// them the reference is dragged along, so a robot that's pushed or turned
// by hand doesn't wind up and lunge back.
extern float maxPositionError; // [rad] of wheel rotation
extern float maxHeadingError;  // [rad]

// Self-righting (see LQRControl.cpp and docs/self-righting.md). Lying on
// its hoops the wheels are in the air, so the wheels are used as reaction
// wheels to rock the robot on its hoop until the wheels touch the ground,
// then the balance controller takes over.
extern uint8_t selfRightMode;     // 0 off, 1 on request (web / Xbox X / CAN), 2 also automatic
extern float selfRightCurrent;    // [A] per wheel while rocking
extern float selfRightWheelCap;   // [rad/s] no more push in the direction the wheels already spin this fast
extern float selfRightHandover;   // [deg] from the set point: hand over to balancing inside this
extern float selfRightTimeout;    // [s] per attempt
extern uint8_t selfRightTries;    // attempts per request
extern float selfRightDelay;      // [s] lying still this long before an automatic start
extern bool selfRightInvert;      // flip the push direction (if a push test rocks it the wrong way)
extern bool selfRightRecord;      // record each self-righting
