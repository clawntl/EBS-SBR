#pragma once

#include <Arduino.h> // FreeRTOS TaskHandle_t

// Orientation estimate, updated by Task_BMI160.
// {yaw, pitch, roll}, units: rad and rad/s
extern float theta[3];
extern float theta_dot[3];

// Raw sensor axes in the BMI160's own (body) frame, for logging and the web
// page's chart. Gyro: rad/s with the boot-time offsets removed. Accel: g.
// On this robot X is vertical (up/down through the chassis), Y points
// forward/back and Z lies along the wheel axle (see Task_BMI160 for how they
// map onto yaw / pitch / tilt).
extern float imuGyro[3];  // {x, y, z} rad/s
extern float imuAccel[3]; // {x, y, z} g

// Sets up the SPI bus and the BMI160, then zeroes the gyro offsets.
// Blocking: the zero routine takes ~3 seconds.
void initIMU();

// Registers a task to be woken (xTaskNotifyGive) after every new orientation
// sample. Used by Task_LQR to run in lock-step with the IMU.
void setImuSampleListener(TaskHandle_t task);

// Samples the BMI160 at IMUsampleRate (fixed rate) and updates theta/theta_dot.
void Task_BMI160(void *pvParams);
void Task_outputGyroData(void *pvParams);
