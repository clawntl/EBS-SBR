#pragma once

#include <Arduino.h> // FreeRTOS TaskHandle_t

// Orientation estimate, updated by Task_BMI160.
// {yaw, pitch, roll}, units: rad and rad/s
extern float theta[3];
extern float theta_dot[3];

// Raw sensor axes in the BMI160's own (body) frame, for logging and the web
// page's chart. Gyro: rad/s with the gyro offsets removed. Accel: g.
// On this robot X is vertical (up/down through the chassis), Y points
// forward/back and Z lies along the wheel axle (see Task_BMI160 for how they
// map onto yaw / pitch / tilt).
extern float imuGyro[3];  // {x, y, z} rad/s
extern float imuAccel[3]; // {x, y, z} g

// Sets up the SPI bus and the BMI160 (a few ms). The gyro offsets start at 0:
// GyroCal (src/GyroCal.h) measures and sets them once Task_BMI160 runs.
// Returns false if the BMI160 didn't initialise.
bool initIMU();
bool imuInitOk();

// Registers a task to be woken (xTaskNotifyGive) after every new orientation
// sample. Used by Task_LQR to run in lock-step with the IMU.
void setImuSampleListener(TaskHandle_t task);

// Samples the BMI160 at IMUsampleRate (fixed rate) and updates theta/theta_dot.
void Task_BMI160(void *pvParams);
void Task_outputGyroData(void *pvParams);

// ---- Gyro offsets ------------------------------------------------------------
// In deg/s, in the BMI160's own axes (X, Y, Z), subtracted from every reading.
void setGyroOffsets(const float dps[3]);
void getGyroOffsets(float dps[3]);

// ---- Gyro measurement ----------------------------------------------------------
// Mean and standard deviation of the raw gyro rates over a number of samples,
// measured by Task_BMI160 on the samples it reads anyway (so it needs that
// task running). Used for zeroing and checking the offsets.
struct GyroMeasurement {
    uint16_t samples;
    float mean[3]; // deg/s, offsets NOT removed
    float std[3];  // deg/s
    float accelG;  // mean accelerometer magnitude, g (about 1.000 at rest)
    float maxStd() const;
};

// Starts a measurement lasting `ms` (at the current sample rate), ignoring
// the samples in the first `settleMs`. Returns false if one is already running.
bool startGyroMeasurement(uint32_t ms, uint32_t settleMs = 0);
bool gyroMeasurementBusy();
// Incremented each time a measurement finishes.
uint32_t gyroMeasurementCount();
GyroMeasurement lastGyroMeasurement();

// Blocking version (polls every 10 ms). False if one was already running or
// it timed out (IMU task not running).
bool measureGyro(uint32_t ms, uint32_t settleMs, GyroMeasurement &out);
