#pragma once

// Orientation estimate, updated by Task_BMI160.
// {yaw, pitch, roll}, units: rad and rad/s
extern float theta[3];
extern float theta_dot[3];

// Sets up the SPI bus and the BMI160, then zeroes the gyro offsets.
// Blocking: the zero routine takes ~3 seconds.
void initIMU();

void Task_BMI160(void *pvParams);
void Task_outputGyroData(void *pvParams);
