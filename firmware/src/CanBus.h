#pragma once

#include <stdint.h>

// Shaft velocities reported by the motor controllers over CAN.
extern float leftShaftVelocity;
extern float rightShaftVelocity;

// True once initCanBus() succeeded.
extern bool canBusOk;

// Brings up the CAN transceiver. Returns true on success.
bool initCanBus();

void sendStartFOC(uint32_t canID);
void sendFloat(uint8_t cmd, float dat, uint32_t canID);
void sendMotorCurrents(float leftCur, float rightCur, uint32_t canID);

void Task_readCAN(void *pvParams);
void Task_sendMotorCurrents(void *pvParams);
