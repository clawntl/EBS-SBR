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
// Sends a command frame ([cmd][payload...], up to 8 bytes) to a driver.
void sendDriverCommand(uint32_t canID, const uint8_t *data, uint8_t len);
void sendFloat(uint8_t cmd, float dat, uint32_t canID);
// Target currents [A] for the left and right drivers (the nodes in
// leftNode / rightNode): one current frame if they share one, otherwise one
// each with NaN ("no command") in the other node's slot.
void sendMotorCurrents(float leftCur, float rightCur);

void Task_readCAN(void *pvParams);
