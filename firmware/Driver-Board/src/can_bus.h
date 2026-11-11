#pragma once

#include <cstdint>

// Configures FDCAN1 (bit rate, filters) and starts it. Call once from setup().
void configureAndStartCAN();

// Sends `dat` as a 5-byte classic CAN frame: [cmd][4 big-endian float bytes].
void sendFloat(uint8_t cmd, float dat, uint32_t canID);

// Checks for a received CAN frame and dispatches it (updating target_current
// or acting on a CAN_CMD). Call once per loop() iteration.
void pollCAN();
