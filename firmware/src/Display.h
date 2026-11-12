#pragma once

// 128x64 SSD1315/SSD1306 OLED on I2C (I2C_SDA / I2C_SCL in PinConfig.h).
//
// Normal screen: balance state, tilt angle, commanded currents and both motor
// drivers' state. If something is wrong (CAN down, a driver offline, a driver
// fault or error) a problem screen is shown instead, naming the fault.
//
// Drawing runs in its own low-priority task on core 0, so the I2C transfers
// (~25 ms per frame) never hold up the control tasks on core 1.

// Initialises the display and shows a boot message. Returns false (and the
// rest of the firmware carries on without a display) if it doesn't respond.
bool initDisplay();

// One-line boot progress message (only before Task_display starts).
void displayBootMessage(const char *line);

void Task_display(void *pvParams);
