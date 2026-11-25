#pragma once

#include <cstdint>

struct RGBColour {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

// Gyro calibration colour (purple).
constexpr RGBColour LED_GYRO_CAL = {246, 20, 247};

// Lights the LED in its normal colour and starts the (low-priority, core 0)
// task that does the flashing below.
void initStatusLED();

// Flashes `colour` on for onMs, normal colour for offMs, until
// stopStatusFlash(). Doesn't block: the LED task does the flashing.
void startStatusFlash(RGBColour colour, uint16_t onMs, uint16_t offMs);
void stopStatusFlash();
