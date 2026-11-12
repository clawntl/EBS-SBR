#pragma once

#include <cstdint>

struct RGBColour {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

void initStatusLED();
void flashStatusLED(RGBColour colour, uint16_t n, uint16_t onTime, uint16_t offTime);
