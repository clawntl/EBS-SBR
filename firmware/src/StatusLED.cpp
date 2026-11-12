#include "StatusLED.h"

#include <Arduino.h>

#include "PinConfig.h"

namespace {

constexpr RGBColour kOnColour = {27, 29, 48};

void writeStatusLED(RGBColour colour) {
    rgbLedWrite(STATUS_LED_PIN,
                (colour.r * RGB_BRIGHTNESS) / 255,
                (colour.g * RGB_BRIGHTNESS) / 255,
                (colour.b * RGB_BRIGHTNESS) / 255);
}

} // namespace

void initStatusLED() {
    writeStatusLED(kOnColour);
}

void flashStatusLED(RGBColour colour, uint16_t n, uint16_t onTime, uint16_t offTime) {
    for (uint16_t i = 0; i < n; i++) {
        writeStatusLED(colour);
        delay(onTime);
        writeStatusLED(kOnColour);
        delay(offTime);
    }
}
