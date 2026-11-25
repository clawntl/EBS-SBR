#include "StatusLED.h"

#include <Arduino.h>

#include "PinConfig.h"

namespace {

constexpr RGBColour kOnColour = {27, 29, 48};

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
bool flashing = false;
RGBColour flashColour = kOnColour;
uint16_t flashOnMs = 150, flashOffMs = 150;
TaskHandle_t ledTask = nullptr;

void writeStatusLED(RGBColour colour) {
    rgbLedWrite(STATUS_LED_PIN,
                (colour.r * RGB_BRIGHTNESS) / 255,
                (colour.g * RGB_BRIGHTNESS) / 255,
                (colour.b * RGB_BRIGHTNESS) / 255);
}

// Only this task writes the LED once it's running.
void Task_statusLED(void *) {
    bool lit = false; // currently showing the flash colour
    while (1) {
        portENTER_CRITICAL(&lock);
        const bool on = flashing;
        const RGBColour c = flashColour;
        const uint16_t onMs = flashOnMs, offMs = flashOffMs;
        portEXIT_CRITICAL(&lock);

        if (!on) {
            if (lit) { writeStatusLED(kOnColour); lit = false; }
            // Woken at once by startStatusFlash().
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        writeStatusLED(c);
        lit = true;
        // A stop during the on time ends it early.
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(onMs))) continue;
        writeStatusLED(kOnColour);
        lit = false;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(offMs));
    }
}

void setFlash(bool on, RGBColour colour, uint16_t onMs, uint16_t offMs) {
    portENTER_CRITICAL(&lock);
    flashing = on;
    if (on) {
        flashColour = colour;
        flashOnMs = onMs ? onMs : 1;
        flashOffMs = offMs ? offMs : 1;
    }
    portEXIT_CRITICAL(&lock);
    if (ledTask) xTaskNotifyGive(ledTask);
}

} // namespace

void initStatusLED() {
    writeStatusLED(kOnColour);
    if (!ledTask) xTaskCreatePinnedToCore(Task_statusLED, "statusLED", 2048, nullptr, 1, &ledTask, 0);
}

void startStatusFlash(RGBColour colour, uint16_t onMs, uint16_t offMs) {
    setFlash(true, colour, onMs, offMs);
}

void stopStatusFlash() {
    setFlash(false, kOnColour, 0, 0);
}
