#pragma once

#include <Arduino.h>

// Boot progress: what setup() is doing, for the OLED's boot screen, the log
// (Serial and the web page's Log tab) and a timing summary.
//
// setup() calls Boot::step() at the start of each stage. Each call logs
//   [boot 03/07 +0.62 s] Starting CAN
// and the OLED shows the step number, its name, an optional detail line
// and a progress bar until Boot::finish(), which logs how long each step
// took. The serial command BOOT prints that summary again.

enum class BootStep : uint8_t {
    Settings,   // load saved settings
    Network,    // WiFi, web page, OTA, Xbox controller (run on in their own tasks)
    Can,        // CAN bus, driver status reception, motor supervisor
    Imu,        // BMI160 set-up and its sampling task
    Gyro,       // gyro offset check / zero (robot must be still)
    Motors,     // wait for both drivers to be running FOC
    Control,    // start the balance controller
    Count
};

namespace Boot {

// Starts a step (ends the previous one): logs it and updates the OLED.
void step(BootStep s);

// Second line on the boot screen (not logged), e.g. "keep still". printf-style;
// "" clears it.
void detail(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Ends the last step, logs the summary and hands the OLED to the normal screen.
void finish();

// True from power-on until finish().
bool active();

struct View {
    uint8_t index;   // 1-based step number (0 before the first step)
    uint8_t total;
    char name[22];
    char detail[22];
};
View view();

// Prints the step durations (also used by finish()). Before finish() it
// shows the steps so far.
void printSummary();

} // namespace Boot
