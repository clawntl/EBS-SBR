#include "Boot.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "DeviceLog.h"

namespace {

constexpr uint8_t kCount = (uint8_t)BootStep::Count;

const char *const kNames[kCount] = {
    "Loading settings",
    "Starting WiFi",
    "Starting CAN",
    "Starting IMU",
    "Calibrating gyro",
    "Starting motors",
    "Starting control",
};

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
volatile bool booting = true;
int8_t current = -1;               // index of the running step
uint32_t startMs[kCount] = {};     // millis() when each step started
uint32_t durationMs[kCount] = {};  // set when the step ends
bool done[kCount] = {};
uint32_t finishedMs = 0;
char detailText[22] = "";

void endCurrent(uint32_t now) {
    if (current >= 0 && !done[current]) {
        durationMs[current] = now - startMs[current];
        done[current] = true;
    }
}

} // namespace

namespace Boot {

void step(BootStep s) {
    const uint8_t i = (uint8_t)s;
    if (i >= kCount) return;
    const uint32_t now = millis();
    portENTER_CRITICAL(&lock);
    endCurrent(now);
    current = i;
    startMs[i] = now;
    done[i] = false;
    detailText[0] = '\0';
    portEXIT_CRITICAL(&lock);
    // millis() counts from when the app started: the ROM bootloader and
    // loading the app (~0.3 s) come before it.
    DeviceLog::printf("[boot %02u/%02u +%.2f s] %s\n", i + 1, kCount, now / 1000.0f, kNames[i]);
}

void detail(const char *fmt, ...) {
    char buf[sizeof(detailText)];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    portENTER_CRITICAL(&lock);
    memcpy(detailText, buf, sizeof(detailText));
    portEXIT_CRITICAL(&lock);
}

void finish() {
    const uint32_t now = millis();
    portENTER_CRITICAL(&lock);
    endCurrent(now);
    finishedMs = now;
    booting = false;
    portEXIT_CRITICAL(&lock);
    printSummary();
}

bool active() { return booting; }

View view() {
    View v;
    portENTER_CRITICAL(&lock);
    v.index = current < 0 ? 0 : current + 1;
    v.total = kCount;
    strncpy(v.name, current < 0 ? "Starting" : kNames[current], sizeof(v.name) - 1);
    v.name[sizeof(v.name) - 1] = '\0';
    memcpy(v.detail, detailText, sizeof(v.detail));
    portEXIT_CRITICAL(&lock);
    return v;
}

void printSummary() {
    uint32_t start[kCount], dur[kCount], fin;
    bool fin_[kCount];
    int8_t cur;
    portENTER_CRITICAL(&lock);
    memcpy(start, startMs, sizeof(start));
    memcpy(dur, durationMs, sizeof(dur));
    memcpy(fin_, done, sizeof(fin_));
    fin = finishedMs;
    cur = current;
    portEXIT_CRITICAL(&lock);

    if (fin) DeviceLog::printf("[boot] ready %.2f s after the app started; steps:\n", fin / 1000.0f);
    else     DeviceLog::println("[boot] still booting; steps so far:");
    for (uint8_t i = 0; i < kCount; i++) {
        if (fin_[i])
            DeviceLog::printf("[boot]   %02u %-17s at %5.2f s, took %5u ms\n", i + 1, kNames[i], start[i] / 1000.0f, (unsigned)dur[i]);
        else if (i == cur)
            DeviceLog::printf("[boot]   %02u %-17s at %5.2f s, running\n", i + 1, kNames[i], start[i] / 1000.0f);
    }
}

} // namespace Boot
