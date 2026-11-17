#include "DeviceLog.h"

#include <stdarg.h>

namespace {

constexpr size_t LINES = 80;
constexpr size_t LINE_LEN = 116;

struct Entry {
    uint32_t seq;
    uint32_t ms;
    char text[LINE_LEN];
};

Entry ring[LINES];
uint32_t nextSeq = 1;
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

void store(const char *text) {
    // Trim trailing newlines; the page and Serial add their own.
    size_t len = strnlen(text, LINE_LEN - 1);
    while (len && (text[len - 1] == '\n' || text[len - 1] == '\r')) len--;

    const uint32_t now = millis();
    portENTER_CRITICAL(&lock);
    Entry &e = ring[nextSeq % LINES];
    e.seq = nextSeq++;
    e.ms = now;
    memcpy(e.text, text, len);
    e.text[len] = '\0';
    portEXIT_CRITICAL(&lock);
}

} // namespace

namespace DeviceLog {

void printf(const char *fmt, ...) {
    char buf[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
    store(buf);
}

void println(const char *text) {
    Serial.println(text);
    store(text);
}

uint32_t latest() {
    portENTER_CRITICAL(&lock);
    const uint32_t seq = nextSeq - 1;
    portEXIT_CRITICAL(&lock);
    return seq;
}

void since(uint32_t after, const std::function<void(uint32_t, uint32_t, const char *)> &fn) {
    const uint32_t newest = latest();
    // Only lines still in the ring (the oldest may have been overwritten).
    uint32_t seq = after + 1;
    if (newest >= LINES && seq <= newest - LINES) seq = newest - LINES + 1;
    for (; seq <= newest; seq++) {
        Entry copy;
        portENTER_CRITICAL(&lock);
        copy = ring[seq % LINES];
        portEXIT_CRITICAL(&lock);
        if (copy.seq == seq) fn(copy.seq, copy.ms, copy.text);
    }
}

} // namespace DeviceLog
