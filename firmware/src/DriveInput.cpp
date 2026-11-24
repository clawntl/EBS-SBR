#include "DriveInput.h"

#include <Arduino.h>
#include <math.h>

#include "DeviceLog.h"

namespace DriveInput {

bool modulesAllowed = true;
bool carSteerReverse = true;

namespace {

struct Input {
    float turn = 0, speed = 0;
    uint32_t updatedMs = 0;  // last update (0 = never)
    uint32_t activeMs = 0;   // last update while off-centre (or, for modules, asking to drive)
    bool drive = false;      // modules: asking to drive
};

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
Input web, gamepad, modules[MAX_MODULES];
Demand used = { 0, 0, Source::None, 0 };
Source loggedSource = Source::None;
uint8_t loggedModule = 0;

bool offCentre(float t, float s) { return fabsf(t) > ACTIVE_THRESHOLD || fabsf(s) > ACTIVE_THRESHOLD; }

float clamp1(float v) {
    if (!isfinite(v)) return 0;
    return v < -1 ? -1 : v > 1 ? 1 : v;
}

void store(Input &in, float turn, float speed, bool active) {
    const uint32_t now = millis() | 1;
    portENTER_CRITICAL(&lock);
    in.turn = clamp1(turn);
    in.speed = clamp1(speed);
    in.updatedMs = now;
    if (active) in.activeMs = now;
    portEXIT_CRITICAL(&lock);
}

// Signed difference: another core may have stored an update stamped a
// millisecond AFTER the caller read `now` (or in the same millisecond, with
// the |1 above); unsigned `now - updatedMs` would then wrap to ~4e9 and the
// input would look stale for one control step.
bool fresh(const Input &in, uint32_t now, uint32_t timeout) {
    return in.updatedMs && (int32_t)(now - in.updatedMs) <= (int32_t)timeout;
}

} // namespace

void setWeb(float turn, float speed) { store(web, turn, speed, offCentre(turn, speed)); }
void setGamepad(float turn, float speed) { store(gamepad, turn, speed, offCentre(turn, speed)); }

void setModule(uint8_t id, float turn, float speed, bool drive) {
    if (id >= MAX_MODULES) return;
    store(modules[id], turn, speed, drive);
    portENTER_CRITICAL(&lock);
    modules[id].drive = drive;
    portEXIT_CRITICAL(&lock);
}

Demand current() {
    const uint32_t now = millis();
    Demand d = { 0, 0, Source::None, 0 };

    portENTER_CRITICAL(&lock);
    const bool webOn = fresh(web, now, WEB_TIMEOUT_MS) && offCentre(web.turn, web.speed);
    const bool gpOn = fresh(gamepad, now, GAMEPAD_TIMEOUT_MS) && offCentre(gamepad.turn, gamepad.speed);
    if (webOn && (!gpOn || web.activeMs >= gamepad.activeMs)) {
        d = { web.turn, web.speed, Source::Web, 0 };
    } else if (gpOn) {
        d = { gamepad.turn, gamepad.speed, Source::Gamepad, 0 };
    } else if (modulesAllowed) {
        for (uint8_t i = 0; i < MAX_MODULES; i++) {
            const Input &m = modules[i];
            if (m.drive && fresh(m, now, MODULE_TIMEOUT_MS)) {
                d = { m.turn, m.speed, Source::Module, i };
                break;
            }
        }
    }
    if (carSteerReverse && (d.source == Source::Web || d.source == Source::Gamepad) && d.speed < -ACTIVE_THRESHOLD)
        d.turn = -d.turn;
    used = d;
    portEXIT_CRITICAL(&lock);

    // Log changes of who's driving (not the frequent web <-> none flicker as
    // the joystick is released and grabbed again).
    if (d.source != loggedSource || (d.source == Source::Module && d.module != loggedModule)) {
        if (d.source == Source::Module || loggedSource == Source::Module ||
            d.source == Source::Gamepad || loggedSource == Source::Gamepad) {
            if (d.source == Source::Module) DeviceLog::printf("[drive] now driven by CAN module %u\n", d.module);
            else DeviceLog::printf("[drive] now driven by: %s\n", sourceName(d.source));
        }
        loggedSource = d.source;
        loggedModule = d.module;
    }
    return d;
}

Demand lastUsed() {
    portENTER_CRITICAL(&lock);
    const Demand d = used;
    portEXIT_CRITICAL(&lock);
    return d;
}

const char *sourceName(Source s) {
    switch (s) {
        case Source::None:    return "none";
        case Source::Web:     return "web";
        case Source::Gamepad: return "gamepad";
        case Source::Module:  return "module";
    }
    return "?";
}

} // namespace DriveInput
