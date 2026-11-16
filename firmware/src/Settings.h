#pragma once

#include <Arduino.h>

// All live-tunable parameters, described by one table (kParams in
// Settings.cpp) that drives the web page's controls, serial commands,
// validation and NVS storage alike.
//
// The values themselves are pushed into the existing globals in
// RobotParams.h (K[], angleSetPoint, ...) by each parameter's apply
// function, so the control code keeps reading plain globals. Each is a single
// 32-bit write, which the control tasks on core 1 see atomically.

struct ParamDef {
    const char *key;      // JSON / NVS key (max 15 chars for NVS)
    const char *label;    // web page label
    const char *group;    // web page panel
    float defaultValue;
    float min, max, step;
    bool rebootRequired;
    const char *help;     // web page explanation
    void (*apply)(float); // writes the value into the global(s) the firmware uses
    uint8_t flags;        // PARAM_FLAG_*
};

// Web page hint: show a fine-adjust slider (small relative nudges) under it.
constexpr uint8_t PARAM_FLAG_FINE = 1 << 0;
// Not used by the controller at the moment (shown greyed out).
constexpr uint8_t PARAM_FLAG_UNUSED = 1 << 1;

namespace Settings {

// Parameter table, in display order.
const ParamDef *params(size_t &count);
const ParamDef *find(const char *key);

// Current value by key (NAN for an unknown key) or table index.
float value(const char *key);
float valueAt(size_t index);

// Incremented on every change, so readers can cheaply detect updates.
uint32_t version();

// Set one parameter by key. The value is clamped to the parameter's range
// and applied immediately. Returns false for an unknown key or a non-finite
// value.
bool set(const char *key, float value, float *applied = nullptr);

// Restore every parameter to its default (does not save).
void resetToDefaults();

// NVS persistence. Flash writes briefly stall BOTH CPU cores, so the control
// loop misses a few samples during save() (it outputs 0 A if the IMU stalls
// for more than 10 ms). Only call it on an explicit user request.
void load();
bool save();

// True if the live settings differ from what is stored in flash.
bool isDirty();

} // namespace Settings
