#pragma once

#include <stdint.h>

// Drive input arbiter: decides which source drives the robot.
//
// Sources, each giving a speed and a turn request as fractions of the top
// speed / turn rate (-1..1; speed +1 = full forward, turn +1 = full right):
//  - Web:     the page's touch joystick or a gamepad on the phone/computer
//  - Gamepad: an Xbox controller paired with the robot over BLE (Gamepad.h)
//  - Module:  a CAN module, 0x200 + id (docs/can-modules.md)
//
// Rules, applied on every control step (current()):
//  1. A manual source (web, gamepad) that is fresh and off-centre wins; if
//     both are, the one that changed most recently.
//  2. Otherwise the lowest-numbered CAN module that is fresh and asking to
//     drive (and modules are allowed: the `modules` setting).
//  3. Otherwise nothing: 0, 0 (the robot holds position and heading).
// So picking up the joystick or the controller always overrides a module.
//
// Each source goes stale on its own if it stops updating (WEB/GAMEPAD/
// MODULE_TIMEOUT_MS), so a dropped connection or a crashed module can't leave
// the robot driving on a frozen command.
namespace DriveInput {

enum class Source : uint8_t { None = 0, Web = 1, Gamepad = 2, Module = 3 };

struct Demand {
    float turn;      // -1..1
    float speed;     // -1..1
    Source source;
    uint8_t module;  // module id if source == Module
};

constexpr uint32_t WEB_TIMEOUT_MS = 300;
constexpr uint32_t GAMEPAD_TIMEOUT_MS = 300;
constexpr uint32_t MODULE_TIMEOUT_MS = 250;
constexpr uint8_t  MAX_MODULES = 16;
// Below this the stick counts as centred (not taking over).
constexpr float ACTIVE_THRESHOLD = 0.05f;

void setWeb(float turn, float speed);
void setGamepad(float turn, float speed);
// drive = false: the module is present but not asking to drive.
void setModule(uint8_t id, float turn, float speed, bool drive);

// Allow CAN modules to drive (the `modules` setting).
extern bool modulesAllowed;

// Car-style steering in reverse (the `revSteer` setting): while a manual
// source (web joystick, Xbox controller) asks to go backwards, its turn is
// mirrored, so "right" swings the robot's back to the right, as when
// reversing a car, instead of turning it clockwise. CAN modules give a turn
// RATE (+ = clockwise from above) and are never mirrored.
extern bool carSteerReverse;

// The demand for this control step (called by Task_LQR).
Demand current();

const char *sourceName(Source s); // "none", "web", "gamepad", "module"

// Latest demand as used by the controller (for the page/display).
Demand lastUsed();

} // namespace DriveInput
