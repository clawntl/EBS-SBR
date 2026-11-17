#pragma once

#include <Arduino.h>

// Xbox Wireless Controller (model 1914 / Series X|S, or an Xbox One
// controller on BLE firmware 5.15+) paired directly with the robot over BLE.
//
// Pairing: start it from the web page (Control tab), then hold the
// controller's pair button (the small one on top) until the Xbox logo flashes
// fast. Only a controller IN PAIRING MODE is accepted during the 30 s window,
// so a nearby controller that's just switched on isn't grabbed. The robot
// remembers that controller (NVS) and from then on only connects to it: turn
// the controller on and it reconnects by itself.
//
// Driving: left stick (or left stick forward/back + right stick left/right,
// the `gpLayout` setting) -> DriveInput::setGamepad(). B = pause, A = resume,
// Y = start a recording (the length set on the web page, `recSecs`) or stop
// one early. The controller buzzes once when a recording starts and twice
// when it finishes.
// The ESP32-S3 only has BLE (not Bluetooth Classic), which these
// controllers support; Wi-Fi and BLE share the radio.
namespace Gamepad {

enum class State : uint8_t {
    Disabled,   // `gpEnable` off (BLE not started)
    NotPaired,  // no controller remembered
    Searching,  // waiting for the remembered controller to come on
    Pairing,    // pairing window open
    Connected,
};

struct Status {
    State state;
    String address;          // remembered controller ("" = none)
    uint8_t pairSecondsLeft; // while pairing
    bool inputActive;        // stick off-centre
    int16_t battery;         // controller battery %, -1 = not known
};

// Settings (Settings.cpp).
extern bool enabled;      // gpEnable (live: off = disconnect and leave the radio alone)
extern uint8_t layout;    // gpLayout: 0 = left stick, 1 = left stick speed + right stick turn
extern float deadzone;    // gpDeadzone: 0..0.5 of full stick travel

// Starts the BLE task (core 0). Call once, after Wi-Fi is up.
void begin();

void requestPairing();    // open the pairing window (30 s)
void requestForget();     // forget the remembered controller

Status status();
uint32_t version();       // bumps when the status changes
const char *stateName(State s);

} // namespace Gamepad
