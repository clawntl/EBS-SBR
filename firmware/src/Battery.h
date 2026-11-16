#pragma once

#include <Arduino.h>

// Battery monitor for the 3S Li-ion pack. There is no voltage sensing on the
// controller board: both motor drivers measure their supply (the same pack)
// and report it in their CAN status heartbeat every 100 ms. This averages
// whichever readings are current, estimates the state of charge and runtime,
// and applies the low-voltage lockout (with hysteresis) used by Task_LQR.
//
// Call update() regularly from a core 0 task; it works at most once per
// BATTERY_CHECK_INTERVAL_MS.
namespace Battery {

// Returns true when a new reading was taken.
bool update(float cutoffVolts);

// True when at least one driver is reporting a voltage.
bool known();

// Average of the drivers' readings, lightly smoothed (~5 s); 0 if unknown.
float voltage();

// Latest reading from each driver (unsmoothed apart from the driver's own
// ~200 ms filter); 0 if that driver isn't reporting one.
float leftVoltage();
float rightVoltage();

// Estimated state of charge, 0..100 %, from a heavily smoothed voltage
// (~30 s) and a typical Li-ion discharge curve. No current measurement, so it
// reads a little low while the motors work hard and recovers at rest.
uint8_t percent();

// Minutes until empty at the recent rate of discharge.
// -1: not enough history yet (first ~2 minutes, or after charging).
// -2: the level isn't falling (e.g. on a bench supply).
int16_t minutesRemaining();

// Below the cutoff (and not yet recovered): Task_LQR outputs 0 A.
bool isLow();

} // namespace Battery
