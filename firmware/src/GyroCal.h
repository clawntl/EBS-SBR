#pragma once

#include <Arduino.h>

// Gyro offset calibration: the boot-time check / zero, and Re-zero gyro from
// the web page. The measurements themselves are done by Task_BMI160 (see
// GyroMeasurement in IMUDriver.h); the offsets saved by the last re-zero
// live in Settings (gyroOffX/Y/Z, hidden; included in settings exports).
//
// At boot, with saved offsets: measure for GYRO_CHECK_MS. If the robot is
// still and every axis is within GYRO_CHECK_TOL_DPS of the saved offsets,
// use that fresh reading (it's measured at today's temperature, so it's
// more accurate than the saved one; the saved one is what shows the robot
// wasn't moving slowly). Otherwise do a full zero (GYRO_ZERO_MS) and use it.
// If the robot never keeps still, fall back to the saved offsets.
// With no saved offsets: full zero, and save it.

namespace GyroCal {

enum class Source : uint8_t {
    None,    // not calibrated yet (offsets 0)
    Check,   // boot check agreed with the saved offsets: its reading is used
    Zero,    // full zero at boot
    Saved,   // the saved offsets (the robot wasn't still at boot)
    Moving,  // no saved offsets and the robot never kept still: last reading, may be off
    Rezero,  // Re-zero gyro from the web page
};
const char *sourceName(Source s);

// Blocking (~0.35 s normally, up to a few seconds if it's being moved). Call
// once Task_BMI160 is running and before the motors start (they move the
// wheels while they align). Flashes the status LED purple meanwhile.
void calibrateAtBoot();

// Starts a re-zero (GYRO_ZERO_MS). Returns an error message, or nullptr if
// it started. Safe from any task.
const char *requestRezero();

// Call regularly from the network task: finishes a re-zero (applies and
// saves the offsets: the flash write belongs in that task). Returns true
// once per finished re-zero, with a message for the page.
bool serviceRezero(String &message, bool &ok);

struct Status {
    float offset[3];   // in use, deg/s
    float saved[3];    // saved in flash, deg/s
    bool haveSaved;
    Source source;
    bool busy;         // a re-zero is running
    float noise;       // largest axis standard deviation of the measurement used (deg/s), 0 if none
};
Status status();
// Incremented on every change of status().
uint32_t version();

} // namespace GyroCal
