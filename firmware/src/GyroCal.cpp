#include "GyroCal.h"

#include <math.h>

#include "Boot.h"
#include "Config.h"
#include "DeviceLog.h"
#include "IMUDriver.h"
#include "LQRControl.h"
#include "Settings.h"
#include "StatusLED.h"

namespace {

const char *const kKeys[3] = { "gyroOffX", "gyroOffY", "gyroOffZ" };

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
GyroCal::Source source = GyroCal::Source::None;
float noiseUsed = 0;
volatile uint32_t changeCounter = 0;
volatile bool rezeroRunning = false;
uint32_t rezeroSeq = 0; // gyroMeasurementCount() when the re-zero started

void bump() { changeCounter = changeCounter + 1; }

bool readSaved(float out[3]) {
    bool any = false;
    for (int i = 0; i < 3; i++) {
        out[i] = Settings::savedValue(kKeys[i]);
        if (!isfinite(out[i])) out[i] = 0;
        if (out[i] != 0) any = true;
    }
    return any;
}

void use(const float dps[3], GyroCal::Source src, float noise) {
    setGyroOffsets(dps);
    portENTER_CRITICAL(&lock);
    source = src;
    noiseUsed = noise;
    portEXIT_CRITICAL(&lock);
    bump();
}

// Writes the offsets into the settings and flash (just these three keys,
// leaving any other unsaved changes unsaved). Stalls both cores briefly.
bool save(const float dps[3]) {
    bool ok = true;
    for (int i = 0; i < 3; i++) {
        ok &= Settings::set(kKeys[i], dps[i]);
        ok &= Settings::saveKey(kKeys[i]);
    }
    bump();
    return ok;
}

float maxDiff(const float a[3], const float b[3]) {
    return fmaxf(fabsf(a[0] - b[0]), fmaxf(fabsf(a[1] - b[1]), fabsf(a[2] - b[2])));
}

bool still(const GyroMeasurement &m) { return m.maxStd() < GYRO_STILL_STD_DPS; }

void logMeasurement(const char *what, const GyroMeasurement &m) {
    DeviceLog::printf("[imu] %s: X %+.3f, Y %+.3f, Z %+.3f deg/s (noise %.2f deg/s, accel %.3f g, %u samples)%s\n", what,
                      m.mean[0], m.mean[1], m.mean[2], m.maxStd(), m.accelG, m.samples,
                      still(m) ? "" : " - MOVING, not used");
}

} // namespace

namespace GyroCal {

const char *sourceName(Source s) {
    switch (s) {
        case Source::None:   return "not calibrated";
        case Source::Check:  return "boot check";
        case Source::Zero:   return "zeroed at boot";
        case Source::Saved:  return "saved (robot moving at boot)";
        case Source::Moving: return "measured while moving";
        case Source::Rezero: return "re-zeroed";
    }
    return "?";
}

void calibrateAtBoot() {
    float saved[3];
    const bool haveSaved = readSaved(saved);
    if (!imuInitOk()) DeviceLog::println("[imu] BMI160 didn't initialise - the gyro readings may be garbage");

    startStatusFlash(LED_GYRO_CAL, 150, 150);
    GyroMeasurement m = {};
    bool measured = false, done = false;
    uint32_t settle = GYRO_SETTLE_MS; // only before the first measurement

    if (haveSaved) {
        DeviceLog::printf("[imu] saved gyro offsets: X %+.3f, Y %+.3f, Z %+.3f deg/s - checking\n", saved[0], saved[1], saved[2]);
        for (uint8_t t = 1; t <= GYRO_BOOT_TRIES && !done; t++) {
            if (t == 1) Boot::detail("quick check");
            else        Boot::detail("keep still! (%u/%u)", t, GYRO_BOOT_TRIES);
            if (!measureGyro(GYRO_CHECK_MS, settle, m)) break;
            settle = 0;
            measured = true;
            logMeasurement("gyro check", m);
            if (!still(m)) continue;
            const float d = maxDiff(m.mean, saved);
            if (d <= GYRO_CHECK_TOL_DPS) {
                use(m.mean, Source::Check, m.maxStd());
                DeviceLog::printf("[imu] gyro check agrees with the saved offsets (within %.2f deg/s): using it\n", d);
                done = true;
            } else {
                DeviceLog::printf("[imu] gyro check is %.2f deg/s from the saved offsets (more than %.2f) - doing a full zero\n",
                                  d, GYRO_CHECK_TOL_DPS);
                break;
            }
        }
        if (!done && measured && !still(m)) {
            use(saved, Source::Saved, 0);
            DeviceLog::println("[imu] robot kept moving - using the saved gyro offsets");
            done = true;
        }
    }

    if (!done) {
        for (uint8_t t = 1; t <= GYRO_BOOT_TRIES && !done; t++) {
            if (t == 1) Boot::detail("zeroing - keep still");
            else        Boot::detail("keep still! (%u/%u)", t, GYRO_BOOT_TRIES);
            if (!measureGyro(GYRO_ZERO_MS, settle, m)) break;
            settle = 0;
            measured = true;
            logMeasurement("gyro zero", m);
            if (!still(m)) continue;
            use(m.mean, Source::Zero, m.maxStd());
            done = true;
            if (!haveSaved) {
                Boot::detail("saving offsets");
                DeviceLog::printf("[imu] no saved gyro offsets: saving these %s\n", save(m.mean) ? "" : "- SAVE FAILED");
            } else {
                DeviceLog::println("[imu] using the new zero (saved offsets kept; Re-zero gyro on the Tune tab saves new ones)");
            }
        }
    }

    if (!done) {
        if (haveSaved) {
            use(saved, Source::Saved, 0);
            DeviceLog::println("[imu] WARNING: couldn't zero the gyro - using the saved offsets");
        } else if (measured) {
            use(m.mean, Source::Moving, m.maxStd());
            DeviceLog::println("[imu] WARNING: the robot kept moving and no offsets are saved - using the last reading; "
                               "lay it still and use Re-zero gyro (Tune tab)");
        } else {
            DeviceLog::println("[imu] WARNING: no gyro measurement (IMU task not running?) - gyro offsets left at 0");
        }
    }
    stopStatusFlash();
}

const char *requestRezero() {
    if (Boot::active()) return "Still starting up - try again in a moment";
    const BalanceState st = balanceState;
    if (st == BalanceState::Balancing || st == BalanceState::Righting)
        return "Can't re-zero while it's balancing or self-righting - lay it down still first";
    if (rezeroRunning || gyroMeasurementBusy()) return "A gyro measurement is already running";
    const uint32_t seq = gyroMeasurementCount();
    if (!startGyroMeasurement(GYRO_ZERO_MS)) return "A gyro measurement is already running";
    rezeroSeq = seq;
    rezeroRunning = true;
    bump();
    startStatusFlash(LED_GYRO_CAL, 150, 150);
    DeviceLog::println("[imu] re-zeroing the gyro - keep the robot still");
    return nullptr;
}

bool serviceRezero(String &message, bool &ok) {
    if (!rezeroRunning || gyroMeasurementCount() == rezeroSeq) return false;
    rezeroRunning = false;
    stopStatusFlash();
    const GyroMeasurement m = lastGyroMeasurement();
    logMeasurement("gyro re-zero", m);
    char buf[160];
    if (!still(m)) {
        snprintf(buf, sizeof(buf), "Gyro not re-zeroed: the robot moved (noise %.2f deg/s, needs under %.1f). "
                                   "Lay it down still and try again.", m.maxStd(), GYRO_STILL_STD_DPS);
        ok = false;
    } else {
        use(m.mean, Source::Rezero, m.maxStd());
        const bool saved = save(m.mean);
        snprintf(buf, sizeof(buf), "Gyro re-zeroed%s: X %+.3f, Y %+.3f, Z %+.3f deg/s",
                 saved ? " and saved" : " (SAVE FAILED)", m.mean[0], m.mean[1], m.mean[2]);
        DeviceLog::printf("[imu] %s\n", buf);
        ok = saved;
    }
    message = buf;
    bump();
    return true;
}

Status status() {
    Status s;
    getGyroOffsets(s.offset);
    s.haveSaved = readSaved(s.saved);
    portENTER_CRITICAL(&lock);
    s.source = source;
    s.noise = noiseUsed;
    portEXIT_CRITICAL(&lock);
    s.busy = rezeroRunning;
    return s;
}

uint32_t version() { return changeCounter; }

} // namespace GyroCal
