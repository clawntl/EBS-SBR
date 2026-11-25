#pragma once

#include <stdint.h>

// Fixed build-time configuration. Tunable parameters live in Settings
// (src/Settings.cpp) and can be changed live from the web page or serial.

// Controller firmware version: shown on the web page's Info tab and stored in
// exported settings files, so an import can warn when the file came from
// different firmware. Bump the minor version when settings are added, removed
// or renamed, the major version when an existing setting changes meaning or
// units, and the patch version for anything else.
#define FIRMWARE_VERSION     "1.2.0"

// ----------------------------------------------
// Boot
// ----------------------------------------------
// After Serial.begin(), wait up to this long for a USB serial host so the
// first boot messages can be seen; carries on as soon as one is there. The
// web page's Log tab has them all anyway (and the serial BOOT command prints
// the boot timing again).
constexpr uint32_t BOOT_SERIAL_WAIT_MS = 500;
// How long setup() waits for both motor drivers to be running FOC before
// starting the balance controller anyway (the supervisor keeps retrying).
constexpr uint32_t BOOT_MOTOR_WAIT_MS = 15000;

// ----------------------------------------------
// Gyro calibration (src/IMUDriver.cpp)
// ----------------------------------------------
// At boot the gyro is measured briefly and compared with the offsets saved by
// the last re-zero (Tune tab, IMU panel). If the robot is still and the
// reading agrees with them, that fresh reading is used; if it disagrees a full
// zero is done instead. With no saved offsets (first boot, or after Defaults)
// a full zero is done and saved.
constexpr uint32_t GYRO_CHECK_MS = 250;     // quick check at boot
constexpr uint32_t GYRO_ZERO_MS = 1000;     // full zero (boot or re-zero from the web page)
// Settling time after the BMI160 is switched on, before its samples are used.
constexpr uint32_t GYRO_SETTLE_MS = 100;
// "Still": the standard deviation of every gyro axis over the measurement is
// below this. The BMI160's own noise is a few tenths of a deg/s per sample;
// a robot being handled or rocking moves far more.
constexpr float GYRO_STILL_STD_DPS = 1.0f;
// Quick check passes if every axis is within this of the saved offset.
// BMI160 offsets drift about 0.05 deg/s per degree C, so a robot that's much
// warmer or colder than at the last re-zero can fail it (then it does a full
// zero, ~1 s longer).
constexpr float GYRO_CHECK_TOL_DPS = 0.5f;
// Measurements to try at boot before giving up waiting for the robot to be
// still (then the saved offsets, or the last measurement, are used).
constexpr uint8_t GYRO_BOOT_TRIES = 3;

// ----------------------------------------------
// Network
// ----------------------------------------------
// Each robot's names come from the last AP_MAC_SUFFIX_BYTES bytes of its
// soft-AP MAC in lower-case hex (the first 3 are Espressif's OUI, identical
// on every board), so several robots can run side by side:
//   access point  SSID "BalanceBot-3f2a1c", password "balancebot-3f2a1c"
//   hostname      balancebot-3f2a1c (http://balancebot-3f2a1c.local/ and OTA
//                 when on a home network)
// The password is derivable from the SSID: it stops casual joins, not a
// determined person. tools/wifi_qr.py makes scan-to-join signs from the suffix.
#define AP_SSID_PREFIX       "BalanceBot-"
#define AP_PASSWORD_PREFIX   "balancebot-"   // full password must be >= 8 chars
#define HOSTNAME_PREFIX      "balancebot-"
constexpr uint8_t AP_MAC_SUFFIX_BYTES = 3;   // 1..6

// OTA upload password. Must match --auth in platformio.ini [env:ota].
#define OTA_PASSWORD         "balancebot"

// Address of the robot on its own access point (and of the captive-portal
// page). Deliberately NOT the ESP32 default 192.168.4.1: some home routers
// (e.g. eero) use 192.168.4.x, and an AP on the same range as the network the
// robot has joined breaks routing. Keep AP_IP_STRING in step with the octets.
#define AP_IP_OCTETS         192, 168, 71, 1
#define AP_IP_STRING         "192.168.71.1"

// "Access point only" (saved in NVS, switchable from the WiFi tab): the
// robot never joins a saved network and always hosts its own access point.
// This is the default for a freshly flashed robot, since open days are the
// main use and visitors connect straight to the robot.
constexpr bool WIFI_AP_ONLY_DEFAULT = true;

// Saved networks (managed from the web page), used when AP-only is off.
constexpr uint8_t  WIFI_MAX_NETWORKS = 5;
// Per-network connection attempt timeout.
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;
// Connected network lost for this long -> start the access point as well.
constexpr uint32_t WIFI_LOST_FALLBACK_MS = 30000;
// While running as an access point with saved networks, look for one of
// them this often - but only while nobody is connected to the AP, because
// the scan and the channel change on joining would disrupt them.
constexpr uint32_t WIFI_AP_RETRY_INTERVAL_MS = 60000;
// After joining a network, keep the fallback AP up until it has had no
// clients for this long, so a page open on the AP isn't cut off mid-use.
constexpr uint32_t WIFI_AP_IDLE_SHUTDOWN_MS = 10000;

// ----------------------------------------------
// Battery (3S Li-ion 18650 pack, measured by both motor drivers)
// ----------------------------------------------
constexpr uint8_t BATTERY_CELLS = 3;
// Voltage must recover to cutoff + this before balancing is allowed again.
constexpr float BATTERY_HYSTERESIS_V = 0.3f;
constexpr uint32_t BATTERY_CHECK_INTERVAL_MS = 1000;
// A driver's voltage reading older than this is ignored (driver offline).
constexpr uint32_t BATTERY_READING_MAX_AGE_MS = 1000;
