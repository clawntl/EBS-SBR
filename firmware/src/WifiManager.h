#pragma once

#include <Arduino.h>
#include <vector>

// WiFi connection management.
//
// "Access point only" mode (saved in NVS, on by default - see Config.h):
// never join a network, always host the robot's own access point. This is
// how the robot is used at open days.
//
// Otherwise - boot: scan, then try the saved networks that are in range,
// strongest first. If none connects, host an access point named from the MAC
// (see Config.h), with a captive portal.
//
// Runtime:
//  - connected network lost for WIFI_LOST_FALLBACK_MS -> start the AP too
//  - running as AP with saved networks -> rescan every
//    WIFI_AP_RETRY_INTERVAL_MS while no one is connected to the AP
//  - joined a network while the AP is up -> drop the AP once it has had no
//    clients for WIFI_AP_IDLE_SHUTDOWN_MS
//
// begin() and loop() must be called from one task (the network task). The
// other functions are safe to call from any task.
namespace WifiManager {

struct Status {
    bool connected;        // joined a saved network
    String ssid;           // network joined (or being tried)
    String ip;             // station IP if connected
    int8_t rssi;           // dBm, when connected
    bool connecting;       // an attempt is in progress
    bool apActive;         // our access point is up
    String apSsid;
    String apIp;
    uint8_t apClients;
    bool apOnly;           // "access point only" mode
};

struct ScanResult {
    String ssid;
    int8_t rssi;
    bool open;
    bool saved;
};

void begin();
void loop();

Status status();
// Changes whenever status, saved networks or scan results change, so the
// web page can be updated only when needed.
uint32_t version();

const String &apSsid();
const String &apPassword();
// "balancebot-3f2a1c": mDNS / OTA / DHCP hostname. Valid from before begin().
const String &hostname();
// The MAC suffix on its own, e.g. "3f2a1c".
const String &suffix();
// Works out the names above. Called by begin(); may be called earlier.
void makeNames();
// True when the request arrived through our access point (vs. the home
// network), so captive-portal redirects only apply there.
bool isApAddress(const IPAddress &localIp);

// Saved networks. Passwords are write-only: they are never returned.
std::vector<String> savedNetworks();
bool add(const char *ssid, const char *password);   // replaces an existing entry
bool remove(const char *ssid);
bool remove(uint8_t index);
void clear();
void list(Print &out);

// Access point only. setApOnly() saves to flash (stalls both cores briefly),
// so call it from the network task, not the async web server task.
bool apOnly();
bool setApOnly(bool on);

// Asynchronous actions, carried out by loop().
void requestScan();
void requestConnect();   // try the saved networks now (ignored in AP-only mode)
bool scanRunning();
std::vector<ScanResult> scanResults();

} // namespace WifiManager
