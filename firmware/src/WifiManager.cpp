#include "WifiManager.h"

#include <Preferences.h>
#include <WiFi.h>
#include <algorithm>
#include <atomic>
#include <esp_mac.h>
#include <mutex>

#include "Config.h"
#include "DeviceLog.h"

namespace {

constexpr const char *NVS_NAMESPACE = "wifi";
const IPAddress AP_IP(AP_IP_OCTETS);
const IPAddress AP_MASK(255, 255, 255, 0);
// While a connected network is lost, ask the driver to rejoin this often.
constexpr uint32_t RECONNECT_INTERVAL_MS = 5000;
// Ignore RSSI wobble smaller than this when deciding whether to notify.
constexpr int8_t RSSI_NOTIFY_DELTA = 3;

struct Credentials {
    String ssid;
    String password;
};

enum class State : uint8_t { Disconnected, Connecting, Connected };

// --- Owned by the network task (begin/loop) ---
State state = State::Disconnected;
bool apActive = false;
bool scanning = false;
bool scanForConnect = false;
std::vector<Credentials> candidates;
size_t nextCandidate = 0;
String attemptSsid;
uint32_t attemptDeadlineMs = 0;
uint32_t lostSinceMs = 0;
uint32_t lastReconnectMs = 0;
uint32_t lastApRetryMs = 0;
uint32_t apIdleSinceMs = 0;

// --- Shared with other tasks ---
std::mutex mtx;               // guards the members below and NVS read-modify-write
WifiManager::Status published;
std::vector<WifiManager::ScanResult> results;
String apSsidStr, apPasswordStr, hostnameStr, suffixStr;
std::atomic<bool> apOnlyMode{WIFI_AP_ONLY_DEFAULT};
std::atomic<uint32_t> changeCounter{0};
std::atomic<bool> scanRequested{false};
std::atomic<bool> connectRequested{false};
std::atomic<bool> scanBusy{false};

void bump() { changeCounter++; }

// ------------------------------------------------------------------
// NVS storage (namespace "wifi"). Callers hold mtx.
// ------------------------------------------------------------------
std::vector<Credentials> loadAll() {
    std::vector<Credentials> out;
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) return out; // nothing saved yet
    const int n = constrain(prefs.getInt("numNetworks", 0), 0, (int)WIFI_MAX_NETWORKS);
    for (int i = 0; i < n; i++) {
        String ssid = prefs.getString(("ssid_" + String(i)).c_str(), "");
        if (ssid.isEmpty()) continue;
        out.push_back({ssid, prefs.getString(("pass_" + String(i)).c_str(), "")});
    }
    prefs.end();
    return out;
}

bool loadApOnly() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) return WIFI_AP_ONLY_DEFAULT;
    const bool on = prefs.getBool("apOnly", WIFI_AP_ONLY_DEFAULT);
    prefs.end();
    return on;
}

bool saveAll(const std::vector<Credentials> &list) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return false;
    // Not prefs.clear(): that would also wipe the AP-only setting.
    for (uint8_t i = 0; i < WIFI_MAX_NETWORKS; i++) {
        prefs.remove(("ssid_" + String(i)).c_str());
        prefs.remove(("pass_" + String(i)).c_str());
    }
    for (size_t i = 0; i < list.size(); i++) {
        prefs.putString(("ssid_" + String(i)).c_str(), list[i].ssid);
        prefs.putString(("pass_" + String(i)).c_str(), list[i].password);
    }
    prefs.putInt("numNetworks", list.size());
    prefs.end();
    return true;
}

// ------------------------------------------------------------------
// Access point
// ------------------------------------------------------------------
void startAp() {
    if (apActive) return;
    // AP+STA, so we can keep scanning for / joining saved networks while
    // the access point is up.
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apSsidStr.c_str(), apPasswordStr.c_str());
    delay(100); // let the AP interface come up before reconfiguring it
    // Gateway = our IP, so DHCP clients use us as router *and* DNS server
    // (needed for the captive portal).
    WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);
    apActive = true;
    apIdleSinceMs = millis();
    DeviceLog::printf("[wifi] AP \"%s\" (password \"%s\") up at %s\n", apSsidStr.c_str(),
                  apPasswordStr.c_str(), WiFi.softAPIP().toString().c_str());
    // Paste this into any QR generator to make a scan-to-join code.
    DeviceLog::printf("[wifi] QR: WIFI:T:WPA;S:%s;P:%s;;\n", apSsidStr.c_str(), apPasswordStr.c_str());
    bump();
}

void stopAp() {
    if (!apActive) return;
    if (apOnlyMode) return; // never in AP-only mode
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apActive = false;
    DeviceLog::println("[wifi] AP stopped (on home network, no AP clients)");
    bump();
}

// ------------------------------------------------------------------
// Scanning and connecting
// ------------------------------------------------------------------
void startScan(bool forConnect) {
    WiFi.scanDelete();
    if (WiFi.scanNetworks(true /*async*/) == WIFI_SCAN_FAILED) {
        DeviceLog::println("[wifi] scan failed to start");
        return;
    }
    scanning = true;
    scanBusy = true;
    scanForConnect = forConnect;
    bump();
}

void giveUpConnecting() {
    state = State::Disconnected;
    attemptSsid = "";
    startAp(); // no-op if already up
    lastApRetryMs = millis();
    bump();
}

void connectNext() {
    if (apOnlyMode) {
        candidates.clear();
        giveUpConnecting();
        return;
    }
    if (nextCandidate >= candidates.size()) {
        if (!candidates.empty()) DeviceLog::println("[wifi] no saved network could be joined");
        giveUpConnecting();
        return;
    }
    const Credentials &c = candidates[nextCandidate++];
    DeviceLog::printf("[wifi] trying \"%s\"\n", c.ssid.c_str());
    WiFi.disconnect();
    WiFi.begin(c.ssid.c_str(), c.password.c_str());
    attemptSsid = c.ssid;
    attemptDeadlineMs = millis() + WIFI_CONNECT_TIMEOUT_MS;
    state = State::Connecting;
    bump();
}

void finishScan(int n) {
    scanning = false;
    scanBusy = false;

    std::vector<Credentials> saved;
    {
        std::lock_guard<std::mutex> lock(mtx);
        saved = loadAll();
    }
    auto isSaved = [&](const String &ssid) {
        return std::any_of(saved.begin(), saved.end(), [&](const Credentials &c) { return c.ssid == ssid; });
    };

    // One entry per SSID (strongest access point), strongest first.
    std::vector<WifiManager::ScanResult> found;
    for (int i = 0; i < n; i++) {
        const String ssid = WiFi.SSID(i);
        if (ssid.isEmpty()) continue; // hidden network
        const int8_t rssi = WiFi.RSSI(i);
        auto it = std::find_if(found.begin(), found.end(), [&](const WifiManager::ScanResult &r) { return r.ssid == ssid; });
        if (it == found.end()) {
            found.push_back({ssid, rssi, WiFi.encryptionType(i) == WIFI_AUTH_OPEN, isSaved(ssid)});
        } else if (rssi > it->rssi) {
            it->rssi = rssi;
        }
    }
    WiFi.scanDelete();
    std::sort(found.begin(), found.end(), [](const auto &a, const auto &b) { return a.rssi > b.rssi; });
    {
        std::lock_guard<std::mutex> lock(mtx);
        results = found;
    }
    bump();

    if (!scanForConnect) return;
    scanForConnect = false;

    // Try only saved networks that are actually in range, strongest first,
    // instead of waiting out a timeout for each one that isn't.
    candidates.clear();
    nextCandidate = 0;
    for (const auto &r : found) {
        for (const auto &c : saved) {
            if (c.ssid == r.ssid) candidates.push_back(c);
        }
    }
    if (candidates.empty()) DeviceLog::println("[wifi] no saved network in range");
    connectNext();
}

void publishStatus() {
    WifiManager::Status s;
    s.connected = state == State::Connected && WiFi.status() == WL_CONNECTED;
    s.connecting = state == State::Connecting;
    s.ssid = state == State::Disconnected ? String() : attemptSsid;
    s.ip = s.connected ? WiFi.localIP().toString() : String();
    s.rssi = s.connected ? WiFi.RSSI() : 0;
    s.apActive = apActive;
    s.apSsid = apSsidStr;
    s.apIp = apActive ? WiFi.softAPIP().toString() : String();
    s.apClients = apActive ? WiFi.softAPgetStationNum() : 0;
    s.apOnly = apOnlyMode;

    std::lock_guard<std::mutex> lock(mtx);
    const bool changed = s.connected != published.connected || s.connecting != published.connecting ||
                         s.ssid != published.ssid || s.ip != published.ip || s.apActive != published.apActive ||
                         s.apClients != published.apClients || s.apOnly != published.apOnly ||
                         abs(s.rssi - published.rssi) >= RSSI_NOTIFY_DELTA;
    if (changed) {
        published = s;
        bump();
    }
}

} // namespace

namespace WifiManager {

void makeNames() {
    if (!suffixStr.isEmpty()) return;
    // The soft-AP MAC is what phones see as the BSSID, and it's readable from
    // eFuse before the WiFi driver starts, so the names are fixed for the
    // life of the chip (and match the printed QR signs from tools/wifi_qr.py).
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char sfx[13] = {0};
    for (size_t i = 6 - AP_MAC_SUFFIX_BYTES; i < 6; i++) {
        snprintf(sfx + 2 * (i - (6 - AP_MAC_SUFFIX_BYTES)), 3, "%02x", mac[i]);
    }
    suffixStr = sfx;
    apSsidStr = String(AP_SSID_PREFIX) + sfx;
    apPasswordStr = String(AP_PASSWORD_PREFIX) + sfx;
    hostnameStr = String(HOSTNAME_PREFIX) + sfx;
}

void begin() {
    makeNames();
    bool haveSaved;
    {
        std::lock_guard<std::mutex> lock(mtx);
        apOnlyMode = loadApOnly();
        haveSaved = !loadAll().empty();
    }
    WiFi.persistent(false);      // credentials live in our own NVS namespace
    WiFi.setAutoReconnect(false); // we manage reconnection ourselves
    WiFi.setHostname(hostnameStr.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // lower WebSocket latency

    if (apOnlyMode) {
        DeviceLog::println("[wifi] access point only mode");
        startAp();
        lastApRetryMs = millis();
    } else if (haveSaved) {
        startScan(true);
    } else {
        DeviceLog::println("[wifi] no saved networks");
        startAp();
    }
    publishStatus();
}

void loop() {
    const uint32_t now = millis();

    if (scanning) {
        const int n = WiFi.scanComplete();
        if (n != WIFI_SCAN_RUNNING) finishScan(n < 0 ? 0 : n);
    }

    // Requests from the web page / serial. Not while busy: they'd cancel a
    // scan or attempt in progress.
    if (!scanning && state != State::Connecting) {
        if (connectRequested.exchange(false) && !apOnlyMode) startScan(true);
        else if (scanRequested.exchange(false)) startScan(false);
    }

    switch (state) {
    case State::Connecting:
        if (WiFi.status() == WL_CONNECTED) {
            state = State::Connected;
            lostSinceMs = 0;
            apIdleSinceMs = now;
            DeviceLog::printf("[wifi] connected to \"%s\", IP %s, http://%s.local/\n", attemptSsid.c_str(),
                          WiFi.localIP().toString().c_str(), hostnameStr.c_str());
            // A home network on the same /24 as our fallback AP would clash
            // with it (two different devices at the same address).
            {
                const IPAddress ip = WiFi.localIP();
                if (ip[0] == AP_IP[0] && ip[1] == AP_IP[1] && ip[2] == AP_IP[2]) {
                    DeviceLog::printf("[wifi] WARNING: this network uses %u.%u.%u.x, the same range as the "
                                      "robot's own access point - change AP_IP_OCTETS in Config.h\n",
                                      ip[0], ip[1], ip[2]);
                }
            }
            bump();
        } else if ((int32_t)(now - attemptDeadlineMs) > 0) {
            DeviceLog::printf("[wifi] \"%s\" timed out\n", attemptSsid.c_str());
            connectNext();
        }
        break;

    case State::Connected:
        if (WiFi.status() != WL_CONNECTED) {
            if (!lostSinceMs) {
                lostSinceMs = lastReconnectMs = now;
                DeviceLog::printf("[wifi] lost \"%s\", reconnecting\n", attemptSsid.c_str());
                WiFi.reconnect();
            } else if (now - lostSinceMs > WIFI_LOST_FALLBACK_MS) {
                DeviceLog::println("[wifi] network gone - falling back to access point");
                WiFi.disconnect();
                giveUpConnecting();
            } else if (now - lastReconnectMs > RECONNECT_INTERVAL_MS) {
                lastReconnectMs = now;
                WiFi.reconnect();
            }
        } else {
            if (lostSinceMs) {
                lostSinceMs = 0;
                DeviceLog::println("[wifi] reconnected");
            }
            // Drop the fallback AP once nobody is using it.
            if (apActive) {
                if (WiFi.softAPgetStationNum() > 0) apIdleSinceMs = now;
                else if (now - apIdleSinceMs > WIFI_AP_IDLE_SHUTDOWN_MS) stopAp();
            }
        }
        break;

    case State::Disconnected:
        // Look for a saved network again, but only while nobody is on the
        // AP: scanning hops channels and joining may move the AP's channel.
        if (!apOnlyMode && apActive && !scanning && now - lastApRetryMs > WIFI_AP_RETRY_INTERVAL_MS &&
            WiFi.softAPgetStationNum() == 0) {
            lastApRetryMs = now;
            bool haveSaved;
            {
                std::lock_guard<std::mutex> lock(mtx);
                haveSaved = !loadAll().empty();
            }
            if (haveSaved) startScan(true);
        }
        break;
    }

    publishStatus();
}

Status status() {
    std::lock_guard<std::mutex> lock(mtx);
    return published;
}

uint32_t version() { return changeCounter; }

const String &apSsid() { return apSsidStr; }
const String &apPassword() { return apPasswordStr; }
const String &hostname() { return hostnameStr; }
const String &suffix() { return suffixStr; }

bool apOnly() { return apOnlyMode; }

bool setApOnly(bool on) {
    bool ok;
    {
        std::lock_guard<std::mutex> lock(mtx);
        Preferences prefs;
        ok = prefs.begin(NVS_NAMESPACE, false);
        if (ok) {
            ok = prefs.putBool("apOnly", on) == 1;
            prefs.end();
        }
    }
    if (on == apOnlyMode) return ok;
    apOnlyMode = on;
    if (on) {
        // Drop any network connection (or attempt) and make sure the AP is up.
        DeviceLog::println("[wifi] access point only mode on - leaving any network");
        scanForConnect = false;
        candidates.clear();
        if (state != State::Disconnected) WiFi.disconnect();
        state = State::Disconnected;
        attemptSsid = "";
        lostSinceMs = 0;
        startAp();
        lastApRetryMs = millis();
    } else {
        DeviceLog::println("[wifi] access point only mode off - looking for saved networks");
        connectRequested = true;
    }
    bump();
    return ok;
}

bool isApAddress(const IPAddress &localIp) {
    // Only while our AP is actually up: with it down, no request can have
    // come through it, whatever address it was sent to.
    return apActive && localIp == AP_IP;
}

std::vector<String> savedNetworks() {
    std::lock_guard<std::mutex> lock(mtx);
    std::vector<String> out;
    for (const auto &c : loadAll()) out.push_back(c.ssid);
    return out;
}

bool add(const char *ssid, const char *password) {
    if (!ssid || !*ssid || strlen(ssid) > 32) return false;
    if (password && strlen(password) > 63) return false;
    bool ok;
    {
        std::lock_guard<std::mutex> lock(mtx);
        auto list = loadAll();
        auto it = std::find_if(list.begin(), list.end(), [&](const Credentials &c) { return c.ssid == ssid; });
        if (it != list.end()) {
            it->password = password ? password : "";
        } else if (list.size() >= WIFI_MAX_NETWORKS) {
            return false;
        } else {
            list.push_back({ssid, password ? password : ""});
        }
        ok = saveAll(list);
        for (auto &r : results) if (r.ssid == ssid) r.saved = true;
    }
    bump();
    return ok;
}

bool remove(const char *ssid) {
    bool ok = false;
    {
        std::lock_guard<std::mutex> lock(mtx);
        auto list = loadAll();
        auto it = std::find_if(list.begin(), list.end(), [&](const Credentials &c) { return c.ssid == ssid; });
        if (it != list.end()) {
            list.erase(it);
            ok = saveAll(list);
            for (auto &r : results) if (r.ssid == ssid) r.saved = false;
        }
    }
    bump();
    return ok;
}

bool remove(uint8_t index) {
    String ssid;
    {
        std::lock_guard<std::mutex> lock(mtx);
        auto list = loadAll();
        if (index >= list.size()) return false;
        ssid = list[index].ssid;
    }
    return remove(ssid.c_str());
}

void clear() {
    {
        std::lock_guard<std::mutex> lock(mtx);
        saveAll({});
        for (auto &r : results) r.saved = false;
    }
    bump();
}

void list(Print &out) {
    const auto saved = savedNetworks();
    if (saved.empty()) out.println("No saved networks.");
    for (size_t i = 0; i < saved.size(); i++) out.printf("[%u] %s\n", (unsigned)i, saved[i].c_str());
}

void requestScan() { scanRequested = true; }
void requestConnect() { connectRequested = true; }
bool scanRunning() { return scanBusy || scanRequested; }

std::vector<ScanResult> scanResults() {
    std::lock_guard<std::mutex> lock(mtx);
    return results;
}

} // namespace WifiManager
