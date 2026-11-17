#include "WebInterface.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <memory>
#include <mutex>
#include <vector>

#include "Battery.h"
#include "CANConfig.h"
#include "Capture.h"
#include "Config.h"
#include "DeviceLog.h"
#include "DriveInput.h"
#include "Gamepad.h"
#include "LQRControl.h"
#include "MotorDrivers.h"
#include "RobotParams.h"
#include "Settings.h"
#include "WifiManager.h"
#include "WebIndexHtml.h" // generated from data/index.html by scripts/embed_web.py

namespace {

constexpr uint16_t DNS_PORT = 53;
const char *PORTAL_HOST = AP_IP_STRING;
const char *PORTAL_URL = "http://" AP_IP_STRING "/";
// Settings are re-broadcast when they change (from any client or serial),
// but no more often than this while a slider is being dragged.
constexpr uint32_t SETTINGS_BROADCAST_MIN_MS = 100;
constexpr uint32_t WIFI_BROADCAST_MIN_MS = 250;
constexpr uint32_t CLEANUP_INTERVAL_MS = 1000;

DNSServer dnsServer;
bool dnsRunning = false;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Flash writes (settings, saved networks) are slow and stall both cores, so
// the WebSocket handler (in the async_tcp task) only queues them; the
// network task does the work.
struct PendingAction {
    enum Kind : uint8_t { Save, Load, Defaults, WifiAdd, WifiRemove, ApOnlyOn, ApOnlyOff } kind;
    String ssid;
    String password;
};
std::mutex pendingMutex;
std::vector<PendingAction> pending;

void queueAction(PendingAction a) {
    std::lock_guard<std::mutex> lock(pendingMutex);
    if (pending.size() < 8) pending.push_back(std::move(a));
}

// ---------------------------------------------------------------------
// Outgoing messages
// ---------------------------------------------------------------------

void sendJson(const JsonDocument &doc, AsyncWebSocketClient *client = nullptr) {
    String out;
    serializeJson(doc, out);
    if (client) client->text(out);
    else ws.textAll(out);
}

void sendInfo(AsyncWebSocketClient *client) {
    JsonDocument doc;
    doc["type"] = "info";
    doc["name"] = WifiManager::apSsid();
    doc["hostname"] = WifiManager::hostname();
    doc["suffix"] = WifiManager::suffix();
    // Same as on the printed QR sign; anyone seeing this page has joined already.
    doc["apPassword"] = WifiManager::apPassword();
    doc["build"] = __DATE__ " " __TIME__;
    doc["psram"] = ESP.getPsramSize();
    sendJson(doc, client);
}

void sendSchema(AsyncWebSocketClient *client) {
    JsonDocument doc;
    doc["type"] = "schema";
    JsonArray arr = doc["params"].to<JsonArray>();
    size_t n;
    const ParamDef *defs = Settings::params(n);
    for (size_t i = 0; i < n; i++) {
        JsonObject p = arr.add<JsonObject>();
        p["key"] = defs[i].key;
        p["label"] = defs[i].label;
        p["group"] = defs[i].group;
        p["min"] = defs[i].min;
        p["max"] = defs[i].max;
        p["step"] = defs[i].step;
        p["default"] = defs[i].defaultValue;
        if (defs[i].rebootRequired) p["reboot"] = true;
        if (defs[i].help) p["help"] = defs[i].help;
        if (defs[i].flags & PARAM_FLAG_FINE) p["fine"] = true;
        if (defs[i].flags & PARAM_FLAG_UNUSED) p["unused"] = true;
        if (defs[i].flags & PARAM_FLAG_HIDDEN) p["hidden"] = true;
    }
    sendJson(doc, client);
}

void sendSettings(AsyncWebSocketClient *client = nullptr) {
    JsonDocument doc;
    doc["type"] = "settings";
    JsonObject values = doc["values"].to<JsonObject>();
    size_t n;
    const ParamDef *defs = Settings::params(n);
    for (size_t i = 0; i < n; i++) values[defs[i].key] = Settings::valueAt(i);
    doc["dirty"] = Settings::isDirty();
    sendJson(doc, client);
}

void sendWifi(AsyncWebSocketClient *client = nullptr) {
    const WifiManager::Status st = WifiManager::status();
    JsonDocument doc;
    doc["type"] = "wifi";
    JsonObject s = doc["status"].to<JsonObject>();
    s["connected"] = st.connected;
    s["connecting"] = st.connecting;
    s["ssid"] = st.ssid;
    s["ip"] = st.ip;
    s["rssi"] = st.rssi;
    s["apActive"] = st.apActive;
    s["apSsid"] = st.apSsid;
    s["apIp"] = st.apIp;
    s["apClients"] = st.apClients;
    s["apOnly"] = st.apOnly;
    s["hostname"] = WifiManager::hostname();

    JsonArray saved = doc["saved"].to<JsonArray>();
    for (const String &ssid : WifiManager::savedNetworks()) saved.add(ssid);
    doc["max"] = WIFI_MAX_NETWORKS;

    doc["scanning"] = WifiManager::scanRunning();
    JsonArray nets = doc["networks"].to<JsonArray>();
    for (const auto &r : WifiManager::scanResults()) {
        JsonObject o = nets.add<JsonObject>();
        o["ssid"] = r.ssid;
        o["rssi"] = r.rssi;
        o["open"] = r.open;
        o["saved"] = r.saved;
    }
    sendJson(doc, client);
}

// Device log lines with seq > after, as one message. Sent as the backlog
// to a newly connected page, then incrementally to everyone.
void sendLog(uint32_t after, AsyncWebSocketClient *client = nullptr) {
    // Random per boot, so the page can tell a restart (sequence numbers
    // start again) from a reconnect (backlog re-sent).
    static const uint32_t bootId = esp_random();
    JsonDocument doc;
    doc["type"] = "log";
    doc["boot"] = bootId;
    JsonArray lines = doc["lines"].to<JsonArray>();
    DeviceLog::since(after, [&](uint32_t seq, uint32_t ms, const char *text) {
        JsonObject l = lines.add<JsonObject>();
        l["seq"] = seq;
        l["ms"] = ms;
        l["text"] = text;
    });
    if (lines.size()) sendJson(doc, client);
}

// Motor driver firmware/settings (Drivers tab).
void sendDrivers(AsyncWebSocketClient *client = nullptr) {
    JsonDocument doc;
    doc["type"] = "drivers";
    doc["job"] = driverJobText();
    doc["balancing"] = balanceState == BalanceState::Balancing;
    JsonArray heard = doc["heard"].to<JsonArray>(); // node IDs heard on the bus (0 = old "right" board)
    const uint16_t bits = nodesHeard();
    for (int n = 0; n <= NODE_ID_MAX; n++) if (bits & (1u << n)) heard.add(n);
    JsonArray arr = doc["d"].to<JsonArray>();
    for (int s = 0; s < 2; s++) {
        const DriverConfigView c = getDriverConfig((MotorSide)s);
        const MotorDriverInfo m = getMotorDriverInfo((MotorSide)s);
        JsonObject o = arr.add<JsonObject>();
        o["link"] = motorLinkName(m.link);
        o["node"] = driverNode((MotorSide)s);
        o["known"] = c.infoKnown;
        o["old"] = c.noConfigSupport;
        if (c.infoKnown) {
            o["proto"] = c.protocol;
            o["fw"] = String(c.fwMajor) + "." + c.fwMinor + "." + c.fwPatch;
            o["kb"] = c.imageKB;
        }
        if (c.activeKnown) {
            JsonObject a = o["active"].to<JsonObject>();
            a["node"] = c.node;
            a["invert"] = (c.flags & CFG_FLAG_INVERT) != 0;
            a["cal"] = (c.flags & CFG_FLAG_CALIBRATION) != 0;
            a["ccw"] = (c.flags & CFG_FLAG_SENSOR_DIR_CCW) != 0;
            a["zero"] = c.zeroAngle;
            o["unsaved"] = (c.flags & CFG_FLAG_UNSAVED) != 0;
        }
        if (c.pendingKnown) {
            JsonObject p = o["pending"].to<JsonObject>();
            p["node"] = c.pendNode;
            p["invert"] = (c.pendFlags & CFG_FLAG_INVERT) != 0;
            p["cal"] = (c.pendFlags & CFG_FLAG_CALIBRATION) != 0;
            o["unsaved"] = (c.pendFlags & CFG_FLAG_UNSAVED) != 0;
        }
        o["calUsed"] = (m.flags & DRIVER_FLAG_CALIBRATION_USED) != 0;
        o["result"] = c.result;
    }
    sendJson(doc, client);
}

// BLE Xbox controller status (Control tab).
void sendGamepad(AsyncWebSocketClient *client = nullptr) {
    const Gamepad::Status g = Gamepad::status();
    JsonDocument doc;
    doc["type"] = "gamepad";
    doc["state"] = Gamepad::stateName(g.state);
    doc["addr"] = g.address;
    doc["left"] = g.pairSecondsLeft;
    doc["active"] = g.inputActive;
    if (g.battery >= 0) doc["battery"] = g.battery;
    sendJson(doc, client);
}

void sendNotice(const String &text, bool ok = true) {
    JsonDocument doc;
    doc["type"] = "notice";
    doc["text"] = text;
    doc["ok"] = ok;
    sendJson(doc);
}

// Rounds to a fixed number of decimals so the JSON stays short.
float r2(float v) { return roundf(v * 100.0f) / 100.0f; }
float r3(float v) { return roundf(v * 1000.0f) / 1000.0f; }

void broadcastTelemetry() {
    uint16_t loopHz = 0;
    const ControlSample c = getControlSnapshot(&loopHz);
    JsonDocument doc;
    doc["type"] = "telemetry";
    doc["t"] = millis();
    doc["state"] = balanceStateName(c.state);
    doc["paused"] = paused;
    doc["tilt"] = r2(c.tilt);
    doc["rate"] = r2(c.tiltRate);
    doc["sp"] = r2(c.setpoint);
    doc["dev"] = r2(c.tilt - c.setpoint);
    doc["phi"] = r2(c.phi);
    doc["wheel"] = r2(c.phiDot);
    doc["wl"] = r2(c.wheelL);
    doc["wr"] = r2(c.wheelR);
    doc["yaw"] = r2(c.yawRate);
    doc["lqr"] = r3(c.lqr);
    doc["cl"] = r3(c.curL);
    doc["cr"] = r3(c.curR);
    doc["jx"] = r2(c.joyX);
    doc["jy"] = r2(c.joyY);
    doc["perr"] = r3(c.posErr);
    doc["vref"] = r2(c.speedRef);
    doc["hdg"] = r2(c.heading);
    doc["herr"] = r2(c.headErr);
    doc["yref"] = r2(c.yawRateRef);
    doc["ycur"] = r3(c.yawCur);
    doc["pitch"] = r2(c.pitch);
    doc["gx"] = r2(c.gyro[0]);
    doc["gy"] = r2(c.gyro[1]);
    doc["gz"] = r2(c.gyro[2]);
    doc["ax"] = r3(c.accel[0]);
    doc["ay"] = r3(c.accel[1]);
    doc["az"] = r3(c.accel[2]);
    doc["hz"] = loopHz;
    {
        const DriveInput::Demand dd = DriveInput::lastUsed();
        doc["src"] = DriveInput::sourceName(dd.source);
        if (dd.source == DriveInput::Source::Module) doc["srcMod"] = dd.module;
    }

    doc["batt"] = r2(Battery::voltage());
    doc["battL"] = r2(Battery::leftVoltage());
    doc["battR"] = r2(Battery::rightVoltage());
    doc["battKnown"] = Battery::known();
    doc["battPct"] = Battery::percent();
    doc["battMins"] = Battery::minutesRemaining(); // -1 estimating, -2 not discharging
    doc["battLow"] = Battery::isLow();

    JsonArray drv = doc["drv"].to<JsonArray>();
    for (int s = 0; s < 2; s++) {
        const MotorDriverInfo m = getMotorDriverInfo((MotorSide)s);
        JsonObject d = drv.add<JsonObject>();
        d["link"] = motorLinkName(m.link);
        d["flags"] = m.flags;
        d["tries"] = m.startAttempts;
        if (m.lastError) {
            d["err"] = driverErrorName(m.lastError);
            d["errAge"] = m.lastErrorAgeMs / 1000;
        }
    }

    JsonObject cap = doc["cap"].to<JsonObject>();
    cap["state"] = Capture::stateName(Capture::state());
    cap["n"] = Capture::samples();
    cap["max"] = Capture::capacity();
    cap["hz"] = Capture::rateHz();
    sendJson(doc);
}

// ---------------------------------------------------------------------
// Incoming messages (run in the async_tcp task)
// ---------------------------------------------------------------------

void handleJoystick(float x, float y) {
    if (!isfinite(x) || !isfinite(y)) x = y = 0;
    DriveInput::setWeb(x, y);
}

void handleMessage(const uint8_t *data, size_t len) {
    JsonDocument doc;
    if (deserializeJson(doc, data, len)) return;
    const char *cmd = doc["cmd"];
    if (!cmd) return;

    if (!strcmp(cmd, "joystick")) { // most frequent first
        handleJoystick(doc["x"].as<float>(), doc["y"].as<float>());
    } else if (!strcmp(cmd, "set")) {
        // The network task notices the version change and re-broadcasts
        // the settings, so every open page stays in sync.
        if (!Settings::set(doc["key"], doc["value"].as<float>())) sendNotice("Unknown setting", false);
    } else if (!strcmp(cmd, "pause")) {
        const bool p = doc["paused"].as<bool>();
        if (p != paused) DeviceLog::printf("[web] %s\n", p ? "paused" : "resumed");
        paused = p;
    } else if (!strcmp(cmd, "save")) {
        queueAction({PendingAction::Save, "", ""});
    } else if (!strcmp(cmd, "load")) {
        queueAction({PendingAction::Load, "", ""});
    } else if (!strcmp(cmd, "defaults")) {
        queueAction({PendingAction::Defaults, "", ""});
    } else if (!strcmp(cmd, "capture")) {
        const bool onBalance = !strcmp(doc["trigger"] | "now", "balance");
        const bool ok = Capture::start(doc["seconds"] | 10, onBalance ? Capture::Trigger::NextBalance : Capture::Trigger::Now);
        if (!ok) sendNotice("Couldn't start recording (out of memory, or a download is running)", false);
    } else if (!strcmp(cmd, "captureCancel")) {
        Capture::cancel();
    } else if (!strcmp(cmd, "motorRetry")) {
        resetMotorStartAttempts();
        DeviceLog::println("[web] motor start attempts reset");
    } else if (!strcmp(cmd, "driver")) {
        const int side = doc["side"] | -1;
        const char *op = doc["op"] | "";
        const char *err = "Unknown driver operation";
        if (!strcmp(op, "swap")) {
            err = requestSwapDriverSides();
        } else if (!strcmp(op, "nodes")) {
            err = requestDriverNodes(doc["left"] | 0, doc["right"] | 0);
        } else if (side == 0 || side == 1) {
            const MotorSide sd = (MotorSide)side;
            const float v = doc["value"] | 0.0f;
            if      (!strcmp(op, "refresh"))  err = requestDriverOp(sd, DriverOp::Refresh);
            else if (!strcmp(op, "invert"))   err = requestDriverOp(sd, DriverOp::SetInvert, v);
            else if (!strcmp(op, "calStore")) err = requestDriverOp(sd, DriverOp::CalStore);
            else if (!strcmp(op, "calClear")) err = requestDriverOp(sd, DriverOp::CalClear);
            else if (!strcmp(op, "save"))     err = requestDriverOp(sd, DriverOp::Save);
            else if (!strcmp(op, "revert"))   err = requestDriverOp(sd, DriverOp::Revert);
            else if (!strcmp(op, "reboot"))   err = requestDriverOp(sd, DriverOp::Reboot);
            else if (!strcmp(op, "recal"))    err = requestDriverOp(sd, DriverOp::Recalibrate);
            else if (!strcmp(op, "setNode"))  err = requestDriverOp(sd, DriverOp::SetNodeId, v);
        }
        if (err) sendNotice(err, false);
    } else if (!strcmp(cmd, "spinTest")) {
        const char *err = requestSpinTest(doc["amps"] | 0.1f, doc["ms"] | 200);
        sendNotice(err ? err : "Spin test running - recording", !err);
    } else if (!strcmp(cmd, "recSecs")) {
        // Saved on its own (not part of Save to flash), unless balancing:
        // a flash write briefly stalls the control loop.
        if (Settings::set("recSecs", doc["value"] | 10.0f) && balanceState != BalanceState::Balancing)
            Settings::saveKey("recSecs");
    } else if (!strcmp(cmd, "gpPair")) {
        if (!Gamepad::enabled) sendNotice("Xbox controller support is off (Tune tab: Xbox controller)", false);
        else Gamepad::requestPairing();
    } else if (!strcmp(cmd, "gpForget")) {
        Gamepad::requestForget();
    } else if (!strcmp(cmd, "wifiScan")) {
        WifiManager::requestScan();
    } else if (!strcmp(cmd, "wifiConnect")) {
        if (WifiManager::apOnly()) {
            sendNotice("Access point only is on - turn it off first", false);
        } else {
            WifiManager::requestConnect();
            sendNotice("Looking for saved networks...");
        }
    } else if (!strcmp(cmd, "apOnly")) {
        queueAction({doc["on"].as<bool>() ? PendingAction::ApOnlyOn : PendingAction::ApOnlyOff, "", ""});
    } else if (!strcmp(cmd, "wifiAdd")) {
        queueAction({PendingAction::WifiAdd, doc["ssid"] | "", doc["password"] | ""});
    } else if (!strcmp(cmd, "wifiRemove")) {
        queueAction({PendingAction::WifiRemove, doc["ssid"] | "", ""});
    }
}

void onWsEvent(AsyncWebSocket *, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        DeviceLog::printf("[ws] client #%lu connected from %s\n", (unsigned long)client->id(),
                          client->remoteIP().toString().c_str());
        sendInfo(client);
        sendSchema(client);
        sendSettings(client);
        sendWifi(client);
        sendDrivers(client);
        sendGamepad(client);
        sendLog(0, client); // backlog
    } else if (type == WS_EVT_DISCONNECT) {
        DeviceLog::printf("[ws] client #%lu disconnected\n", (unsigned long)client->id());
        // Stop driving straight away if a page drops - don't wait for
        // Task_LQR's joystick staleness timeout.
        handleJoystick(0, 0);
    } else if (type == WS_EVT_DATA) {
        // Only whole, unfragmented text frames; our messages are tiny.
        const AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            handleMessage(data, len);
        }
    }
}

// ---------------------------------------------------------------------
// HTTP routes
// ---------------------------------------------------------------------

// Captive-portal behaviour only applies to requests that came in through
// our access point; on a home network the page behaves like a normal site.
bool viaAccessPoint(AsyncWebServerRequest *req) {
    return WifiManager::isApAddress(req->client()->localIP());
}

void sendIndex(AsyncWebServerRequest *req) {
    // Logged so you can see from the Log tab whether a device's request
    // reached the robot at all (useful when a phone can't load the page).
    DeviceLog::printf("[http] %s %s%s from %s\n", req->methodToString(), req->host().c_str(), req->url().c_str(),
                      req->client()->remoteIP().toString().c_str());
    if (req->method() == HTTP_HEAD) { // headers only (the library would send the body)
        req->send(200, "text/html", "");
        return;
    }
    // Served from flash (rodata), gzipped. Unlike a LittleFS read this
    // doesn't go through the SPI flash driver, so it never stalls the
    // control tasks on the other core.
    AsyncWebServerResponse *res = req->beginResponse(200, "text/html", WEB_INDEX_HTML_GZ, WEB_INDEX_HTML_GZ_LEN);
    res->addHeader("Content-Encoding", "gzip");
    res->addHeader("Cache-Control", "no-store"); // captive portal browsers cache aggressively
    req->send(res);
}

void redirectToPortal(AsyncWebServerRequest *req) {
    // Logged (at most once a second - phones fire many probes) so a
    // misbehaving redirect can be diagnosed from the Log tab.
    static uint32_t lastLogMs = 0;
    const uint32_t now = millis();
    if (now - lastLogMs >= 1000) {
        lastLogMs = now;
        DeviceLog::printf("[http] captive redirect: %s%s (from %s to %s)\n", req->host().c_str(), req->url().c_str(),
                          req->client()->remoteIP().toString().c_str(), req->client()->localIP().toString().c_str());
    }
    req->redirect(PORTAL_URL);
}

void setupRoutes() {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // OS connectivity-check URLs. On the AP side a redirect makes phones
    // open their sign-in sheet straight onto the page.
    static const char *probes[] = {
        "/generate_204", "/gen_204",                          // Android / ChromeOS
        "/hotspot-detect.html", "/library/test/success.html", // Apple
        "/connecttest.txt", "/ncsi.txt", "/redirect",         // Windows
        "/canonical.html", "/success.txt",                    // Firefox
        "/chat",                                              // older Android
    };
    for (const char *p : probes) {
        server.on(p, HTTP_ANY, [](AsyncWebServerRequest *req) {
            if (viaAccessPoint(req)) redirectToPortal(req);
            else req->send(404);
        });
    }

    // GET and HEAD (curl -I, link checkers); HEAD gets the headers only.
    server.on("/", HTTP_GET | HTTP_HEAD, [](AsyncWebServerRequest *req) {
        // On the AP, always land on the IP so the page's ws://<host>/ws is
        // consistent whichever hostname the OS probed.
        if (viaAccessPoint(req) && req->host() != PORTAL_HOST) return redirectToPortal(req);
        sendIndex(req);
    });

    // Last recording as CSV, generated on the fly in chunks (it can be a few
    // MB as text, far too big to build in RAM).
    server.on("/capture.csv", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (Capture::state() != Capture::State::Done || Capture::samples() == 0) {
            req->send(409, "text/plain", "No finished recording to download");
            return;
        }
        auto reader = std::make_shared<Capture::CsvReader>();
        AsyncWebServerResponse *res = req->beginChunkedResponse(
            "text/csv", [reader](uint8_t *buf, size_t maxLen, size_t) -> size_t { return reader->read(buf, maxLen); });
        char disposition[96];
        snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s-capture-%lu.csv\"",
                 robotName(), (unsigned long)(millis() / 1000));
        res->addHeader("Content-Disposition", disposition);
        res->addHeader("Cache-Control", "no-store");
        req->send(res);
    });

    // Don't answer the favicon request with a redirect (the page has an inline icon).
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *req) { req->send(204); });

    server.onNotFound([](AsyncWebServerRequest *req) {
        if (viaAccessPoint(req)) redirectToPortal(req);
        else {
            DeviceLog::printf("[http] 404 %s%s from %s\n", req->host().c_str(), req->url().c_str(),
                              req->client()->remoteIP().toString().c_str());
            req->send(404, "text/plain", "Not found");
        }
    });
}

void setupOta() {
    static bool wasPaused = false;
    ArduinoOTA.setHostname(WifiManager::hostname().c_str()); // also starts mDNS
    if (OTA_PASSWORD && *OTA_PASSWORD) ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
        // Writing flash stalls both cores; stop driving the wheels first.
        wasPaused = paused;
        paused = true;
        DeviceLog::println("[ota] update started - balancing paused");
    });
    ArduinoOTA.onEnd([]() { DeviceLog::println("[ota] done, rebooting"); });
    ArduinoOTA.onError([](ota_error_t error) {
        DeviceLog::printf("[ota] error %u\n", error);
        paused = wasPaused;
    });
    ArduinoOTA.begin();
    MDNS.addService("http", "tcp", 80);
}

// The wildcard DNS server (captive portal) runs only while the AP is up.
void updateCaptiveDns(bool apActive) {
    if (apActive && !dnsRunning) {
        // Every name resolves to us. NoError (not NXDOMAIN) keeps clients
        // from treating lookups as failures.
        dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
        dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
        dnsRunning = true;
    } else if (!apActive && dnsRunning) {
        dnsServer.stop();
        dnsRunning = false;
    }
}

void runPendingActions() {
    std::vector<PendingAction> todo;
    {
        std::lock_guard<std::mutex> lock(pendingMutex);
        todo.swap(pending);
    }
    for (const PendingAction &a : todo) {
        switch (a.kind) {
        case PendingAction::Save: {
            const bool ok = Settings::save();
            DeviceLog::println(ok ? "[settings] saved to flash" : "[settings] save FAILED");
            sendNotice(ok ? "Saved to flash" : "Save failed", ok);
            sendSettings(); // dirty flag changed even if no value did
            break;
        }
        case PendingAction::Load:
            Settings::load();
            DeviceLog::println("[settings] reverted to saved");
            sendNotice("Reverted to saved settings");
            sendSettings();
            break;
        case PendingAction::Defaults:
            Settings::resetToDefaults();
            DeviceLog::println("[settings] factory defaults restored (not saved)");
            sendNotice("Defaults restored (not saved)");
            sendSettings();
            break;
        case PendingAction::WifiAdd: {
            const bool ok = WifiManager::add(a.ssid.c_str(), a.password.c_str());
            sendNotice(ok ? "Saved network \"" + a.ssid + "\""
                          : String("Couldn't save network (list full or invalid SSID/password)"), ok);
            break;
        }
        case PendingAction::WifiRemove: {
            const bool ok = WifiManager::remove(a.ssid.c_str());
            sendNotice(ok ? "Removed \"" + a.ssid + "\"" : String("Network not found"), ok);
            break;
        }
        case PendingAction::ApOnlyOn:
        case PendingAction::ApOnlyOff: {
            const bool on = a.kind == PendingAction::ApOnlyOn;
            const bool ok = WifiManager::setApOnly(on);
            sendNotice(!ok ? String("Couldn't save the setting")
                       : on ? String("Access point only: on")
                            : String("Access point only: off - looking for saved networks"), ok);
            break;
        }
        }
    }
}

void networkTask(void *) {
    WifiManager::begin(); // non-blocking: scanning/connecting continues in loop()
    setupRoutes();
    server.begin();
    setupOta();
    DeviceLog::printf("[http] server started (%s on the access point, http://%s.local/ on a network)\n",
                      PORTAL_URL, WifiManager::hostname().c_str());

    uint32_t lastTelemetryMs = 0, lastSettingsMs = 0, lastWifiMs = 0, lastCleanupMs = 0;
    uint32_t seenSettings = Settings::version();
    uint32_t seenWifi = WifiManager::version();
    uint32_t seenDrivers = driverConfigVersion(), lastDriversMs = 0;
    bool sentBalancing = false;
    uint16_t sentHeard = 0;
    uint32_t seenGamepad = Gamepad::version(), lastGamepadMs = 0;
    uint32_t sentLogSeq = DeviceLog::latest(); // new clients get the backlog on connect

    for (;;) {
        const uint32_t now = millis();
        WifiManager::loop();
        updateCaptiveDns(WifiManager::status().apActive);
        if (dnsRunning) dnsServer.processNextRequest(); // no-op on core 3.x (async)
        ArduinoOTA.handle();
        runPendingActions();
        Battery::update(Settings::value("battCutoff"));
        serviceSetPointTrim();

        if (ws.count()) {
            const uint32_t telemetryMs = (uint32_t)Settings::value("telemetryMs");
            // Skip a frame rather than queue it if a client is slow.
            if (now - lastTelemetryMs >= telemetryMs && ws.availableForWriteAll()) {
                lastTelemetryMs = now;
                broadcastTelemetry();
            }
            const uint32_t sv = Settings::version();
            if (sv != seenSettings && now - lastSettingsMs >= SETTINGS_BROADCAST_MIN_MS) {
                seenSettings = sv;
                lastSettingsMs = now;
                sendSettings();
            }
            const uint32_t logSeq = DeviceLog::latest();
            if (logSeq != sentLogSeq && ws.availableForWriteAll()) {
                sendLog(sentLogSeq);
                sentLogSeq = logSeq;
            }
            // Drivers tab: on any change (and when balancing starts/stops,
            // which enables/disables its buttons), at most every 250 ms.
            const uint32_t dv = driverConfigVersion();
            const bool bal = balanceState == BalanceState::Balancing;
            const uint16_t heard = nodesHeard();
            if ((dv != seenDrivers || bal != sentBalancing || heard != sentHeard) && now - lastDriversMs >= WIFI_BROADCAST_MIN_MS) {
                seenDrivers = dv;
                sentBalancing = bal;
                sentHeard = heard;
                lastDriversMs = now;
                sendDrivers();
            }
            const uint32_t gv = Gamepad::version();
            if (gv != seenGamepad && now - lastGamepadMs >= WIFI_BROADCAST_MIN_MS) {
                seenGamepad = gv;
                lastGamepadMs = now;
                sendGamepad();
            }
            const uint32_t wv = WifiManager::version();
            if (wv != seenWifi && now - lastWifiMs >= WIFI_BROADCAST_MIN_MS) {
                seenWifi = wv;
                lastWifiMs = now;
                sendWifi();
            }
        } else {
            sentLogSeq = DeviceLog::latest();
        }
        if (now - lastCleanupMs >= CLEANUP_INTERVAL_MS) {
            lastCleanupMs = now;
            ws.cleanupClients(); // drop dead sockets (e.g. portal sheets closed abruptly)
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

} // namespace

const char *robotName() {
    WifiManager::makeNames();
    return WifiManager::apSsid().c_str();
}

void startWebInterface() {
    WifiManager::makeNames();
    xTaskCreatePinnedToCore(networkTask, "network", 8192, nullptr, 1, nullptr, 0);
}
