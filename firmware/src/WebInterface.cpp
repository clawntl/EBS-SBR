#include "WebInterface.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>

#include "LQRControl.h"
#include "RobotParams.h"

// Captive portal:
//   1. Soft AP hands itself out as gateway + DNS server (DHCP).
//   2. A wildcard DNS server answers EVERY hostname with our IP.
//   3. The phone's OS fires a connectivity probe (e.g.
//      connectivitycheck.gstatic.com/generate_204). DNS sends it to us and we
//      reply 302 -> http://192.168.4.1/, so the OS decides the network is
//      captive and opens its sign-in sheet straight onto the tuning page.

namespace {

// The robot hosts its own access point so a phone/laptop can connect to it
// directly without needing an existing WiFi network nearby.
const char *AP_SSID = "BalanceBot";
const char *AP_PASSWORD = "balancebot"; // nullptr/"" = open; otherwise >= 8 chars
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_MASK(255, 255, 255, 0);
constexpr uint16_t DNS_PORT = 53;

// The page treats >3 s of silence as a dead socket (portal sheets can freeze
// a socket without firing 'close'), so the server must say something at
// least this often even when nothing has changed.
constexpr uint32_t HEARTBEAT_MS = 1000;

DNSServer dnsServer;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

String apIpStr;   // "192.168.4.1"
String portalUrl; // "http://192.168.4.1/"
uint32_t lastHeartbeatMs = 0;

void buildStateJson(JsonDocument &doc) {
    JsonArray k = doc["K"].to<JsonArray>();
    for (float i : K) {
        k.add(i);
    }
    doc["angleSetPoint"] = angleSetPoint * RAD_TO_DEG;
    doc["deactivationAngle"] = deactivationAngle;
    doc["alpha"] = alpha;
    doc["IMUsampleRate"] = IMUsampleRate;
    doc["paused"] = paused;
    doc["turnCurrentGain"] = turnCurrentGain;
    doc["moveCurrentGain"] = moveCurrentGain;
}

void broadcastState() {
    JsonDocument doc;
    buildStateJson(doc);
    String out;
    serializeJson(doc, out);
    ws.textAll(out);
}

void handleSet(const char *key, float value) {
    if (!key) return;
    if (!strcmp(key, "K0")) K[0] = value;
    else if (!strcmp(key, "K1")) K[1] = value;
    else if (!strcmp(key, "K2")) K[2] = value;
    else if (!strcmp(key, "K3")) K[3] = value;
    else if (!strcmp(key, "K4")) K[4] = value;
    else if (!strcmp(key, "K5")) K[5] = value;
    else if (!strcmp(key, "angleSetPoint")) angleSetPoint = value * DEG_TO_RAD;
    else if (!strcmp(key, "deactivationAngle")) deactivationAngle = value;
    else if (!strcmp(key, "alpha")) alpha = value;
    else if (!strcmp(key, "IMUsampleRate")) IMUsampleRate = (uint16_t)value;
    else if (!strcmp(key, "paused")) paused = value != 0;
    else if (!strcmp(key, "turnCurrentGain")) turnCurrentGain = value;
    else if (!strcmp(key, "moveCurrentGain")) moveCurrentGain = value;
}

void handleJoystick(float x, float y) {
    joystickAxisX = constrain(x, -1.0f, 1.0f);
    joystickAxisY = constrain(y, -1.0f, 1.0f);
    joystickLastUpdateMs = millis();
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[ws] client #%u connected from %s\n", client->id(),
                      client->remoteIP().toString().c_str());
        JsonDocument doc;
        buildStateJson(doc);
        String out;
        serializeJson(doc, out);
        client->text(out);
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[ws] client #%u disconnected\n", client->id());
        // Immediately stop driving if the controlling page drops - don't
        // wait for Task_LQR's joystick staleness timeout.
        handleJoystick(0, 0);
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) return;

            const char *cmd = doc["cmd"].as<const char *>();
            if (cmd && !strcmp(cmd, "set")) {
                handleSet(doc["key"].as<const char *>(), doc["value"].as<float>());
                // Echo the authoritative state back to every client so all
                // connected pages (and the one that sent the change) agree.
                broadcastState();
            } else if (cmd && !strcmp(cmd, "joystick")) {
                handleJoystick(doc["x"].as<float>(), doc["y"].as<float>());
            }
        }
    }
}

// Always redirect to the IP rather than "/", so the page's origin is
// http://192.168.4.1 whichever hostname the OS probed, and the page's
// ws://${location.host}/ws resolves consistently.
void redirectToPortal(AsyncWebServerRequest *req) {
    req->redirect(portalUrl);
}

void setupRoutes() {
    // 1. WebSocket endpoint (register before the catch-alls).
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // 2. Known OS connectivity-check URLs. Any unexpected reply marks the
    //    network as captive; a 302 makes the sign-in sheet open on our page.
    static const char *probes[] = {
        "/generate_204", "/gen_204",                          // Android / ChromeOS
        "/hotspot-detect.html", "/library/test/success.html", // Apple
        "/connecttest.txt", "/ncsi.txt", "/redirect",         // Windows
        "/canonical.html", "/success.txt",                    // Firefox
        "/chat",                                              // older Android
    };
    for (const char *p : probes) server.on(p, HTTP_ANY, redirectToPortal);

    // 3. The tuning page. Requests under any other hostname bounce to the IP.
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (req->host() != apIpStr) return redirectToPortal(req);
        AsyncWebServerResponse *res = req->beginResponse(LittleFS, "/index.html", "text/html");
        res->addHeader("Cache-Control", "no-store"); // portal browsers cache aggressively
        req->send(res);
    });

    // Don't answer the favicon request with a redirect to an HTML page.
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *req) { req->send(204); });

    // 4. Everything else -> portal (also catches probe URLs not listed above).
    server.onNotFound(redirectToPortal);
}

} // namespace

void initWebInterface() {
    if (!LittleFS.begin(true)) {
        Serial.println("[fs] LittleFS mount failed");
    } else if (!LittleFS.exists("/index.html")) {
        Serial.println("[fs] /index.html missing - run Upload Filesystem Image");
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, (AP_PASSWORD && *AP_PASSWORD) ? AP_PASSWORD : nullptr);
    delay(100); // let the AP interface come up before reconfiguring it
    // Gateway = our IP, so DHCP clients use us as router *and* DNS server.
    WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);
    WiFi.setSleep(false); // lower WebSocket latency

    apIpStr = WiFi.softAPIP().toString();
    portalUrl = "http://" + apIpStr + "/";
    Serial.printf("[wifi] AP \"%s\" up at %s\n", AP_SSID, apIpStr.c_str());

    // Wildcard DNS: every name resolves to us. NoError (not NXDOMAIN) keeps
    // clients from treating lookups as failures.
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

    setupRoutes();
    server.begin();
    Serial.println("[http] server started");
}

void loopWebInterface() {
    dnsServer.processNextRequest(); // no-op on Arduino core 3.x (async DNS), needed on 2.x

    // Throttled: loop() spins with vTaskDelay(0), and cleanupClients() takes
    // the WebSocket lock every call, contending with the async_tcp task.
    uint32_t now = millis();
    if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
        lastHeartbeatMs = now;
        if (ws.count()) ws.textAll("{\"hb\":1}");
        ws.cleanupClients(); // drop dead sockets (portal sheets closed abruptly)
    }
}
