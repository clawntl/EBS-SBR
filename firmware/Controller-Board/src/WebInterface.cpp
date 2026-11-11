#include "WebInterface.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>

#include "LQRControl.h"
#include "RobotParams.h"

namespace {

// The robot hosts its own access point so a phone/laptop can connect to it
// directly without needing an existing WiFi network nearby.
const char *AP_SSID = "BalanceBot";
const char *AP_PASSWORD = "balancebot";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void buildStateJson(JsonDocument &doc) {
    JsonArray k = doc["K"].to<JsonArray>();
    for (float i : K) {
        k.add(i);
    }
    doc["angleSetPoint"] = angleSetPoint * RAD_TO_DEG;
    doc["deactivationAngle"] = deactivationAngle;
    doc["alpha"] = alpha;
    doc["IMUsampleRate"] = IMUsampleRate;
    doc["motorSendRate"] = motorSendRate;
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
    else if (!strcmp(key, "motorSendRate")) motorSendRate = (uint16_t)value;
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
        JsonDocument doc;
        buildStateJson(doc);
        String out;
        serializeJson(doc, out);
        client->text(out);
    } else if (type == WS_EVT_DISCONNECT) {
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

} // namespace

void initWebInterface() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed");
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    server.begin();
}

void loopWebInterface() {
    ws.cleanupClients();
}
