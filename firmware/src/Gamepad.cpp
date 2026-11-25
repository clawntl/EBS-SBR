#include "Gamepad.h"

// The library keeps its state in header-level statics: include it in this
// file only.
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Preferences.h>
#include <atomic>

#include "Capture.h"
#include "DeviceLog.h"
#include "Settings.h"
#include "DriveInput.h"
#include "LQRControl.h"
#include "RobotParams.h"

namespace Gamepad {

bool enabled = true;
uint8_t layout = 0;
float deadzone = 0.12f;

namespace {

constexpr uint32_t PAIR_WINDOW_MS = 30000;
// Sticks that are off-centre send a stream of notifications (they're never
// perfectly still). If none has arrived for this long, treat the input as
// lost rather than keep driving on the last value until the BLE link times
// out.
constexpr uint32_t STALE_INPUT_MS = 500;
// While looking for the remembered controller, scan at this duty cycle
// (listen SCAN_WINDOW_MS of every SCAN_INTERVAL_MS) instead of the library's
// continuous scan: leaves most of the shared radio to Wi-Fi, and a controller
// that's switched on is still seen within a fraction of a second (it
// advertises many times a second while reconnecting). Pairing keeps the
// library's continuous scan.
constexpr uint16_t SCAN_INTERVAL_MS = 100;
constexpr uint16_t SCAN_WINDOW_MS = 30;
constexpr uint32_t SCAN_DURATION_MS = 4000;
// Once connected, ask for a longer connection interval than controllers
// usually use (7.5-15 ms), so BLE takes fewer radio slots from Wi-Fi.
// Units of 1.25 ms: 12-24 = 15-30 ms. Supervision timeout 2 s (units of
// 10 ms). The controller may refuse; the result is logged.
constexpr uint16_t CONN_ITVL_MIN = 12, CONN_ITVL_MAX = 24, CONN_TIMEOUT = 200;
constexpr uint32_t CONN_PARAMS_DELAY_MS = 1000;

// Placeholder target while pairing with nothing remembered: with no target at
// all the library connects to ANY Xbox controller it sees, paired or not.
const char *const NO_ADDRESS = "00:00:00:00:00:01";

XboxSeriesXControllerESP32_asukiaaa::Core xbox(NO_ADDRESS);

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
Status st = { State::Disabled, "", 0, false, -1 };
std::atomic<uint32_t> ver{0};
std::atomic<bool> pairRequested{false}, forgetRequested{false};

void setState(State s, uint8_t pairLeft = 0) {
    portENTER_CRITICAL(&lock);
    const bool changed = st.state != s || st.pairSecondsLeft != pairLeft;
    st.state = s;
    st.pairSecondsLeft = pairLeft;
    portEXIT_CRITICAL(&lock);
    if (changed) ver++;
}

void setAddress(const String &a) {
    portENTER_CRITICAL(&lock);
    st.address = a;
    portEXIT_CRITICAL(&lock);
    ver++;
}

String savedAddress() {
    Preferences p;
    String a;
    if (p.begin("gamepad", true)) {
        a = p.getString("addr", "");
        p.end();
    }
    return a;
}

void saveAddress(const String &a) {
    Preferences p;
    if (p.begin("gamepad", false)) {
        if (a.length()) p.putString("addr", a);
        else p.remove("addr");
        p.end();
    }
}

void target(const String &a) {
    xbox.setTargetAddress(*NimBLEAddress(std::string(a.c_str()), BLE_ADDR_PUBLIC).getBase());
}

float axis(uint16_t raw) { return (raw / 65535.0f) * 2.0f - 1.0f; }

float applyDeadzone(float v) {
    const float dz = constrain(deadzone, 0.0f, 0.5f);
    const float a = fabsf(v);
    if (a <= dz) return 0;
    return copysignf((a - dz) / (1.0f - dz), v);
}

// Rumble: `pulses` buzzes of `onMs`, `offMs` apart, at `power` % on both
// grip motors (the report counts time in 10 ms units).
void buzz(uint8_t pulses, uint16_t onMs, uint16_t offMs, uint8_t power) {
    XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase r;
    r.v.select.center = false;
    r.v.select.shake = false;
    r.v.select.left = true;
    r.v.select.right = true;
    r.v.power.left = power;
    r.v.power.right = power;
    r.v.power.center = 0;
    r.v.power.shake = 0;
    r.v.timeActive = (uint8_t)min<uint16_t>(onMs / 10, 255);
    r.v.timeSilent = (uint8_t)min<uint16_t>(offMs / 10, 255);
    r.v.countRepeat = pulses ? pulses - 1 : 0;
    xbox.writeHIDReport(r);
}

void setBattery(int16_t pct) {
    portENTER_CRITICAL(&lock);
    const bool changed = st.battery != pct;
    st.battery = pct;
    portEXIT_CRITICAL(&lock);
    if (changed) ver++;
}

void disconnectAll() {
    for (NimBLEClient *c : NimBLEDevice::getConnectedClients()) c->disconnect();
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) scan->stop();
}

void Task_gamepad(void *) {
    String addr = savedAddress();
    setAddress(addr);
    xbox.begin();
    if (addr.length()) target(addr);
    DeviceLog::printf("[gamepad] BLE started; %s\n",
                      addr.length() ? ("remembered controller " + addr).c_str() : "no controller paired yet");

    bool pairing = false;
    uint32_t pairUntil = 0;
    bool wasConnected = false, wasScanning = false, wasEnabled = true;
    uint32_t connectedAtMs = 0;
    bool connParamsAsked = false;
    uint32_t lastNotifyAt = 0, lastNotifySeenMs = 0;
    bool lastA = false, lastB = false, lastX = false, lastY = false;
    Capture::State lastCapture = Capture::state();
    BalanceState lastBalance = balanceState;

    for (;;) {
        const uint32_t now = millis();

        // Switched off (gpEnable): disconnect, stop scanning and leave the
        // radio to Wi-Fi. (The BLE stack stays initialised, just idle.)
        if (!enabled) {
            if (wasEnabled) {
                disconnectAll();
                pairing = false;
                xbox.setIgnoreTargetAddressToConnectToUnkownSearchingDevice(false);
                DriveInput::setGamepad(0, 0);
                DeviceLog::println("[gamepad] Bluetooth controller off");
                wasEnabled = false;
            }
            pairRequested = false;
            wasConnected = wasScanning = false;
            setState(State::Disabled);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (!wasEnabled) {
            DeviceLog::println("[gamepad] Bluetooth controller on");
            wasEnabled = true;
        }

        if (forgetRequested.exchange(false)) {
            for (NimBLEClient *c : NimBLEDevice::getConnectedClients()) c->disconnect();
            NimBLEDevice::deleteAllBonds();
            saveAddress("");
            addr = "";
            setAddress("");
            target(NO_ADDRESS);
            pairing = false;
            xbox.setIgnoreTargetAddressToConnectToUnkownSearchingDevice(false);
            DeviceLog::println("[gamepad] controller forgotten");
        }
        if (pairRequested.exchange(false)) {
            pairing = true;
            pairUntil = now + PAIR_WINDOW_MS;
            xbox.setIgnoreTargetAddressToConnectToUnkownSearchingDevice(true);
            DeviceLog::println("[gamepad] pairing: hold the controller's pair button until its logo flashes fast");
        }
        if (pairing && (int32_t)(now - pairUntil) >= 0) {
            pairing = false;
            xbox.setIgnoreTargetAddressToConnectToUnkownSearchingDevice(false);
            DeviceLog::println("[gamepad] pairing window closed");
        }

        // Nothing to look for: leave the radio to Wi-Fi.
        if (!addr.length() && !pairing && !xbox.isConnected()) {
            setState(State::NotPaired);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        xbox.onLoop(); // scans / connects (can block for a few seconds while connecting)
        const bool connected = xbox.isConnected();

        // The library (re)starts a continuous scan each time; while just
        // looking for the remembered controller, restart it at a lower duty.
        NimBLEScan *scan = NimBLEDevice::getScan();
        const bool scanning = scan->isScanning();
        if (scanning && !wasScanning && !pairing && !connected) {
            scan->setInterval(SCAN_INTERVAL_MS);
            scan->setWindow(SCAN_WINDOW_MS);
            scan->start(SCAN_DURATION_MS, false, true);
        }
        wasScanning = scanning;

        // Ask for a longer connection interval once settled after connecting.
        if (connected && wasConnected && !connParamsAsked && now - connectedAtMs >= CONN_PARAMS_DELAY_MS) {
            connParamsAsked = true;
            NimBLEClient *c = XboxSeriesXControllerESP32_asukiaaa::pConnectedClient;
            if (c && c->isConnected()) {
                const bool ok = c->updateConnParams(CONN_ITVL_MIN, CONN_ITVL_MAX, 0, CONN_TIMEOUT);
                vTaskDelay(pdMS_TO_TICKS(500));
                DeviceLog::printf("[gamepad] connection interval %.2f ms (asked for %.0f-%.0f ms%s)\n",
                                  c->getConnInfo().getConnInterval() * 1.25f,
                                  CONN_ITVL_MIN * 1.25f, CONN_ITVL_MAX * 1.25f, ok ? "" : ", request failed");
            }
        }

        if (connected && !wasConnected) {
            const String a = xbox.buildDeviceAddressStr();
            if (pairing || a != addr) {
                addr = a;
                saveAddress(a);
                setAddress(a);
                target(a);
                pairing = false;
                xbox.setIgnoreTargetAddressToConnectToUnkownSearchingDevice(false);
                DeviceLog::printf("[gamepad] paired with %s\n", a.c_str());
            }
            DeviceLog::printf("[gamepad] connected (%s)\n", a.c_str());
            connectedAtMs = now;
            connParamsAsked = false;
            buzz(2, 80, 80, 35); // connected
            lastBalance = balanceState;
        } else if (!connected && wasConnected) {
            DeviceLog::println("[gamepad] disconnected");
            DriveInput::setGamepad(0, 0);
            setBattery(-1);
        }
        wasConnected = connected;

        if (connected) {
            setState(State::Connected);
            const XboxControllerNotificationParser &n = xbox.xboxNotif;
            const uint32_t at = xbox.getReceiveNotificationAt();
            if (at != lastNotifyAt) { lastNotifyAt = at; lastNotifySeenMs = now; }

            float speed = -applyDeadzone(axis(n.joyLVert)); // stick up = low values
            float turn = applyDeadzone(axis(layout == 1 ? n.joyRHori : n.joyLHori));
            const bool off = speed != 0 || turn != 0;
            if (off && now - lastNotifySeenMs > STALE_INPUT_MS) speed = turn = 0; // input stream stopped
            DriveInput::setGamepad(turn, speed);
            portENTER_CRITICAL(&lock);
            const bool changed = st.inputActive != off;
            st.inputActive = off;
            portEXIT_CRITICAL(&lock);
            if (changed) ver++;

            // Buttons (on press): B = pause, A = resume, X = self-right / stop,
            // Y = record / stop.
            if (n.btnB && !lastB && !paused) { paused = true; DeviceLog::println("[gamepad] B: paused"); }
            if (n.btnA && !lastA && paused)  { paused = false; DeviceLog::println("[gamepad] A: resumed"); }
            if (n.btnY && !lastY) {
                const Capture::State cs = Capture::state();
                if (cs == Capture::State::Recording || cs == Capture::State::Armed) {
                    Capture::cancel(); // keeps what's been recorded
                    DeviceLog::println("[gamepad] Y: recording stopped");
                } else {
                    const int secs = constrain((int)lroundf(Settings::value("recSecs")), 1, (int)Capture::CAPTURE_MAX_SECONDS);
                    if (Capture::start(secs, Capture::Trigger::Now)) {
                        DeviceLog::printf("[gamepad] Y: recording %d s\n", secs);
                    } else {
                        DeviceLog::println("[gamepad] Y: couldn't start a recording (out of memory, or a download is running)");
                        buzz(3, 80, 80, 60);
                    }
                }
            }
            if (n.btnX && !lastX) {
                bool stopping = false;
                const char *err = requestSelfRight(&stopping);
                if (err) {
                    DeviceLog::printf("[gamepad] X: %s\n", err);
                    buzz(3, 80, 80, 60);
                } else {
                    DeviceLog::printf("[gamepad] X: self-righting %s\n", stopping ? "stop" : "start");
                }
            }
            lastA = n.btnA;
            lastB = n.btnB;
            lastX = n.btnX;
            lastY = n.btnY;

            // Recording started / finished (from Y or the web page): buzz.
            const Capture::State cs = Capture::state();
            if (cs != lastCapture) {
                if (cs == Capture::State::Recording) buzz(1, 150, 0, 40);
                else if (cs == Capture::State::Done && lastCapture == Capture::State::Recording) buzz(2, 200, 150, 70);
                lastCapture = cs;
            }

            // Balancing started / fell over (or was picked up): buzz. Not for
            // pausing, a stalled IMU or the drivers dropping out.
            const BalanceState bs = balanceState;
            if (bs != lastBalance) {
                if (bs == BalanceState::Balancing) buzz(1, 80, 0, 80);
                else if (bs == BalanceState::Righting) buzz(2, 60, 60, 50);
                else if (lastBalance == BalanceState::Balancing &&
                         (bs == BalanceState::Tipped || bs == BalanceState::Runaway || bs == BalanceState::OnSide)) buzz(1, 500, 0, 80);
                lastBalance = bs;
            }

            // Battery % (BLE battery service; arrives when it changes).
            if (xbox.getReceiveBatteryAt()) setBattery(xbox.battery);
        } else {
            lastA = lastB = lastX = lastY = false;
            lastCapture = Capture::state();
            lastBalance = balanceState;
            if (pairing) setState(State::Pairing, (uint8_t)((pairUntil - now + 999) / 1000));
            else setState(addr.length() ? State::Searching : State::NotPaired);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

} // namespace

void begin() {
    // Core 0, low priority: the library blocks while connecting. Runs even
    // when switched off (gpEnable), so it can be switched on without a
    // restart; it then just idles.
    xTaskCreatePinnedToCore(Task_gamepad, "gamepad", 8192, nullptr, 1, nullptr, 0);
}

void requestPairing() { pairRequested = true; }
void requestForget() { forgetRequested = true; }

Status status() {
    portENTER_CRITICAL(&lock);
    const Status s = st;
    portEXIT_CRITICAL(&lock);
    return s;
}

uint32_t version() { return ver; }

const char *stateName(State s) {
    switch (s) {
        case State::Disabled:  return "off";
        case State::NotPaired: return "not paired";
        case State::Searching: return "searching";
        case State::Pairing:   return "pairing";
        case State::Connected: return "connected";
    }
    return "?";
}

} // namespace Gamepad
