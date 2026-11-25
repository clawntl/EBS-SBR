#include "Display.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "Battery.h"
#include "Boot.h"
#include "CANConfig.h"
#include "CanBus.h"
#include "Config.h"
#include "DeviceLog.h"
#include "IMUDriver.h"
#include "LQRControl.h"
#include "MotorDrivers.h"
#include "PinConfig.h"
#include "RobotParams.h"
#include "WifiManager.h"

namespace {

// Addresses to probe, in order (most modules are 0x3C, some are strapped to 0x3D).
constexpr uint8_t  OLED_ADDRS[] = { 0x3C, 0x3D };
constexpr uint32_t REFRESH_MS = 200;   // 5 Hz
// How long a driver error report stays on the problem screen.
constexpr uint32_t ERROR_SHOW_MS = 10'000;
constexpr uint8_t  DEGREE = 0xF8;      // degree sign in the CP437 font

Adafruit_SSD1306 display(128, 64, &Wire, -1); // -1: no reset pin
bool displayOk = false;

const char *const kSide[2] = { "L", "R" };

const char *balanceName(BalanceState s) {
    switch (s) {
        case BalanceState::Balancing:      return "BALANCING";
        case BalanceState::Paused:         return "PAUSED";
        case BalanceState::Tipped:         return "TIPPED";
        case BalanceState::MotorsNotReady: return "MOTORS OFF";
        case BalanceState::ImuStalled:     return "IMU STALL";
        case BalanceState::BatteryLow:     return "BATT LOW";
        case BalanceState::Waiting:        return "HOLD UPRIGHT";
        case BalanceState::Runaway:        return "RUNAWAY";
        case BalanceState::Righting:       return "SELF-RIGHT";
        case BalanceState::OnSide:         return "ON SIDE";
    }
    return "?";
}

void drawAddress();

// Problem screen. Returns true if there was something to show.
bool drawProblems() {
    const MotorDriverInfo info[2] = { getMotorDriverInfo(MOTOR_LEFT), getMotorDriverInfo(MOTOR_RIGHT) };

    char lines[6][22];
    int n = 0;
    auto add = [&](const char *fmt, auto... args) {
        if (n < 6) snprintf(lines[n++], sizeof(lines[0]), fmt, args...);
    };

    if (!canBusOk) add("CAN init failed");
    if (balanceState == BalanceState::ImuStalled) add("IMU stalled - 0 A");
    if (Battery::isLow()) add("Battery low %.2fV", Battery::voltage());

    for (int s = 0; s < 2; s++) {
        const MotorDriverInfo &m = info[s];
        switch (m.link) {
            case MotorLink::Offline: add("%s driver offline", kSide[s]);                          break;
            case MotorLink::Fault:   add("%s FOC start failed", kSide[s]);                        break;
            case MotorLink::GaveUp:  add("%s gave up (%u tries)", kSide[s], m.startAttempts);     break;
            default: break;
        }
        if (m.flags & DRIVER_FLAG_DRV_FAULT) add("%s DRV8316 fault", kSide[s]);
        if (m.lastError && m.lastErrorAgeMs < ERROR_SHOW_MS) add("%s err: %s", kSide[s], driverErrorName(m.lastError));
        // Only a problem while balancing; at other times the host sends 0 A
        // anyway and the driver timing out is expected during startup.
        if ((m.flags & DRIVER_FLAG_COMMAND_TIMEOUT) && m.link == MotorLink::Running && balanceState == BalanceState::Balancing)
            add("%s cmd timeout", kSide[s]);
    }

    // A driver that's just starting isn't a problem on its own.
    if (n == 0) return false;

    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("! FAULT");
    display.setTextSize(1);
    // Four lines fit above the address line; with more, the last says so.
    for (int i = 0; i < n && i < 4; i++) {
        display.setCursor(0, 18 + i * 8);
        if (i == 3 && n > 4) display.printf("+%d more", n - 3);
        else display.print(lines[i]);
    }
    drawAddress();
    return true;
}

// Bottom line: where to find the web page. The address on the joined
// network if there is one, otherwise the robot's own access point.
void drawAddress() {
    const WifiManager::Status st = WifiManager::status();
    display.setTextSize(1);
    display.setCursor(0, 56);
    if (st.connected && st.ip.length())  display.printf("IP %s", st.ip.c_str());
    else if (st.apActive)                display.printf("AP %s", st.apIp.c_str());
    else if (st.connecting)              display.print("Wi-Fi connecting...");
    else                                 display.print("no network");
}

// Boot screen: step number and name, a detail line, a progress bar and the
// web page's address (it comes up early, while the rest is still starting).
void drawBoot() {
    const Boot::View v = Boot::view();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("SBR");
    display.setTextSize(1);
    display.setCursor(44, 0);
    display.print("v" FIRMWARE_VERSION);
    display.setCursor(44, 8);
    display.printf("booting %.1fs", millis() / 1000.0f);

    display.setCursor(0, 20);
    if (v.index) display.printf("%u/%u %s", v.index, v.total, v.name);
    else         display.print(v.name);
    display.setCursor(0, 30);
    display.print(v.detail);

    // Progress: the steps already done, plus half of the running one.
    const int w = 128;
    display.drawRect(0, 42, w, 7, SSD1306_WHITE);
    const int fill = v.total ? ((w - 4) * (2 * (v.index ? v.index - 1 : 0) + 1)) / (2 * v.total) : 0;
    display.fillRect(2, 44, fill, 3, SSD1306_WHITE);

    drawAddress();
}

void drawNormal() {
    // Line 1: balance state, large.
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(balanceName(balanceState));

    // Tilt (the angle the LQR uses) relative to upright, and the set point.
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.printf("%+5.1f", theta[2] * RAD_TO_DEG);
    display.write(DEGREE);
    display.setTextSize(1);
    display.setCursor(84, 20);
    display.print("set");
    display.setCursor(84, 28);
    display.printf("%+.1f", angleSetPoint * RAD_TO_DEG);
    display.write(DEGREE);

    // Commanded currents.
    display.setCursor(0, 38);
    display.printf("I  L%+5.2f  R%+5.2f A", leftTargetCurrent, rightTargetCurrent);

    // Driver state per side (+ T if the driver reports a command timeout).
    const MotorDriverInfo l = getMotorDriverInfo(MOTOR_LEFT);
    const MotorDriverInfo r = getMotorDriverInfo(MOTOR_RIGHT);
    display.setCursor(0, 47);
    display.printf("L:%s%s R:%s%s",
                   motorLinkName(l.link), (l.flags & DRIVER_FLAG_COMMAND_TIMEOUT) ? "T" : "",
                   motorLinkName(r.link), (r.flags & DRIVER_FLAG_COMMAND_TIMEOUT) ? "T" : "");

    drawAddress();
}

} // namespace

// True if a device ACKs its address. A zero-length write is an address-only
// probe, so nothing is sent to the device.
bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

bool initDisplay() {
    displayOk = false;
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    // Adafruit_SSD1306::begin() doesn't check the device is there (it only
    // fails if the frame buffer can't be allocated), so without this probe a
    // missing OLED makes every display() call fail and spam the log.
    uint8_t addr = 0;
    for (uint8_t a : OLED_ADDRS) {
        if (i2cProbe(a)) { addr = a; break; }
    }
    if (addr == 0) {
        DeviceLog::println("[oled] not found on I2C - continuing without display");
        Wire.end(); // nothing else is on this bus; release the pins and driver
        return false;
    }

    if (!display.begin(SSD1306_SWITCHCAPVCC, addr)) {
        DeviceLog::println("[oled] init failed (out of memory) - continuing without display");
        Wire.end();
        return false;
    }
    DeviceLog::printf("[oled] found at 0x%02X\n", addr);
    displayOk = true;
    display.cp437(true); // so DEGREE maps to the degree sign
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    drawBoot();
    display.display();
    return true;
}

void Task_display(void *pvParams) {
    if (!displayOk) vTaskDelete(nullptr);

    TickType_t lastWake = xTaskGetTickCount();
    while (1) {
        display.clearDisplay();
        if (Boot::active()) drawBoot();
        else if (!drawProblems()) drawNormal();
        display.display();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(REFRESH_MS));
    }
}
