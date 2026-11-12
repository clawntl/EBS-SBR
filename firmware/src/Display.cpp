#include "Display.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "CANConfig.h"
#include "CanBus.h"
#include "IMUDriver.h"
#include "LQRControl.h"
#include "MotorDrivers.h"
#include "PinConfig.h"
#include "RobotParams.h"

namespace {

constexpr uint8_t  OLED_ADDR = 0x3C;   // some modules are 0x3D
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
    }
    return "?";
}

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
    for (int i = 0; i < n; i++) {
        display.setCursor(0, 18 + i * 8);
        display.print(lines[i]);
    }
    return true;
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
    display.setCursor(0, 40);
    display.printf("I  L%+5.2f  R%+5.2f A", leftTargetCurrent, rightTargetCurrent);

    // Driver state per side (+ T if the driver reports a command timeout).
    const MotorDriverInfo l = getMotorDriverInfo(MOTOR_LEFT);
    const MotorDriverInfo r = getMotorDriverInfo(MOTOR_RIGHT);
    display.setCursor(0, 52);
    display.printf("L:%s%s R:%s%s",
                   motorLinkName(l.link), (l.flags & DRIVER_FLAG_COMMAND_TIMEOUT) ? "T" : "",
                   motorLinkName(r.link), (r.flags & DRIVER_FLAG_COMMAND_TIMEOUT) ? "T" : "");
}

} // namespace

bool initDisplay() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED init failed (check wiring / I2C address) - continuing without display");
        displayOk = false;
        return false;
    }
    displayOk = true;
    display.cp437(true); // so DEGREE maps to the degree sign
    display.setTextColor(SSD1306_WHITE);
    displayBootMessage("Starting...");
    return true;
}

void displayBootMessage(const char *line) {
    if (!displayOk) return;
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("SBR");
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print(line);
    display.display();
}

void Task_display(void *pvParams) {
    if (!displayOk) vTaskDelete(nullptr);

    TickType_t lastWake = xTaskGetTickCount();
    while (1) {
        display.clearDisplay();
        if (!drawProblems()) drawNormal();
        display.display();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(REFRESH_MS));
    }
}
