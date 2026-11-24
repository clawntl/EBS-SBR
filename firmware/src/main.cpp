#include <Arduino.h>

#include "CANConfig.h"
#include "Config.h"
#include "CanBus.h"
#include "DeviceLog.h"
#include "Gamepad.h"
#include "Display.h"
#include "IMUDriver.h"
#include "LQRControl.h"
#include "MotorDrivers.h"
#include "PinConfig.h"
#include "Settings.h"
#include "SerialCommands.h"
#include "StatusLED.h"
#include "WebInterface.h"


void setup() {

    Serial.begin(115200);
    delay(5000);
    DeviceLog::println("[boot] EBS-SBR controller " FIRMWARE_VERSION " starting (built " __DATE__ " " __TIME__ ")");

    // Saved tuning (or the defaults) into the RobotParams globals, before
    // anything uses them.
    Settings::load();

    initDisplay();

    initStatusLED();

    displayBootMessage("Starting WiFi...");
    // Network task on core 0: WiFi, web page, OTA and the battery monitor.
    startWebInterface();
    // BLE Xbox controller (its own task on core 0; Wi-Fi shares the radio).
    Gamepad::begin();

    pinMode(BTN_PIN, INPUT);

    delay(500);
    DeviceLog::println("[boot] starting CAN");
    displayBootMessage("Starting CAN...");
    initCanBus();
    delay(10);

    // CAN receive (status/velocity frames) runs from the start so the
    // supervisor can see the drivers' heartbeats. Motor currents are sent by
    // Task_LQR once it's running; until then the drivers' own 100 ms command
    // timeout holds them at 0 A.
    Serial.println("Starting Read CAN Loop");
    xTaskCreatePinnedToCore(Task_readCAN, "readCAN", 4098, NULL, 1, NULL, 1);

    // Starts FOC on each driver as soon as it reports idle (and again if a
    // driver resets later). Replaces the fixed START_FOC + delay sequence.
    xTaskCreatePinnedToCore(Task_motorSupervisor, "motorSupervisor", 3072, NULL, 1, NULL, 0);

    // Live status on the OLED from here on.
    xTaskCreatePinnedToCore(Task_display, "display", 4096, NULL, 1, NULL, 0);

    // Wait for both motors before zeroing the IMU: the FOC alignment moves
    // the wheels, which would upset the gyro offset measurement.
    DeviceLog::println("[boot] waiting for the motor drivers...");
    if (waitForMotorsReady(15'000)) {
        DeviceLog::println("[boot] both motor drivers running");
    } else {
        DeviceLog::println("[boot] WARNING: motor drivers not both running - continuing (supervisor keeps retrying)");
    }
    delay(500); // let the robot settle after alignment

    initIMU();

    Serial.println("Starting BMI160 Loop");
    // Core 1 priorities: LQR (3) > IMU (2) > everything else (1), so a new
    // IMU sample is taken on time and the LQR runs and sends immediately
    // after it, instead of waiting for a time slice. Both block between
    // samples, so the priority-1 tasks still get the rest of the CPU.
    xTaskCreatePinnedToCore(Task_BMI160, "BMI160 Loop", 2048, NULL, 2, NULL, 1);

    // Serial.println("Starting output Loop");
    // xTaskCreatePinnedToCore(Task_outputGyroData, "Task_outputGyroData", 2048, NULL, 1, NULL, 1);

    delay(3000);
    DeviceLog::println("[boot] starting the balance controller");
    xTaskCreatePinnedToCore(Task_LQR, "LQR Task", 4096, NULL, 3, NULL, 1);
}

void loop() {
    processSerialCommands();
    // loop() runs on core 1 alongside Task_readCAN at the same priority;
    // a short sleep leaves that core to the control tasks.
    vTaskDelay(pdMS_TO_TICKS(10));
}
