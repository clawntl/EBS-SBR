#include <Arduino.h>

#include "Boot.h"
#include "CANConfig.h"
#include "Config.h"
#include "CanBus.h"
#include "DeviceLog.h"
#include "Gamepad.h"
#include "GyroCal.h"
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

    // OLED first, so the boot screen shows straight away. Task_display draws
    // it (and the normal screen after booting) from here on.
    initDisplay();
    xTaskCreatePinnedToCore(Task_display, "display", 4096, NULL, 1, NULL, 0);
    initStatusLED();

    // Give a USB serial terminal a moment to connect so the boot messages can
    // be seen; carries on as soon as one does. (They're all in the web page's
    // Log tab too, and the serial BOOT command repeats the timing.)
    const uint32_t serialWaitStart = millis();
    while (!Serial && millis() - serialWaitStart < BOOT_SERIAL_WAIT_MS) delay(10);
    DeviceLog::println("[boot] EBS-SBR controller " FIRMWARE_VERSION " starting (built " __DATE__ " " __TIME__ ")");

    // Saved tuning (or the defaults) into the RobotParams globals, before
    // anything uses them.
    Boot::step(BootStep::Settings);
    Settings::load();

    // Network task on core 0: WiFi, web page, OTA and the battery monitor.
    // These carry on starting in their own tasks; the web page is usually up
    // before the rest of the boot has finished.
    Boot::step(BootStep::Network);
    startWebInterface();
    // BLE Xbox controller (its own task on core 0; Wi-Fi shares the radio).
    Gamepad::begin();
    pinMode(BTN_PIN, INPUT);

    Boot::step(BootStep::Can);
    initCanBus();
    delay(10);
    // CAN receive (status/velocity frames) runs from the start so the
    // supervisor can see the drivers' heartbeats. Motor currents are sent by
    // Task_LQR once it's running; until then the drivers' own 100 ms command
    // timeout holds them at 0 A.
    xTaskCreatePinnedToCore(Task_readCAN, "readCAN", 4098, NULL, 1, NULL, 1);
    // Starts FOC on each driver as soon as it reports idle (and again if a
    // driver resets later), once allowMotorStarts() has been called below.
    xTaskCreatePinnedToCore(Task_motorSupervisor, "motorSupervisor", 3072, NULL, 1, NULL, 0);

    // Core 1 priorities: LQR (3) > IMU (2) > everything else (1), so a new
    // IMU sample is taken on time and the LQR runs and sends immediately
    // after it, instead of waiting for a time slice. Both block between
    // samples, so the priority-1 tasks still get the rest of the CPU.
    Boot::step(BootStep::Imu);
    initIMU();
    xTaskCreatePinnedToCore(Task_BMI160, "BMI160 Loop", 2048, NULL, 2, NULL, 1);

    // Gyro offsets, while the motor drivers are still booting: their FOC
    // start moves the wheels, which would upset the measurement, so the
    // motors only start once this is done.
    Boot::step(BootStep::Gyro);
    GyroCal::calibrateAtBoot();

    Boot::step(BootStep::Motors);
    allowMotorStarts();
    const uint32_t motorWaitStart = millis();
    while (!motorsReady() && millis() - motorWaitStart < BOOT_MOTOR_WAIT_MS) {
        const MotorDriverInfo l = getMotorDriverInfo(MOTOR_LEFT), r = getMotorDriverInfo(MOTOR_RIGHT);
        const bool aligning = (l.link == MotorLink::Starting && !(l.flags & DRIVER_FLAG_CALIBRATION_USED)) ||
                              (r.link == MotorLink::Starting && !(r.flags & DRIVER_FLAG_CALIBRATION_USED));
        if (aligning) Boot::detail("aligning - no stored");
        else          Boot::detail("L:%s R:%s", motorLinkName(l.link), motorLinkName(r.link));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (motorsReady()) {
        DeviceLog::println("[boot] both motor drivers running");
    } else {
        DeviceLog::println("[boot] WARNING: motor drivers not both running - continuing (supervisor keeps retrying)");
    }

    // The tilt estimate starts from the accelerometer (Task_BMI160), so the
    // controller can start straight away; it only balances once it's stood
    // upright and held still anyway.
    Boot::step(BootStep::Control);
    xTaskCreatePinnedToCore(Task_LQR, "LQR Task", 4096, NULL, 3, NULL, 1);

    Boot::finish();
}

void loop() {
    processSerialCommands();
    // loop() runs on core 1 alongside Task_readCAN at the same priority;
    // a short sleep leaves that core to the control tasks.
    vTaskDelay(pdMS_TO_TICKS(10));
}
