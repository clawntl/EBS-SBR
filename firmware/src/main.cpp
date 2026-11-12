#include <Arduino.h>

#include "CANConfig.h"
#include "CanBus.h"
#include "Display.h"
#include "IMUDriver.h"
#include "LQRControl.h"
#include "MotorDrivers.h"
#include "PinConfig.h"
#include "SerialCommands.h"
#include "StatusLED.h"
#include "WebInterface.h"


void setup() {

    Serial.begin(115200);
    delay(5000);
    Serial.println("Hi world");

    initDisplay();

    initStatusLED();

    Serial.println("Starting Web Interface");
    displayBootMessage("Starting WiFi...");
    initWebInterface();

    pinMode(BTN_PIN, INPUT);

    delay(500);
    Serial.println("Starting CAN");
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
    Serial.println("Waiting for motor drivers...");
    if (waitForMotorsReady(15'000)) {
        Serial.println("Both motor drivers running");
    } else {
        Serial.println("WARNING: motor drivers not both running - continuing (supervisor keeps retrying)");
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
    Serial.println("Starting LQR Loop");
    xTaskCreatePinnedToCore(Task_LQR, "LQR Task", 4096, NULL, 3, NULL, 1);
}

void loop() {
    processSerialCommands();
    loopWebInterface();
    vTaskDelay(0);
}
