#include <Arduino.h>

#include "CANConfig.h"
#include "CanBus.h"
#include "IMUDriver.h"
#include "LQRControl.h"
#include "PinConfig.h"
#include "SerialCommands.h"
#include "StatusLED.h"
#include "WebInterface.h"

void setup() {

    Serial.begin(115200);
    delay(5000);
    Serial.println("Hi world");

    initStatusLED();

    Serial.println("Starting Web Interface");
    initWebInterface();

    pinMode(BTN_PIN, INPUT);

    delay(500);
    Serial.println("Starting CAN");
    initCanBus();
    delay(10);

    sendStartFOC(LEFT_MOTOR_CAN_ADDR);
    delay(2000);
    sendStartFOC(RIGHT_MOTOR_CAN_ADDR);
    delay(10'000);

    initIMU();

    Serial.println("Starting BMI160 Loop");
    xTaskCreatePinnedToCore(Task_BMI160, "BMI160 Loop", 2048, NULL, 1, NULL, 1);

    // Serial.println("Starting output Loop");
    // xTaskCreatePinnedToCore(Task_outputGyroData, "Task_outputGyroData", 2048, NULL, 1, NULL, 1);

    Serial.println("Starting Read CAN Loop");
    xTaskCreatePinnedToCore(Task_readCAN, "readCAN", 4098, NULL, 1, NULL, 1);

    Serial.println("Starting Send Motor Loop");
    xTaskCreatePinnedToCore(Task_sendMotorCurrents, "sendMotorCurrents", 4098, NULL, 1, NULL, 1);

    delay(3000);
    Serial.println("Starting LQR Loop");
    xTaskCreatePinnedToCore(Task_LQR, "LQR Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
    processSerialCommands();
    loopWebInterface();
    vTaskDelay(0);
}
