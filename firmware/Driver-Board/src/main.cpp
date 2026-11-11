#include <Arduino.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

#include "pins.h"
#include "can_bus.h"
#include "motor_control.h"
#include "dfu.h"

void setup() {
    Serial.begin(115200);

    delay(5000);
    Serial.println("begin");

    pinMode(DRV_CS, OUTPUT);
    pinMode(ENC_CS, OUTPUT);

    pinMode(SO_A, INPUT);
    pinMode(SO_B, INPUT);
    pinMode(SO_C, INPUT);

    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(CAN_EN, OUTPUT);
    digitalWrite(CAN_EN, HIGH); // enable CAN transceiver for now

    pinMode(DFU_BUTTON, INPUT); // internal pull-up: idle high, pressed = low

    SimpleFOCDebug::enable(&Serial);

    SimpleFOC_CORDIC_Config();

    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    Serial.println("Initializing...");

    configureAndStartCAN();

    setupFOC();

    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    checkDfuButton();
    checkSerialForDFU();

    motorControlLoop();

    pollCAN();
}
