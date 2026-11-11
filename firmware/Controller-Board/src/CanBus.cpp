#include "CanBus.h"

#include <Arduino.h>
#include <ACAN_ESP32FD.h>

#include "ByteUtils.h"
#include "CANConfig.h"
#include "LQRControl.h"
#include "PinConfig.h"
#include "RobotParams.h"

float leftShaftVelocity = 0;
float rightShaftVelocity = 0;

namespace {
ACAN_ESP32FD can0;
} // namespace

bool initCanBus() {
    ACAN_ESP32FD_Settings settings(CAN_BIT_RATE, CAN_SAMPLE_POINT_PERMILL);
    settings.mTxPin = CAN_TX_PIN;
    settings.mRxPin = CAN_RX_PIN;
    settings.mModuleMode = ACAN_ESP32FD_Settings::NORMAL;

    // See CANConfig.h for why these are set explicitly instead of trusting
    // the auto solver's choice of time quanta.
    settings.mArbitrationBitRatePrescaler = CAN_ARBITRATION_BRP;
    settings.mArbitrationPhaseSegment1 = CAN_ARBITRATION_PHASE_SEGMENT_1;
    settings.mArbitrationPhaseSegment2 = CAN_ARBITRATION_PHASE_SEGMENT_2;
    settings.mArbitrationSJW = CAN_ARBITRATION_SJW;

    // settings.mModuleMode = ACAN_ESP32FD_Settings::LOOP_BACK_NO_ACK;
    settings.mFilters.addMaskFilter(CAN_RX_FILTER_ID, CAN_RX_FILTER_MASK, false);

    const uint32_t errorCode = can0.begin(settings);
    Serial.printf("can0.begin: %s (0x%08lX), actual bit rate %lu bit/s, sample point %.1f %%\n",
                  (errorCode == 0) ? "OK" : "FAILED", (unsigned long) errorCode,
                  (unsigned long) settings.actualArbitrationBitRate(),
                  settings.arbitrationSamplePointFromBitStart());

    return errorCode == 0;
}

// Send the message to start FOC on the motor
void sendStartFOC(uint32_t canID) {
    CANMessage txMsg;
    txMsg.id = canID;
    txMsg.ext = false;
    txMsg.rtr = false;
    txMsg.len = 1;
    txMsg.data[0] = CMD_START_FOC;

    // Send the message
    can0.tryToSend(txMsg);
}

// Send a message with float datatype
void sendFloat(uint8_t cmd, float dat, uint32_t canID) {
    uint8_t floatData[4]; // Stores a single float (4 bytes)

    // Write the float into the byte array
    float2bytesMSB(dat, floatData);

    CANMessage txMsg;
    txMsg.id = canID;
    txMsg.ext = false;
    txMsg.rtr = false;
    txMsg.len = 5;

    // Set the first byte of the message to the CAN_CMD type
    txMsg.data[0] = cmd;
    // Copy the float into the message payload
    memcpy(&txMsg.data[1], floatData, sizeof(floatData));

    // Send the message
    can0.tryToSend(txMsg);
}

// Send a message with float datatype
void sendMotorCurrents(float leftCur, float rightCur, uint32_t canID) {
    uint8_t leftFloatData[4]; // Stores a single float (4 bytes)
    uint8_t rightFloatData[4]; // Stores a single float (4 bytes)

    // Write the float into the byte array
    float2bytesMSB(leftCur, leftFloatData);
    float2bytesMSB(rightCur, rightFloatData);

    CANMessage txMsg;
    txMsg.id = canID;
    txMsg.ext = false;
    txMsg.rtr = false;
    txMsg.len = 8;

    // Copy the left motor data into the message payload
    memcpy(txMsg.data, leftFloatData, 4);
    // Copy the right motor data into the message payload
    memcpy(txMsg.data + 4, rightFloatData, 4);

    // Send the message
    can0.tryToSend(txMsg);
}

// Task checks the rx buffer and deals with the messages
void Task_readCAN(void *pvParams) {
    while (1) {

        // If there are message in the rx buffer
        if (can0.available()) {

            // Read the message
            CANMessage rxMsg;
            can0.receive(rxMsg);

            // If it is using standard ID format (11 bit)
            if (!rxMsg.ext) {
                // Check the first payload byte (command byte)
                switch (rxMsg.data[0]) {
                    // If it is a velocity
                    case CMD_SHAFT_VELOCITY: {
                            // Find if it is left or right side using the message ID
                            switch (rxMsg.id) {
                                case RIGHT_MOTOR_VELOCITY_CAN_ID : {
                                    rightShaftVelocity = bytes2floatMSB(rxMsg.data + 1);
                                    break;
                                }
                                case LEFT_MOTOR_VELOCITY_CAN_ID : {
                                    leftShaftVelocity = bytes2floatMSB(rxMsg.data + 1);
                                    break;
                                }
                            }
                            break;
                    }
                }

            }
        }
        vTaskDelay(0);
    }
}

void Task_sendMotorCurrents(void *pvParams) {
    while (1) {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        sendMotorCurrents(leftTargetCurrent, rightTargetCurrent, BOTH_MOTOR_CAN_ADDR);

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS((uint16_t)(1000 / motorSendRate)));
    }
}
