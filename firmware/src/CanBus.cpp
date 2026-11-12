#include "CanBus.h"

#include <Arduino.h>
#include <ACAN_ESP32FD.h>

#include "ByteUtils.h"
#include "CANConfig.h"
#include "LQRControl.h"
#include "MotorDrivers.h"
#include "PinConfig.h"
#include "RobotParams.h"

float leftShaftVelocity = 0;
float rightShaftVelocity = 0;

bool canBusOk = false;

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

    canBusOk = (errorCode == 0);
    return canBusOk;
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

// Handles one frame from a motor driver (telemetry IDs 0x100/0x101).
static void handleDriverFrame(MotorSide side, const CANMessage &rxMsg) {
    if (rxMsg.len < 1) return;

    switch (rxMsg.data[0]) {
        case CMD_SHAFT_VELOCITY: {
            if (rxMsg.len < 5) return;
            const float v = bytes2floatMSB(rxMsg.data + 1);
            if (side == MOTOR_LEFT) leftShaftVelocity = v;
            else                    rightShaftVelocity = v;
            break;
        }
        case CMD_DRIVER_STATUS: {
            if (rxMsg.len < 3) return;
            handleDriverStatusFrame(side, rxMsg.data[1], rxMsg.data[2]);
            break;
        }
        case CMD_START_FOC: {
            if (rxMsg.len < 5) return;
            handleDriverStartReply(side, bytes2floatMSB(rxMsg.data + 1) > 0.5f);
            break;
        }
        case CMD_ERROR: {
            if (rxMsg.len < 5) return;
            handleDriverError(side, (uint8_t)bytes2floatMSB(rxMsg.data + 1));
            break;
        }
        default:
            break;
    }
}

// Task checks the rx buffer and deals with the messages
void Task_readCAN(void *pvParams) {
    while (1) {
        CANMessage rxMsg;
        // Drain everything that's queued, then yield.
        while (can0.receive(rxMsg)) {
            // Only standard (11 bit) IDs are used by this protocol.
            if (rxMsg.ext) continue;

            switch (rxMsg.id) {
                case LEFT_MOTOR_VELOCITY_CAN_ID:  handleDriverFrame(MOTOR_LEFT, rxMsg);  break;
                case RIGHT_MOTOR_VELOCITY_CAN_ID: handleDriverFrame(MOTOR_RIGHT, rxMsg); break;
                default: break;
            }
        }
        // 1 tick: lets lower-priority tasks on this core run (vTaskDelay(0)
        // only yields to equal/higher priority). Frames queue in the driver's
        // receive buffer in the meantime.
        vTaskDelay(1);
    }
}

void Task_sendMotorCurrents(void *pvParams) {
    // Initialised ONCE: xTaskDelayUntil() advances it by exactly one period
    // per call, giving a fixed send rate. (Re-reading the tick count every
    // iteration made it a plain delay of one period after the send, so the
    // rate drifted below motorSendRate.)
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        sendMotorCurrents(leftTargetCurrent, rightTargetCurrent, BOTH_MOTOR_CAN_ADDR);

        // motorSendRate can be changed live from the web page; guard against 0
        // and keep the period at least one tick.
        const uint16_t rate = motorSendRate ? motorSendRate : 1;
        TickType_t period = pdMS_TO_TICKS(1000 / rate);
        if (period == 0) period = 1;

        // If we've fallen more than a period behind (e.g. the rate was just
        // lowered), resync rather than sending a burst to catch up.
        if (xTaskGetTickCount() - xLastWakeTime > period) xLastWakeTime = xTaskGetTickCount();

        xTaskDelayUntil(&xLastWakeTime, period);
    }
}
