#include "can_bus.h"

#include <Arduino.h>

#include <ACANFD_STM32.h>          // -> fdcan1 object, pins PB8(RX)/PB9(TX)
#include <ACANFD_STM32_Settings.h> // -> ACANFD_STM32_Settings, DataBitRateFactor

#include "can_protocol.h"
#include "motor_control.h"

static void float2bytesMSB(float val, uint8_t *bytes) {
    // Copy the raw bits of val into an integer so can perform
    // bitwise operations
    uint32_t valAsInt;
    memcpy(&valAsInt, &val, sizeof(float));

    // Extract the bytes in big-endian (MSB-first) order
    bytes[0] = (valAsInt >> 24) & 0xFF;
    bytes[1] = (valAsInt >> 16) & 0xFF;
    bytes[2] = (valAsInt >> 8)  & 0xFF;
    bytes[3] = (valAsInt >> 0)  & 0xFF;
}

static float bytes2floatMSB(const uint8_t *bytes) {
    // Read the bytes in big-endian (MSB-first) order
    uint32_t valAsInt =
        ((uint32_t)bytes[0] << 24) |
        ((uint32_t)bytes[1] << 16) |
        ((uint32_t)bytes[2] << 8)  |
        ((uint32_t)bytes[3]);

    // Copy the raw bits to a float and return
    float val;
    memcpy(&val, &valAsInt, sizeof(float));

    return val;
}

// Accept only frames addressed to both motors (broadcast) or to this motor.
// Equivalent to the old SimpleCAN mask filter (BOTH_MOTOR_CAN_ADDR / CAN_MASK),
// but expressed directly as the two identifiers it actually let through.
void configureAndStartCAN() {
    // DataBitRateFactor::x1 -> data phase == arbitration phase (we never set BRS,
    // so only the arbitration-phase sample point below actually matters).
    ACANFD_STM32_Settings settings(CAN_BIT_RATE, CAN_SAMPLE_POINT, DataBitRateFactor::x1, CAN_SAMPLE_POINT);
    settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;
    settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::REJECT;

    if (!settings.mBitSettingOk) {
        Serial.println("CAN bit rate/sample point could not be matched within tolerance!");
    }

    ACANFD_STM32_StandardFilters stdFilters;
    stdFilters.addDual(BOTH_MOTOR_CAN_ADDR, THIS_MOTOR_CAN_ADDR, ACANFD_STM32_FilterAction::FIFO0);

    const uint32_t errorCode = fdcan1.beginFD(settings, stdFilters);

    Serial.print("CAN bit rate: ");
    Serial.print(settings.actualArbitrationBitRate());
    Serial.print(" bit/s, sample point: ");
    Serial.print(settings.arbitrationSamplePointFromBitStart());
    Serial.println(" %");

    if (errorCode != 0) {
        Serial.print("fdcan1 configuration error: 0x");
        Serial.println(errorCode, HEX);
    }
}

void sendFloat(uint8_t cmd, float dat, uint32_t canID) {
    uint8_t floatData[4]; // Stores a single float (4 bytes)

    // Write the float into the byte array
    float2bytesMSB(dat, floatData);

    // Build a classic (non-FD) CAN data frame: cmd byte + 4 float bytes
    CANFDMessage txMsg;
    txMsg.id   = canID;
    txMsg.ext  = false;
    txMsg.type = CANFDMessage::CAN_DATA;
    txMsg.len  = 5;
    txMsg.data[0] = cmd;
    memcpy(&txMsg.data[1], floatData, sizeof(floatData));

    const uint32_t sendStatus = fdcan1.tryToSendReturnStatusFD(txMsg);
    if (sendStatus != 0) {
        Serial.print("CAN send error: 0x");
        Serial.println(sendStatus, HEX);
    }
}

static void handleCommandMessage(const CANFDMessage &rxMsg) {
    switch (rxMsg.data[0]) {
        case CMD_SHAFT_VELOCITY: {
            // Do nothing if received, as this is sent, not consumed.
            break;
        }
        case CMD_START_FOC: {
            if (!enabled) {
                Serial.println("Received CMD_START_FOC");
                startFOC();
            } else {
                Serial.println("FOC already started");
            }
            break;
        }
        case CMD_ERROR: {
            break;
        }
        default: {
            break;
        }
    }
}

void pollCAN() {
    CANFDMessage rxMsg;
    if (!fdcan1.receiveFD0(rxMsg)) return;

    // Only standard (non-extended) ID messages are used by this protocol.
    if (rxMsg.ext) return;

    switch (rxMsg.id) {
        case BOTH_MOTOR_CAN_ADDR: {
            uint8_t received_float_data[4];

            // If this is the left motor read the first 4 bytes
            #ifdef IS_LEFT_MOTOR
                memcpy(received_float_data, rxMsg.data, 4);
            // If this is the right motor read the last 4 bytes
            #else
                memcpy(received_float_data, rxMsg.data + 4, 4);
            #endif

            // Convert the raw bytes into a float assuming big-endian order,
            // constrained to +- 2 Amps
            target_current = constrain(bytes2floatMSB(received_float_data), -2, 2);
            break;
        }

        default: {
            handleCommandMessage(rxMsg);
            break;
        }
    }
}
