#include "CanBus.h"

#include <Arduino.h>
#include <ACAN_ESP32FD.h>

#include "ByteUtils.h"
#include "CANConfig.h"
#include "DeviceLog.h"
#include "LQRControl.h"
#include "MotorDrivers.h"
#include "PinConfig.h"
#include "RobotParams.h"
#include "Battery.h"
#include "DriveInput.h"

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
    settings.mFilters.addDualMaskFilter(CAN_RX_FILTER_ID, CAN_RX_FILTER_MASK,
                                        CAN_RX_FILTER2_ID, CAN_RX_FILTER2_MASK, false);

    const uint32_t errorCode = can0.begin(settings);
    DeviceLog::printf("[can] begin: %s (0x%08lX), actual bit rate %lu bit/s, sample point %.1f %%\n",
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

void sendDriverCommand(uint32_t canID, const uint8_t *data, uint8_t len) {
    CANMessage txMsg;
    txMsg.id = canID;
    txMsg.ext = false;
    txMsg.rtr = false;
    txMsg.len = len > 8 ? 8 : len;
    memcpy(txMsg.data, data, txMsg.len);
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

static void sendCurrentFrame(uint32_t id, float slot0, float slot1) {
    CANMessage txMsg;
    txMsg.id = id;
    txMsg.ext = false;
    txMsg.rtr = false;
    txMsg.len = 8;
    float2bytesMSB(slot0, txMsg.data);
    float2bytesMSB(slot1, txMsg.data + 4);
    can0.tryToSend(txMsg);
}

void sendMotorCurrents(float leftCur, float rightCur) {
    const uint8_t nl = driverNode(MOTOR_LEFT), nr = driverNode(MOTOR_RIGHT);
    const uint32_t fl = CURRENT_FRAME_BASE + (nl - 1) / 2, fr = CURRENT_FRAME_BASE + (nr - 1) / 2;
    // Slot 0 = odd node, slot 1 = even node.
    if (fl == fr) {
        if (nl % 2) sendCurrentFrame(fl, leftCur, rightCur);
        else        sendCurrentFrame(fl, rightCur, leftCur);
    } else {
        sendCurrentFrame(fl, (nl % 2) ? leftCur : NAN, (nl % 2) ? NAN : leftCur);
        sendCurrentFrame(fr, (nr % 2) ? rightCur : NAN, (nr % 2) ? NAN : rightCur);
    }
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
            // Bytes 3-4: supply voltage in mV, big-endian (0 / absent = none).
            const uint16_t mv = (rxMsg.len >= 5) ? (uint16_t)((rxMsg.data[3] << 8) | rxMsg.data[4]) : 0;
            handleDriverStatusFrame(side, rxMsg.data[1], rxMsg.data[2], mv);
            break;
        }
        case CMD_START_FOC: {
            if (rxMsg.len < 5) return;
            handleDriverStartReply(side, bytes2floatMSB(rxMsg.data + 1) > 0.5f);
            break;
        }
        case CMD_GET_INFO:
        case CMD_GET_CONFIG:
        case CMD_SET_PARAM:
        case CMD_CALIBRATION:
        case CMD_SAVE:
        case CMD_REVERT:
        case CMD_REBOOT:
        case CMD_GET_BOARD_INFO:
        case CMD_GET_PARAM:
        case CMD_NACK:
            handleDriverConfigReply(side, rxMsg.data, rxMsg.len);
            break;
        case CMD_ERROR: {
            if (rxMsg.len < 5) return;
            handleDriverError(side, (uint8_t)bytes2floatMSB(rxMsg.data + 1));
            break;
        }
        default:
            break;
    }
}

static int16_t sat16(float v) {
    if (!isfinite(v)) return 0;
    return (int16_t)constrain(lroundf(v), -32767L, 32767L);
}

static int16_t be16s(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

// Drive request from CAN module `id` (0x200 + id).
static void handleModuleFrame(uint8_t id, const CANMessage &m) {
    if (m.len < 5) return;
    const float a = be16s(m.data), b = be16s(m.data + 2);
    const uint8_t flags = m.data[4];
    float speed, turn;
    if (flags & MODULE_FLAG_FRACTION) {
        speed = a / 32767.0f;
        turn = b / 32767.0f;
    } else {
        speed = maxWheelSpeed > 0 ? (a / 100.0f) / maxWheelSpeed : 0;
        turn = maxTurnRate > 0 ? (b / 10.0f) * DEG_TO_RAD / maxTurnRate : 0;
    }
    DriveInput::setModule(id, turn, speed, flags & MODULE_FLAG_DRIVE);
}

static void put16(uint8_t *p, int v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

// Robot state for CAN modules (ROBOT_STATE_ID / ROBOT_INFO_ID).
static void sendRobotState(uint32_t nowMs) {
    static uint32_t lastStateMs = 0, lastInfoMs = 0;
    if (nowMs - lastStateMs >= 20) {
        lastStateMs = nowMs;
        const ControlSample c = getControlSnapshot();
        CANMessage f;
        f.id = ROBOT_STATE_ID; f.ext = false; f.rtr = false; f.len = 8;
        f.data[0] = (uint8_t)c.state;
        f.data[1] = c.source;
        put16(f.data + 2, sat16((c.tilt - c.setpoint) * 100));
        put16(f.data + 4, sat16(-c.phiDot * 100)); // drivers report forward as negative
        put16(f.data + 6, sat16(c.yawRate * 10));
        can0.tryToSend(f);
    }
    if (nowMs - lastInfoMs >= 500) {
        lastInfoMs = nowMs;
        CANMessage f;
        f.id = ROBOT_INFO_ID; f.ext = false; f.rtr = false; f.len = 8;
        const float v = Battery::voltage();
        put16(f.data, Battery::known() ? (int)lroundf(v * 1000) : 0);
        f.data[2] = Battery::known() ? (uint8_t)constrain(Battery::percent(), 0, 100) : 255;
        f.data[3] = (paused ? ROBOT_FLAG_PAUSED : 0) | (Battery::isLow() ? ROBOT_FLAG_BATTERY_LOW : 0)
                  | (DriveInput::modulesAllowed ? ROBOT_FLAG_MODULES_ALLOWED : 0);
        put16(f.data + 4, (int)constrain(lroundf(maxWheelSpeed * 100), 0L, 65535L));
        put16(f.data + 6, (int)constrain(lroundf(maxTurnRate * RAD_TO_DEG * 10), 0L, 65535L));
        can0.tryToSend(f);
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

            if (rxMsg.id >= MODULE_DRIVE_ID_BASE && rxMsg.id < MODULE_DRIVE_ID_BASE + DriveInput::MAX_MODULES) {
                handleModuleFrame(rxMsg.id - MODULE_DRIVE_ID_BASE, rxMsg);
                continue;
            }
            if (rxMsg.id < TELEMETRY_ID_BASE || rxMsg.id > TELEMETRY_ID_BASE + NODE_ID_MAX) continue;
            const uint8_t node = rxMsg.id - TELEMETRY_ID_BASE; // 0 = old firmware "right" board
            if (rxMsg.len >= 2 && rxMsg.data[0] == CMD_DRIVER_STATUS) noteNodeHeard(node, rxMsg.data[1]);
            if (node == driverNode(MOTOR_LEFT))       handleDriverFrame(MOTOR_LEFT, rxMsg);
            else if (node == driverNode(MOTOR_RIGHT)) handleDriverFrame(MOTOR_RIGHT, rxMsg);
        }
        if (canBusOk) sendRobotState(millis());
        // 1 tick: lets lower-priority tasks on this core run (vTaskDelay(0)
        // only yields to equal/higher priority). Frames queue in the driver's
        // receive buffer in the meantime.
        vTaskDelay(1);
    }
}
