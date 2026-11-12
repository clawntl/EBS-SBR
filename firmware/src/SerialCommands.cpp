#include "SerialCommands.h"

#include <Arduino.h>

#include "RobotParams.h"

namespace {
char serialBuff[32];
#define serialBuff_len sizeof(serialBuff) / sizeof(char)
} // namespace

void processSerialCommands() {
    while (Serial.available() > 0) {
        // Clear the buffer
        memset(serialBuff, 0, serialBuff_len);

        Serial.readBytesUntil('\n', serialBuff, serialBuff_len);
        Serial.println(serialBuff);

        // Tokenize the received char array to split at the whitespaces
        char* token = strtok(serialBuff, " ");
        uint8_t idx = 0;

        char *outArray[10];
        while (token != NULL) {
            outArray[idx] = token;
            idx++;
            token = strtok(NULL, " ");
        }
        uint8_t outArrayLength = idx;

        // if (!strcmp(outArray[0], "LC")) {
        //     leftTargetCurrent = atof(outArray[1]);
        //     constrain(leftTargetCurrent, -2, 2);

        //     //sendFloat(CMD_TARGET_CURRENT, leftTargetCurrent, LEFT_MOTOR_CAN_ADDR);
        // }

        // else if (!strcmp(outArray[0], "RC")) {
        //     rightTargetCurrent = atof(outArray[1]);
        //     constrain(rightTargetCurrent, -2, 2);
        //     //sendFloat(CMD_TARGET_CURRENT, rightTargetCurrent, RIGHT_MOTOR_CAN_ADDR);
        // }

        if (!strcmp(outArray[0], "SP")) {
            angleSetPoint = atof(outArray[1]) * DEG_TO_RAD;
            Serial.println(angleSetPoint);
        }

        else if (!strcmp(outArray[0], "DA")) {
            deactivationAngle = atof(outArray[1]);
            Serial.println(deactivationAngle);
        }

        else if (!strcmp(outArray[0], "K0")) {
            K[0] = atof(outArray[1]);
            Serial.println(K[0]);
        }
        else if (!strcmp(outArray[0], "K1")) {
            K[1] = atof(outArray[1]);
            Serial.println(K[1]);
        }
        else if (!strcmp(outArray[0], "K3")) {
            K[3] = atof(outArray[1]);
            Serial.println(K[3]);
        }
        else if (!strcmp(outArray[0], "K4")) {
            K[4] = atof(outArray[1]);
            Serial.println(K[4]);
        }

        else if (!strcmp(outArray[0], "CA")) {
            alpha = atof(outArray[1]);
            Serial.println(alpha);
        }

    }
}
