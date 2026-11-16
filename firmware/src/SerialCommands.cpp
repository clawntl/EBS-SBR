#include "SerialCommands.h"

#include <Arduino.h>

#include "RobotParams.h"
#include "CANConfig.h"
#include "MotorDrivers.h"
#include "Settings.h"

#include <strings.h>

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
        // Split on spaces/tabs and also CR/LF: the PlatformIO monitor (and
        // most terminals) end lines with CR+LF, and readBytesUntil('\n')
        // leaves the CR in the buffer
        char* token = strtok(serialBuff, " \t\r\n");
        uint8_t idx = 0;

        char *outArray[10];
        while (token != NULL && idx < 10) {
            outArray[idx] = token;
            idx++;
            token = strtok(NULL, " \t\r\n");
        }
        uint8_t outArrayLength = idx;

        // Empty line: outArray[0] would be uninitialised.
        if (outArrayLength == 0) continue;

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

        // Tuning goes through Settings, so the web page stays in sync and
        // the values can be saved. Short names kept from the old commands.
        static const struct { const char *cmd; const char *key; } kAliases[] = {
            {"SP", "setpoint"}, {"DA", "deactAngle"}, {"CA", "alpha"},
            {"K0", "K0"}, {"K1", "K1"}, {"K2", "K2"}, {"K3", "K3"}, {"K4", "K4"}, {"K5", "K5"},
        };
        const char *settingKey = nullptr;
        for (const auto &a : kAliases) {
            if (!strcasecmp(outArray[0], a.cmd)) settingKey = a.key;
        }
        // SET <key> <value>: any setting by its key (LIST shows them)
        if (!strcasecmp(outArray[0], "SET") && outArrayLength >= 3) {
            settingKey = outArray[1];
            outArray[1] = outArray[2];
        }

        if (settingKey) {
            if (outArrayLength < 2) {
                Serial.printf("%s = %g\n", settingKey, Settings::value(settingKey));
            } else {
                float applied;
                if (Settings::set(settingKey, atof(outArray[1]), &applied)) Serial.printf("%s = %g\n", settingKey, applied);
                else Serial.printf("Unknown setting %s\n", settingKey);
            }
        }

        else if (!strcasecmp(outArray[0], "LIST")) {
            size_t n;
            const ParamDef *defs = Settings::params(n);
            for (size_t i = 0; i < n; i++) Serial.printf("  %-12s %10g   %s\n", defs[i].key, Settings::valueAt(i), defs[i].label);
            Serial.println(Settings::isDirty() ? "(unsaved changes - SAVE to keep them)" : "(all saved)");
        }

        else if (!strcasecmp(outArray[0], "SAVE")) {
            Serial.println(Settings::save() ? "Saved to flash" : "Save failed");
        }

        // Motor driver status
        else if (!strcmp(outArray[0], "MS")) {
            const char *names[2] = { "Left", "Right" };
            for (int s = 0; s < 2; s++) {
                const MotorDriverInfo m = getMotorDriverInfo((MotorSide)s);
                Serial.printf("%-5s: %-8s flags 0x%02X%s%s%s%s  start attempts %u",
                              names[s], motorLinkName(m.link), m.flags,
                              (m.flags & DRIVER_FLAG_COMMAND_TIMEOUT)  ? " cmd-timeout" : "",
                              (m.flags & DRIVER_FLAG_CALIBRATION_USED) ? " stored-align" : "",
                              (m.flags & DRIVER_FLAG_INVERTED)         ? " inverted" : "",
                              (m.flags & DRIVER_FLAG_DRV_FAULT)        ? " DRV-FAULT" : "",
                              m.startAttempts);
                if (m.lastError) Serial.printf("  last error: %s (%lu s ago)", driverErrorName(m.lastError), (unsigned long)(m.lastErrorAgeMs / 1000));
                Serial.println();
            }
        }

        // Motor retry: let a driver that gave up starting be tried again
        else if (!strcmp(outArray[0], "MR")) {
            resetMotorStartAttempts();
            Serial.println("Motor start attempts reset");
        }

    }
}
