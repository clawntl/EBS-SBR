#pragma once

// Motor construction parameter: number of pole pairs (magnet pole pairs on
// the rotor). See the motor's datasheet or count magnets and divide by 2.
#define MOTOR_POLE_PAIRS 11

// Supply voltage feeding the DRV8316, in Volts.
#define MOTOR_VOLTAGE_POWER_SUPPLY 12.0f

// Driver voltage limit, in Volts.
#define MOTOR_VOLTAGE_LIMIT 10.0f

// Motor phase current limit, in Amps.
#define MOTOR_CURRENT_LIMIT 2.0f

// Voltage applied during sensor alignment at startup, in Volts.
#define MOTOR_VOLTAGE_SENSOR_ALIGN 3.0f

// How often the shaft velocity is broadcast over CAN, in microseconds.
#define SHAFT_VELOCITY_TELEMETRY_INTERVAL_US 10'000
