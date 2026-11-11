#include "motor_control.h"

#include <SPI.h>
#include <SimpleFOCDrivers.h>

#include "encoders/as5048a/MagneticSensorAS5048A.h"
#include "drivers/drv8316/drv8316.h"

#include "pins.h"
#include "can_protocol.h"
#include "can_bus.h"
#include "motor_config.h"

static SPIClass SPI_1(SPI1_MOSI, SPI1_MISO, SPI1_SCK);

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

//  BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
//  - phA_h, phA_l - A phase pwm pin high/low pair
//  - phB_h, phB_l - B phase pwm pin high/low pair
//  - phB_h, phC_l - C phase pwm pin high/low pair
static DRV8316Driver6PWM driver = DRV8316Driver6PWM(PHA_H, PHA_L, PHB_H, PHB_L, PHC_H, PHC_L, DRV_CS, false);

static LowsideCurrentSense current_sense = LowsideCurrentSense(600.0, SO_A, SO_B, SO_C);

static MagneticSensorAS5048A sensor(ENC_CS, false, SPISettings(5'000'000, MSBFIRST, SPI_MODE1));

float target_current = 0;

bool enabled = false;

static uint32_t lastVelocitySendTime = 0;

static void printDRV8316Status() {
  DRV8316Status status = driver.getStatus();
	Serial.println("DRV8316 Status:");
	Serial.print("Fault: ");
	Serial.println(status.isFault());
	Serial.print("Buck Error: ");
	Serial.print(status.isBuckError());
	Serial.print("  Undervoltage: ");
	Serial.print(status.isBuckUnderVoltage());
	Serial.print("  OverCurrent: ");
	Serial.println(status.isBuckOverCurrent());
	Serial.print("Charge Pump UnderVoltage: ");
	Serial.println(status.isChargePumpUnderVoltage());
	Serial.print("OTP Error: ");
	Serial.println(status.isOneTimeProgrammingError());
	Serial.print("OverCurrent: ");
	Serial.print(status.isOverCurrent());
	Serial.print("  Ah: ");
	Serial.print(status.isOverCurrent_Ah());
	Serial.print("  Al: ");
	Serial.print(status.isOverCurrent_Al());
	Serial.print("  Bh: ");
	Serial.print(status.isOverCurrent_Bh());
	Serial.print("  Bl: ");
	Serial.print(status.isOverCurrent_Bl());
	Serial.print("  Ch: ");
	Serial.print(status.isOverCurrent_Ch());
	Serial.print("  Cl: ");
	Serial.println(status.isOverCurrent_Cl());
	Serial.print("OverTemperature: ");
	Serial.print(status.isOverTemperature());
	Serial.print("  Shutdown: ");
	Serial.print(status.isOverTemperatureShutdown());
	Serial.print("  Warning: ");
	Serial.println(status.isOverTemperatureWarning());
	Serial.print("OverVoltage: ");
	Serial.println(status.isOverVoltage());
	Serial.print("PowerOnReset: ");
	Serial.println(status.isPowerOnReset());
	Serial.print("SPI Error: ");
	Serial.print(status.isSPIError());
	Serial.print("  Address: ");
	Serial.print(status.isSPIAddressError());
	Serial.print("  Clock: ");
	Serial.print(status.isSPIClockFramingError());
	Serial.print("  Parity: ");
	Serial.println(status.isSPIParityError());
	if (status.isFault())
		driver.clearFault();
	delayMicroseconds(1); // ensure 400ns delay
	DRV8316_PWMMode val = driver.getPWMMode();
	Serial.print("PWM Mode: ");
	Serial.println(val);
	delayMicroseconds(1); // ensure 400ns delay
	bool lock = driver.isRegistersLocked();
	Serial.print("Lock: ");
	Serial.println(lock);
}

void setupFOC() {
    // initialise magnetic sensor hardware
    sensor.init(&SPI_1);
    // link the motor to the sensor
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = MOTOR_VOLTAGE_POWER_SUPPLY;
    driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;

    driver.init(&SPI_1);

    // small delay to allow SPI response
    delay(10);

    // Set the buck output tot 5V
    driver.setBuckVoltage(VB_5V);
    // Wait some time for everything to stabilise
    delay(500);

    // It's advised to have at least 400ns between read and write calls
    driver.setOCPLevel(Curr_16A);
    delayMicroseconds(1);
    driver.setOCPMode(AutoRetry_Fault);
    delayMicroseconds(1);
    driver.setOCPRetryTime(Retry500ms);
    delayMicroseconds(1);
    driver.setSlew(Slew_200Vus);
    delayMicroseconds(1);
    driver.setPWM100Frequency(FREQ_20KHz);
    delayMicroseconds(1);
    driver.setCurrentSenseGain(Gain_0V25); // 600 mV/A
    delayMicroseconds(1);
    driver.setPWMMode(PWM6_Mode);
    delayMicroseconds(1);

    Serial.println("Init complete...");

    delay(100);
    printDRV8316Status();

    motor.current_limit = MOTOR_CURRENT_LIMIT;

    current_sense.linkDriver(&driver);

    // Link driver to the motor
    motor.linkDriver(&driver);

    // aligning voltage
    motor.voltage_sensor_align = MOTOR_VOLTAGE_SENSOR_ALIGN;
    // choose FOC modulation
    motor.foc_modulation = FOCModulationType::SinePWM;

    motor.torque_controller = TorqueControlType::foc_current;

    // set motion control loop to be used
    motor.controller = MotionControlType::torque;

    // comment out if not needed
    motor.useMonitoring(Serial);
}

void startFOC() {
    // initialize motor
    motor.init();

    if (current_sense.init())  Serial.println("Current sense init success!");
    else {
        Serial.println("Current sense init failed!");
        return;
    }
    // link the current sense to the motor
    motor.linkCurrentSense(&current_sense);

    // align encoder and start FOC
    motor.initFOC();

    Serial.println(F("Motor ready."));

    enabled = true;

    _delay(1000);
}

void motorControlLoop() {
    if (!enabled) return;

    motor.loopFOC();
    motor.move(target_current);

    const uint32_t currentTime = _micros();
    if (currentTime - lastVelocitySendTime > SHAFT_VELOCITY_TELEMETRY_INTERVAL_US) {
        sendFloat(CMD_SHAFT_VELOCITY, motor.shaftVelocity(), CAN_SEND_TO);
        lastVelocitySendTime = currentTime;
    }
}
