#include "IMUDriver.h"

#include <Arduino.h>
#include <SPI.h>

// BMI160 Bosch Sensor API
#include "bmi160.h"
#include "bmi160_defs.h"

#include "Config.h"
#include "DeviceLog.h"
#include "PinConfig.h"
#include "RobotParams.h"

float theta_dot[3];  // {yaw, pitch, roll}
float theta[3];      // {yaw, pitch, roll}
float imuGyro[3];
float imuAccel[3];

namespace {

// Setup an SPI bus using FSPI on ESP32-S3
SPIClass *fspi = NULL;

const uint32_t BMI_spiClk = 10'000'000UL;  // 10 MHz SPI

struct bmi160_dev bmi; // BMI160 Struct for API

// Gyro offsets [rad/s] in the BMI160's axes, subtracted from every reading.
// Set by the gyro calibration (src/GyroCal.cpp) through setGyroOffsets().
volatile float gyroOffset[3] = {0, 0, 0};

// Gyro measurement (mean and spread over a number of samples), done by
// Task_BMI160 on the samples it reads anyway, so nothing else touches the
// SPI bus. Requested by startGyroMeasurement(); the task owns the running
// sums.
portMUX_TYPE measLock = portMUX_INITIALIZER_UNLOCKED;
volatile bool measRequested = false;
volatile bool measRunning = false;
uint16_t reqSkip = 0, reqTarget = 0;    // handed over under measLock
uint16_t measSkip = 0, measTarget = 0, measCount = 0;
double measSum[3], measSumSq[3], measAccel;
GyroMeasurement measResult = {};        // under measLock
volatile uint32_t measSeq = 0;          // finished measurements
bool imuOk = false;

// Task woken (FreeRTOS task notification) after every new orientation sample.
TaskHandle_t sampleListener = nullptr;

uint32_t prevMicros_gyro;

float gyroYaw;
float gyroPitch;
float gyroRoll;

float yaw;
float pitch;
float roll;

int8_t BMI160_SPI_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {

  // Begin SPI transaction at the SPI speed
  fspi->beginTransaction(SPISettings(BMI_spiClk, MSBFIRST, SPI_MODE0));

  // Select BMI160
  digitalWrite(CS_BMI, LOW);

  // Send register address, library clears write bit
  fspi->transfer(reg_addr);

  // Write the data
  for (uint16_t i = 0; i < len; i++) {
      fspi->transfer(data[i]);
  }

  // Deselect BMI160
  digitalWrite(CS_BMI, HIGH);

  // End SPI transaction
  fspi->endTransaction();

  return 0; // Return 0 on success
}

int8_t BMI160_SPI_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {

  // Begin SPI transaction at the SPI speed
  fspi->beginTransaction(SPISettings(BMI_spiClk, MSBFIRST, SPI_MODE0));

  // Select BMI160
  digitalWrite(CS_BMI, LOW);

  // Send register address with the read bit
  fspi->transfer(reg_addr);

  // Read the data
  for (uint16_t i = 0; i < len; i++) {
      data[i] = fspi->transfer(0x00);
  }

  // Deselect BMI160
  digitalWrite(CS_BMI, HIGH);

  // End SPI transaction
  fspi->endTransaction();

  return 0; // Return 0 on success
}

void BMI160_DELAY_ms(uint32_t period_ms) {
  delay(period_ms);
}

// Accelerometer scale [g per LSB], from the range actually set in the chip
// (read back in initIMU). Starts at the +-2 g value the code asks for.
float accelScale = 2.0f / 32768.0f;

float convertRawAccel(int16_t aRaw) {
  return aRaw * accelScale; // g
}

float convertRawGyro(int16_t gRaw) {
  // since we are using 2000 degrees/seconds range
  // -2000 maps to a raw value of -32768
  // +2000 maps to a raw value of 32767
  // Convert to radians

  float g = (gRaw * 2000.0f * DEG_TO_RAD) / 32768.0f;
  return g;
}

// Called by Task_BMI160 with every good sample.
void accumulateMeasurement(const float raw[3], float accelG) {
  if (!measRunning) {
    if (!measRequested) return;
    portENTER_CRITICAL(&measLock);
    measSkip = reqSkip;
    measTarget = reqTarget;
    measRequested = false;
    measRunning = true;
    portEXIT_CRITICAL(&measLock);
    measCount = 0;
    for (int i = 0; i < 3; i++) measSum[i] = measSumSq[i] = 0;
    measAccel = 0;
  }
  if (measSkip) { measSkip--; return; }
  for (int i = 0; i < 3; i++) {
    const double v = raw[i] * RAD_TO_DEG;
    measSum[i] += v;
    measSumSq[i] += v * v;
  }
  measAccel += accelG;
  if (++measCount < measTarget) return;

  GyroMeasurement m;
  m.samples = measCount;
  m.accelG = (float)(measAccel / measCount);
  for (int i = 0; i < 3; i++) {
    const double mean = measSum[i] / measCount;
    const double var = measSumSq[i] / measCount - mean * mean;
    m.mean[i] = (float)mean;
    m.std[i] = (float)sqrt(var > 0 ? var : 0);
  }
  portENTER_CRITICAL(&measLock);
  measResult = m;
  measRunning = false;
  measSeq = measSeq + 1;
  portEXIT_CRITICAL(&measLock);
}

} // namespace

// Soft-resets and configures the BMI160. False if it doesn't answer or the
// configuration fails.
static bool configureBmi160() {
  // Soft reset and chip ID check (0xD1).
  const int8_t initRslt = bmi160_init(&bmi);
  if (initRslt != BMI160_OK) {
    DeviceLog::printf("[imu] BMI160 not found (init %d, chip ID 0x%02X)\n", initRslt, bmi.chip_id);
    return false;
  }
  int8_t rslt;

  // Seth the accelerometer configuration
  bmi.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;      // Data output rate
  bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;      // Range
  bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;   // Bandwidth and filter
  bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;   // Power mode

  // Seth the accelerometer configuration
  bmi.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;        // Data output rate
  bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;  // Range
  bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;     // Bandwidth and filter
  bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;     // Power mode

  // Set the configuration
  rslt = bmi160_set_sens_conf(&bmi);

  // The Bosch API only writes ACC_RANGE when the requested range differs
  // from its idea of the chip's default (+-2 g), so asking for +-2 g never
  // actually writes it. On this robot the register was left at +-16 g, so
  // every reading came out 8x too small (1 g read as 0.125 g). Write it
  // explicitly, read it back, and scale by what the chip really uses.
  {
    uint8_t range = BMI160_ACCEL_RANGE_2G;
    bmi160_set_regs(BMI160_ACCEL_RANGE_ADDR, &range, 1, &bmi);
    delay(2);
    uint8_t conf[2] = {0, 0};
    bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR, conf, 2, &bmi);
    const uint8_t r = conf[1] & 0x0F;
    const float fullScaleG = (r == BMI160_ACCEL_RANGE_2G) ? 2.0f
                           : (r == BMI160_ACCEL_RANGE_4G) ? 4.0f
                           : (r == BMI160_ACCEL_RANGE_8G) ? 8.0f
                           : (r == BMI160_ACCEL_RANGE_16G) ? 16.0f : 0.0f;
    if (fullScaleG > 0) accelScale = fullScaleG / 32768.0f;
    DeviceLog::printf("[imu] accel config 0x%02X, range 0x%02X (+-%.0f g)%s\n", conf[0], conf[1], fullScaleG,
                      fullScaleG == 2.0f ? "" : fullScaleG > 0 ? " - NOT the +-2 g asked for" : " - unknown, assuming +-2 g");
  }
  // Same blind spot for the gyro: +-2000 deg/s is the API's assumed default,
  // so it's never written. Write it and check it (convertRawGyro assumes it).
  {
    uint8_t range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160_set_regs(BMI160_GYRO_RANGE_ADDR, &range, 1, &bmi);
    delay(2);
    uint8_t readBack = 0xFF;
    bmi160_get_regs(BMI160_GYRO_RANGE_ADDR, &readBack, 1, &bmi);
    DeviceLog::printf("[imu] gyro range 0x%02X%s\n", readBack,
                      (readBack & 0x07) == BMI160_GYRO_RANGE_2000_DPS ? " (+-2000 deg/s)" : " - NOT +-2000 deg/s, rates will be scaled wrong!");
  }

  if (rslt != BMI160_OK) {
    DeviceLog::printf("[imu] BMI160 configuration failed (%d)\n", rslt);
    return false;
  }
  return true;
}

// Reads the sensor directly for up to `ms` and returns true once the
// accelerometer reads a plausible total (0.7-1.3 g: it isn't being thrown
// around at power-on). An IMU that isn't answering reads all zeros (or all
// ones), which would look like an upright, motionless robot.
static bool readingsPlausible(uint32_t ms, float *lastG) {
  const uint32_t start = millis();
  float g = 0;
  do {
    struct bmi160_sensor_data a, gy;
    if (bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL, &a, &gy, &bmi) == BMI160_OK) {
      const float x = convertRawAccel(a.x), y = convertRawAccel(a.y), z = convertRawAccel(a.z);
      g = sqrtf(x * x + y * y + z * z);
      if (g > 0.7f && g < 1.3f) { *lastG = g; return true; }
    }
    delay(2);
  } while (millis() - start < ms);
  *lastG = g;
  return false;
}

bool initIMU() {
  // Set CS_BMI as output and make inactive
  pinMode(CS_BMI, OUTPUT);
  digitalWrite(CS_BMI, HIGH);

  // Create fspi instance
  fspi = new SPIClass(FSPI);
  fspi->begin(FSPI_SCK, FSPI_MISO, FSPI_MOSI);

  // Give the data to the struct
  bmi.id = 0;                       // Device ID
  bmi.intf = BMI160_SPI_INTF;       // Use SPI
  bmi.read = BMI160_SPI_read;       // Read function pointer
  bmi.write = BMI160_SPI_write;     // Write function pointer
  bmi.delay_ms = BMI160_DELAY_ms;   // Delay function pointer

  // Checked and retried: at power-on the BMI160 has occasionally not
  // answered, and then the controller saw a level, motionless robot (all
  // readings 0) and started balancing while it lay on its back.
  imuOk = false;
  for (uint8_t attempt = 1; attempt <= IMU_INIT_TRIES && !imuOk; attempt++) {
    if (attempt > 1) delay(IMU_INIT_RETRY_MS);
    float g = 0;
    if (!configureBmi160()) {
      DeviceLog::printf("[imu] attempt %u/%u: no BMI160\n", attempt, IMU_INIT_TRIES);
    } else if (!readingsPlausible(IMU_PLAUSIBLE_WAIT_MS, &g)) {
      DeviceLog::printf("[imu] attempt %u/%u: accelerometer reads %.3f g, not about 1 g\n", attempt, IMU_INIT_TRIES, g);
    } else {
      imuOk = true;
      DeviceLog::printf("[imu] BMI160 OK (chip ID 0x%02X, accel %.3f g)%s\n", bmi.chip_id, g,
                        attempt > 1 ? " after a retry" : "");
    }
  }
  if (!imuOk) DeviceLog::println("[imu] ERROR: BMI160 not working - the robot won't balance");
  return imuOk;
}

void setImuSampleListener(TaskHandle_t task) {
  sampleListener = task;
}

void Task_BMI160(void *pvParams) {
  // Initialised ONCE: xTaskDelayUntil() advances it by exactly one period per
  // call, giving a fixed sample rate. (Re-reading the tick count at the top of
  // every iteration made it a plain delay after the work, so the real period
  // was work time + period and the rate drifted below IMUsampleRate.)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool first = true; // start the angles from the accelerometer, not from 0

  while (1) {

    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;

    uint8_t rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &bmi);

    // Raw rates [rad/s], offsets not yet removed.
    const float rawGyro[3] = { convertRawGyro(gyro.x), convertRawGyro(gyro.y), convertRawGyro(gyro.z) };
    const float offX = gyroOffset[0], offY = gyroOffset[1], offZ = gyroOffset[2];

    // Get the angular velocities in rad/s
    theta_dot[0] = -(rawGyro[0] - offX); // Positive is clockwise
    theta_dot[1] = -(rawGyro[1] - offY); // Positive is forwards
    theta_dot[2] = (rawGyro[2] - offZ);  // Positive is clockwise



    uint32_t curMicros_gyro = micros();
    float dt = first ? 0.0f : (curMicros_gyro - prevMicros_gyro) / 1e6f;

    //Integrate the angular velocity over dt
    gyroYaw += (theta_dot[0] * dt);
    pitch += (theta_dot[1] * dt);
    roll += (theta_dot[2] * dt);


    float aX = convertRawAccel(accel.x);
    float aY = convertRawAccel(accel.y);
    float aZ = convertRawAccel(accel.z);
    if (rslt == BMI160_OK) accumulateMeasurement(rawGyro, sqrtf(aX * aX + aY * aY + aZ * aZ));

    imuGyro[0] = rawGyro[0] - offX;
    imuGyro[1] = rawGyro[1] - offY;
    imuGyro[2] = rawGyro[2] - offZ;
    imuAccel[0] = aX;
    imuAccel[1] = aY;
    imuAccel[2] = aZ;

    //filter.updateIMU(theta_dot[2], theta_dot[1], theta_dot[0], aZ, -aY, -aX);

    // Positive accelPitch means the robot is tilted clockwise
    // Unit: rad
    //float accelRoll = atan2(-aY, sqrt(convertRawAccel(accel.z) * convertRawAccel(accel.z) + convertRawAccel(accel.x) * convertRawAccel(accel.x)));
    float accelRoll = atan2(-aY, sqrt((aZ * aZ) + (aX * aX)));

    // Positive accelPitch means the robot is tilted forwards
    // Unit: radians
    //float accelPitch = atan2(-convertRawAccel(accel.z), sqrt(convertRawAccel(accel.y) * convertRawAccel(accel.y) + convertRawAccel(accel.x) * convertRawAccel(accel.x)));
    float accelPitch = atan2(-aZ, sqrt((aY * aY) + (aX * aX)));


    yaw = gyroYaw;
    // // Complementary Filter
    // pitch = ((alpha * gyroPitch) + ((1 - alpha) * accelPitch)) + pitchOffset;

    if (first) {
      // The filter would take a few seconds (several times its ~1 s time
      // constant) to come from 0 to the real angle, which is why setup()
      // used to wait 3 s before starting the controller. Only from a
      // plausible reading (about 1 g): a not-yet-valid sample (all zeros)
      // would start it at 0 deg, i.e. upright.
      pitch = accelPitch;
      roll = accelRoll;
      const float g = sqrtf(aX * aX + aY * aY + aZ * aZ);
      if (rslt == BMI160_OK && g > 0.7f && g < 1.3f) first = false;
    } else {
      pitch = (alpha * pitch) + ((1 - alpha) * accelPitch);
      roll = ((alpha * roll) + ((1 - alpha) * accelRoll));
    }

    // yaw = filter.getYawRadians();
    // pitch = filter.getPitchRadians();
    // roll = filter.getRollRadians();


    // Get the angles in °
    theta[0] = yaw;
    // theta[1] = pitch;
    // theta[2] = roll;
    theta[1] = pitch;
    theta[2] = roll;




    prevMicros_gyro = curMicros_gyro;





    // New sample is complete: wake the LQR task so it runs straight away on
    // a consistent set of theta/theta_dot values.
    if (sampleListener != nullptr) {
      xTaskNotifyGive(sampleListener);
    }

    // Serial.print(theta[0], 8);
    // Serial.print(";");
    // Serial.print(theta[1], 8);
    // Serial.print(";");
    // Serial.println(theta[2], 8);

    // Fixed rate. IMUsampleRate can be changed live from the web page; guard
    // against 0 and keep the period at least one tick. NOTE: the period is a
    // whole number of 1 ms ticks, so e.g. 300 Hz actually runs at 333 Hz.
    const uint16_t rate = IMUsampleRate ? IMUsampleRate : 1;
    TickType_t period = pdMS_TO_TICKS(1000 / rate);
    if (period == 0) period = 1;

    // If we've fallen more than a period behind (e.g. the rate was just
    // lowered), resync rather than sampling in a burst to catch up.
    if (xTaskGetTickCount() - xLastWakeTime > period) xLastWakeTime = xTaskGetTickCount();

    xTaskDelayUntil(&xLastWakeTime, period);
  }
  //vTaskDelay(1);
}

void Task_outputGyroData(void *pvParams) {
  while (1) {

    Serial.print(theta[2] * RAD_TO_DEG, 8);
    Serial.print(";");
    Serial.print(theta[1] * RAD_TO_DEG, 8);
    Serial.print(";");
    Serial.println(theta[0] * RAD_TO_DEG, 8);

    // Serial.print(quarternion[0], 8);
    // Serial.print(";");
    // Serial.print(quarternion[1], 8);
    // Serial.print(";");
    // Serial.print(quarternion[2], 8);
    // Serial.print(";");
    // Serial.println(quarternion[3], 8);


    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------------------------------------------------------------------------
// Gyro offsets and measurements
// ---------------------------------------------------------------------------

float GyroMeasurement::maxStd() const {
  return fmaxf(std[0], fmaxf(std[1], std[2]));
}

bool imuInitOk() { return imuOk; }

void setGyroOffsets(const float dps[3]) {
  for (int i = 0; i < 3; i++) gyroOffset[i] = dps[i] * DEG_TO_RAD;
}

void getGyroOffsets(float dps[3]) {
  for (int i = 0; i < 3; i++) dps[i] = gyroOffset[i] * RAD_TO_DEG;
}

bool startGyroMeasurement(uint32_t ms, uint32_t settleMs) {
  const uint32_t rate = IMUsampleRate ? IMUsampleRate : 1;
  uint32_t target = ms * rate / 1000;
  if (target < 10) target = 10;
  if (target > 60000) target = 60000;
  uint32_t skip = settleMs * rate / 1000;
  if (skip > 60000) skip = 60000;
  bool ok = false;
  portENTER_CRITICAL(&measLock);
  if (!measRequested && !measRunning) {
    reqTarget = (uint16_t)target;
    reqSkip = (uint16_t)skip;
    measRequested = true;
    ok = true;
  }
  portEXIT_CRITICAL(&measLock);
  return ok;
}

bool gyroMeasurementBusy() { return measRequested || measRunning; }

uint32_t gyroMeasurementCount() { return measSeq; }

GyroMeasurement lastGyroMeasurement() {
  portENTER_CRITICAL(&measLock);
  const GyroMeasurement m = measResult;
  portEXIT_CRITICAL(&measLock);
  return m;
}

bool measureGyro(uint32_t ms, uint32_t settleMs, GyroMeasurement &out) {
  const uint32_t before = gyroMeasurementCount();
  if (!startGyroMeasurement(ms, settleMs)) return false;
  // Generous: the sample rate can be lowered to 200 Hz from the web page.
  const uint32_t timeoutMs = 3 * (ms + settleMs) + 500;
  const uint32_t start = millis();
  while (gyroMeasurementCount() == before) {
    if (millis() - start > timeoutMs) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  out = lastGyroMeasurement();
  return true;
}
