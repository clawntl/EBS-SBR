#include "IMUDriver.h"

#include <Arduino.h>
#include <SPI.h>

// BMI160 Bosch Sensor API
#include "bmi160.h"
#include "bmi160_defs.h"

#include "PinConfig.h"
#include "RobotParams.h"
#include "StatusLED.h"

float theta_dot[3];  // {yaw, pitch, roll}
float theta[3];      // {yaw, pitch, roll}

namespace {

// Setup an SPI bus using FSPI on ESP32-S3
SPIClass *fspi = NULL;

const uint32_t BMI_spiClk = 10'000'000UL;  // 10 MHz SPI

struct bmi160_dev bmi; // BMI160 Struct for API

float X_Offset;
float Y_Offset;
float Z_Offset;
float pitchOffset;

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

float convertRawAccel(int16_t aRaw) {
  // since we are using 2 g range
  // -2 g maps to a raw value of -32768
  // +2 g maps to a raw value of 32767
  // Units: m/s

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int16_t gRaw) {
  // since we are using 2000 degrees/seconds range
  // -2000 maps to a raw value of -32768
  // +2000 maps to a raw value of 32767
  // Convert to radians

  float g = (gRaw * 2000.0f * DEG_TO_RAD) / 32768.0f;
  return g;
}

void zeroIMU() {
  // Zero the BMI160
  struct bmi160_sensor_data accel;
  struct bmi160_sensor_data gyro;

  Serial.println("Zeroing......");

  float sum_X = 0, sum_Y = 0, sum_Z = 0;
  float sumPitch = 0;

  //uint32_t iterations = 100'000UL;
  uint32_t iterations = 1000;

  for (uint32_t i = 0; i < iterations; i++) {
    int8_t rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &bmi);

    if (rslt == BMI160_OK) {
      sum_X += convertRawGyro(gyro.x);
      sum_Y += convertRawGyro(gyro.y);
      sum_Z += convertRawGyro(gyro.z);

      // Compute pitch from accelerometer
      float aX = convertRawAccel(accel.x);
      float aY = convertRawAccel(accel.y);
      float aZ = convertRawAccel(accel.z);
      float accelPitch = atan2(-aZ, sqrt((aY * aY) + (aX * aX))); // Unit: rad

      sumPitch += accelPitch;

      Serial.print("X: "); Serial.print(convertRawGyro(gyro.x) * RAD_TO_DEG);
      Serial.print(" Y: "); Serial.print(convertRawGyro(gyro.y) * RAD_TO_DEG);
      Serial.print(" Z: "); Serial.print(convertRawGyro(gyro.z) * RAD_TO_DEG);
      Serial.print(" Accel Pitch: "); Serial.print(accelPitch * RAD_TO_DEG);
      Serial.println();

      delay(3);
    }
  }

  X_Offset = sum_X / (float)iterations;
  Y_Offset = sum_Y / (float)iterations;
  Z_Offset = sum_Z / (float)iterations;
  pitchOffset = sumPitch / (float)iterations;  // Store the reference angle

  Serial.print("SUM X: ");Serial.print(sum_X);
  Serial.print("\tOffset X: "); Serial.print(X_Offset * RAD_TO_DEG); Serial.println(" °/s");

  Serial.print("SUM Y: ");Serial.print(sum_Y);
  Serial.print("\tOffset Y: "); Serial.print(Y_Offset * RAD_TO_DEG); Serial.println(" °/s");

  Serial.print("SUM Z: ");Serial.print(sum_Z);
  Serial.print("\tOffset Z: "); Serial.print(Z_Offset * RAD_TO_DEG); Serial.println(" °/s");

  Serial.print("SUM Pitch: ");Serial.print(sumPitch);
  Serial.print("\tOffset Z: "); Serial.print(pitchOffset * RAD_TO_DEG); Serial.println(" °/s");
}

} // namespace

void initIMU() {
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

  // Initialise the BMI160 by performing a soft reset
  int8_t rslt;
  rslt = bmi160_init(&bmi);

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

  Serial.println("Starting to zero.....");
  //delay(2000);

  flashStatusLED(RGBColour{246, 20, 247}, 3, 150, 850); // 3 Purple Flashes

  // struct bmi160_offsets offsets;
  // struct bmi160_foc_conf foc_conf;
  // //rslt = bmi160_get_offsets(&offsets, &bmi);

  // foc_conf.acc_off_en = BMI160_ENABLE;

  // foc_conf.foc_acc_x  = BMI160_FOC_ACCEL_POSITIVE_G;
  // foc_conf.foc_acc_y  = BMI160_FOC_ACCEL_0G;
  // foc_conf.foc_acc_z  = BMI160_FOC_ACCEL_0G;

  // foc_conf.foc_gyr_en = BMI160_ENABLE;
  // foc_conf.gyro_off_en = BMI160_ENABLE;

  // rslt = bmi160_start_foc(&foc_conf, &offsets, &bmi);

  // Serial.print("X Off: "); Serial.print(offsets.off_gyro_x);
  // Serial.print("\tY Off: "); Serial.print(offsets.off_gyro_y);
  // Serial.print("\tZ Off: "); Serial.println(offsets.off_gyro_z);

  zeroIMU();

  flashStatusLED(RGBColour{246, 20, 247}, 3, 150, 150); // 3 Purple Flashes
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

  while (1) {

    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;

    uint8_t rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &bmi);



    // Get the angular velocities in rad/s
    theta_dot[0] = -(convertRawGyro(gyro.x) - X_Offset); // Positive is clockwise
    theta_dot[1] = -(convertRawGyro(gyro.y) - Y_Offset); // Positive is forwards
    theta_dot[2] = (convertRawGyro(gyro.z) - Z_Offset);  // Positive is clockwise



    uint32_t curMicros_gyro = micros();
    float dt = (curMicros_gyro - prevMicros_gyro) / 1e6f;

    //Integrate the angular velocity over dt
    gyroYaw += (theta_dot[0] * dt);
    pitch += (theta_dot[1] * dt);
    roll += (theta_dot[2] * dt);


    float aX = convertRawAccel(accel.x);
    float aY = convertRawAccel(accel.y);
    float aZ = convertRawAccel(accel.z);

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

    pitch = (alpha * pitch) + ((1 - alpha) * accelPitch);
    roll = ((alpha * roll) + ((1 - alpha) * accelRoll));

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
