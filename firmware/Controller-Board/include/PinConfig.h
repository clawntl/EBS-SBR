#pragma once

#include <Arduino.h>

// ----------------------------------------------
// SPI bus (BMI160 IMU)
// ----------------------------------------------
#define FSPI_MOSI     12
#define FSPI_MISO     10
#define FSPI_SCK      13
#define CS_BMI        11

#define BTN_PIN        1
#define BUZZER_PIN     2

#define I2C_SCL        6
#define I2C_SDA        7

// ----------------------------------------------
// Status LED (WS2812, driven via the Arduino-ESP32 core's built-in
// rgbLedWrite(), no external LED library needed)
// ----------------------------------------------
#define STATUS_LED_PIN   21
#define RGB_BRIGHTNESS   64

// ----------------------------------------------
// CAN transceiver
// ----------------------------------------------
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_8;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_9;
