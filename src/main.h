#pragma once

#include <Arduino.h>
#include <stdio.h>

#include "debug.h"

#define CS_PIN_ADXL_Main 30
#define CS_PIN_LSM_Main 10
#define CS_PIN_BME_Main 28

#define CS_PIN_ADXL_Sat 17
#define CS_PIN_LSM_Sat 25
#define CS_PIN_BME_Sat 16

#define SPI_SCK 13
#define SPI_MISO 12
#define SPI_MOSI 11

#define SPI1_SCK 27
#define SPI1_MISO 1
#define SPI1_MOSI 26

#define PFM_PIN 14 // Power Failure Monitor

#define JUMPER_PIN 7

#define PFM_MONITOR_TIME 500 //ms

#define JUMPER_DEBOUNCE_TIME 500 //ms