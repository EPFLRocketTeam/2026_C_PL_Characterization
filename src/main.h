#pragma once

#include <Arduino.h>
#include <stdio.h>

#include "debug.h"

#define CS_PIN_Accel_Main 0
#define CS_PIN_BME_Main 10

#define CS_PIN_Accel_Sat_1 15
#define CS_PIN_BME_Sat_1 14

#define CS_PIN_Accel_Sat_2 16

#define SPI_SCK 13
#define SPI_MISO 12
#define SPI_MOSI 11

#define SPI1_SCK 27
#define SPI1_MISO 1
#define SPI1_MOSI 26

#define PFM_PIN 18 // Power Failure Monitor

#define JUMPER_PIN 4

#define PFM_MONITOR_TIME 500 //ms

#define JUMPER_DEBOUNCE_TIME 500 //ms