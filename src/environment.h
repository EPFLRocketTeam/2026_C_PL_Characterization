#pragma once

#include "Adafruit_BME280.h"

#include "packets.h"
#include "debug.h"

#define SEALEVELPRESSURE_HPA (1013.25)

bool setup_bme(Adafruit_BME280 *bme);
void print_bme_values(Adafruit_BME280 *bme);
void log_bme_values(Adafruit_BME280 *bme, uint8_t sensor_id);