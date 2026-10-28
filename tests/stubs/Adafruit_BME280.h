#pragma once
#include "SPI.h"
struct Adafruit_BME280 {
    enum { MODE_NORMAL, SAMPLING_X2, SAMPLING_X16, SAMPLING_X1, FILTER_X16, STANDBY_MS_20 };
    Adafruit_BME280(int,SPIClass*) {}
    bool begin() { return true; }
    void setSampling(int,int,int,int,int,int) {}
    int sensorID() { return 0; }
    float readTemperature() { return 0; }
    float readPressure() { return 0; }
    float readHumidity() { return 0; }
    float readAltitude(float) { return 0; }
};
