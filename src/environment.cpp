#include "environment.h"

BME280_PACKET bme_packet;

bool setup_bme(Adafruit_BME280 *bme) {
    /*
    * Initializes aBME280 sensor
    */

    // TODO: Look into forced mode and settings

    bme_packet.header.sync_word = 0xAAAA;
    bme_packet.header.payload_len = sizeof(bme_packet.data);

    if (!bme->begin()) {
        #ifdef DEBUG_
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme->sensorID(),16);
        // Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        // Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        // Serial.print("        ID of 0x60 represents a BME 280.\n");
        // Serial.print("        ID of 0x61 represents a BME 680.\n");
        #endif
        return false;
    }

    bme->setSampling(Adafruit_BME280::MODE_NORMAL,     // Sleep until commanded
                     Adafruit_BME280::SAMPLING_X2,     // Temperature oversampling
                     Adafruit_BME280::SAMPLING_X16,    // Pressure oversampling (Max resolution)
                     Adafruit_BME280::SAMPLING_X1,     // Humidity oversampling
                     Adafruit_BME280::FILTER_X16,      // IIR Filter to block aerodynamic buffeting
                     Adafruit_BME280::STANDBY_MS_20    // Standby time 15.1Hz
    ); 

    return true;
}

void print_bme_values(Adafruit_BME280 *bme) {
    #ifdef DEBUG_
    Serial.print("Temperature = ");
    Serial.print(bme->readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme->readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme->readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme->readHumidity());
    Serial.println(" %");
    #endif
}

void log_bme_values(Adafruit_BME280 *bme, uint8_t sensor_id) {
    bme_packet.header.sensor_type = sensor_id;
    bme_packet.header.timestamp = micros();

    bme_packet.data.temperature = bme->readTemperature();
    bme_packet.data.pressure = bme->readPressure();
    bme_packet.data.humidity = bme->readHumidity();

    ring_buffer_push((uint8_t*)&bme_packet, sizeof(bme_packet));
}