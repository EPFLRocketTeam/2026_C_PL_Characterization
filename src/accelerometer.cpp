#include "accelerometer.h"

volatile bool accel_main_int = false;
volatile bool accel_sat_1_int = false;
volatile bool accel_sat_2_int = false;

volatile uint32_t accel_main_timestamp = 0;
volatile uint32_t accel_sat_1_timestamp = 0;
volatile uint32_t accel_sat_2_timestamp = 0;

ADXL372_PACKET adxl372_packet;

uint32_t adxl372_overruns[3] = {0, 0, 0};
uint32_t adxl372_invalid_blocks[3] = {0, 0, 0};

FASTRUN void accel_main_ISR() {
    accel_main_timestamp = micros();
    accel_main_int = true;
}

FASTRUN void accel_sat_1_ISR() {
    accel_sat_1_timestamp = micros();
    accel_sat_1_int = true;
}

FASTRUN void accel_sat_2_ISR() {
    accel_sat_2_timestamp = micros();
    accel_sat_2_int = true;
}

const char *fifo_order_name(FifoAxisOrder order) {
    if (order == FifoAxisOrder::YZX) return "YZX";
    if (order == FifoAxisOrder::ZXY) return "ZXY";
    return "XYZ";
}

int sensor_index(uint8_t sensor_id) {
    if (sensor_id < ID_ADXL372_MAIN || sensor_id > ID_ADXL372_SAT_2) {
        return -1;
    }

    return sensor_id - ID_ADXL372_MAIN;
}

bool setup_adxl372(ADXL372class *accel) {
    /*
    * Initializes an ADXL372 accelerometer and performs a self-test
    */
    accel->begin();
    
    if (!accel->isConnected()) {
        #ifdef DEBUG_
        accel->printDevice();
        Serial.println("ADXL372 is not connected");
        #endif
        return false; // Sensor is missing or SPI is dead
    }
    
    if (!accel->reset()) {
        #ifdef DEBUG_
        accel->printDevice();
        Serial.println("ADXL372 reset or post-reset ID check failed.");
        #endif
        return false;
    }

    // Silicon anomaly makes this unsafe
    // if (!accel->selfTest()) {
    //     #ifdef DEBUG_
    //     Serial.println("Self Test Accel Error.");
    //     accel->printDevice();
    //     #endif
    //     return false;
    // }

    #ifdef DEBUG_
    accel->printDevice();
    #endif

    accel->setOperatingMode(STANDBY);

    accel->setOdr(ODR_6400Hz);              //output data rate
    accel->setBandwidth(BW_3200Hz);
    accel->enableLowNoiseOperation(true);

    accel->disableHighPassFilter(true);
    accel->disableLowPassFilter(true);
    accel->setFilterSettling(FSP_16ms);

    accel->setFifoMode(FIFO_DISABLED);
    accel->setFifoFormat(XYZ);
    accel->setFifoSamples(255);
    accel->selectInt1Function(FIFO_FULL);
    
    accel->setOperatingMode(FULL_BANDWIDTH);

    delay(25); // Allow the 16 ms filter settling period to complete before collecting direct-register and FIFO means.

    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;

    for (uint8_t i = 0; i < 64; i++) {
        int16_t x, y, z;

        if (!accel->getRawAcceleration(x, y, z)) {
            return false;
        }

        sum_x += x;
        sum_y += y;
        sum_z += z;
    }

    accel->setFifoReference(sum_x / 64, sum_y / 64, sum_z / 64);

    // Leave the final FIFO configuration ready, but do not start it yet.
    accel->setOperatingMode(STANDBY);
    accel->setFifoMode(STREAM);

    adxl372_packet.header.sync_word = 0xAAAA;
    adxl372_packet.header.payload_len = sizeof(adxl372_packet.data);

    return true;
}

void start_adxl372(ADXL372class *accel, uint8_t interrupt_pin, void (*isr)()) {
    pinMode(interrupt_pin, INPUT);

    // Clear any old status before enabling the interrupt.
    accel->getStatus();

    attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr, RISING);
    accel->setOperatingMode(FULL_BANDWIDTH);
}


void print_adxl372_accel(ADXL372class *accel) {
    /*
    * Reads latest measurement and prints to Serial
    */
    float x, y, z;
    accel->readAcceleration(x, y, z);

    #ifdef DEBUG_
    char line[50];
    sprintf(line, "x: %f, y: %f, z: %f", x, y, z);
    Serial.println(line);
    #endif
}

void log_adxl372_fifo(ADXL372class *accel, uint32_t timestamp, uint8_t sensor_id) {
    int index = sensor_index(sensor_id);

    uint8_t status_before = accel->getStatus();
    bool valid = accel->readFifoData(adxl372_packet.data);
    uint8_t status_after = accel->getStatus();

    if (index >= 0 && ((status_before | status_after) & FIFO_OVR)) {
        adxl372_overruns[index]++;
    }

    if (!valid) {
        if (index >= 0) {
            adxl372_invalid_blocks[index]++;
        }
        return;
    }

    // The first FIFO packet is used only to determine the axis order.
    if (!accel->isFifoAxisOrderDetected()) {
        accel->detectFifoAxisOrder(
            adxl372_packet.data,
            ADXL372_PACKET_SAMPLES
        );

        #ifdef DEBUG_
        Serial.print("ADXL372 FIFO order detected: ");
        Serial.println(fifo_order_name(accel->getFifoAxisOrder()));
        #endif

        return;
    }

    adxl372_packet.header.sensor_type = sensor_id;
    adxl372_packet.header.timestamp = timestamp;
    
    ring_buffer_push((uint8_t*)&adxl372_packet, sizeof(adxl372_packet));
}

void print_adxl372_diagnostics() {
    #ifdef DEBUG_
    const char *names[3] = {"Main", "Sat 1", "Sat 2"};

    for (uint8_t i = 0; i < 3; i++) {
        Serial.print("ADXL372 ");
        Serial.print(names[i]);
        Serial.print(": overruns=");
        Serial.print(adxl372_overruns[i]);
        Serial.print(", invalid blocks=");
        Serial.println(adxl372_invalid_blocks[i]);
    }
    #endif
}
