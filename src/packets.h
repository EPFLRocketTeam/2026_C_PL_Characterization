#pragma once

#include <stdint.h>

#include "ADXL372.h"

enum SensorID : uint8_t {
    ID_ADXL372_MAIN  = 0x01,
    ID_ADXL372_SAT_1 = 0x02,
    ID_ADXL372_SAT_2 = 0x03,
    ID_BME280_MAIN   = 0x04,
    ID_BME280_SAT_1  = 0x05,
    ID_MIC           = 0x06,
    PFM              = 0x07,
    EOF_             = 0xFF
};

struct CHUNK_HEADER {
  uint16_t sync_word;   // Magic number to find the start of a packet (e.g., 0xAAAA)
  uint8_t sensor_type;  // ID for the sensor (e.g., 0x01 for ADXL372_MAIN)
  uint32_t timestamp;   // Timestamp of the block, can reconstruct timestamp of each measurement later
  uint32_t payload_len; // How many bytes are in the attached buffer
} __attribute__((packed));

// Accelerometer
constexpr uint16_t ADXL372_PACKET_SAMPLES = 83;
struct ADXL372_PACKET {
  CHUNK_HEADER header;
  TRIPLET data[ADXL372_PACKET_SAMPLES];
} __attribute__((packed));

// Environment
struct BME280_DATA {
    float temperature; // Celsius
    float pressure;    // Pascals or hPa
    float humidity;    // % Relative Humidity
} __attribute__((packed));

struct BME280_PACKET {
    CHUNK_HEADER header;
    BME280_DATA data;
} __attribute__((packed));

// Microphone
#define MIC_CHUNK_SIZE 128

struct Mic_PACKET {
    CHUNK_HEADER header;
    int16_t audio_samples[MIC_CHUNK_SIZE]; 
} __attribute__((packed));


// Buffer
constexpr uint32_t RING_BUFFER_SIZE_KB = 350;
constexpr uint32_t RING_BUFFER_SIZE = RING_BUFFER_SIZE_KB * 1024U;

extern uint8_t ring_buffer[RING_BUFFER_SIZE];
extern volatile uint32_t rb_head;
extern volatile uint32_t rb_tail;

// Push data into the buffer safely
inline bool ring_buffer_push(const uint8_t *data, uint32_t len) {
    __disable_irq(); // Pause interrupts to prevent race conditions
    
    uint32_t space_available = (RING_BUFFER_SIZE + rb_tail - rb_head - 1) % RING_BUFFER_SIZE;
    
    if (len > space_available) {
        __enable_irq();
        return false; // Buffer Overflow!
    }

    for (uint32_t i = 0; i < len; i++) {
        ring_buffer[rb_head] = data[i];
        rb_head = (rb_head + 1) % RING_BUFFER_SIZE;
    }
    
    __enable_irq();
    return true;
}

/* memcpy based version that would be faster (not yet needed)
inline bool ring_buffer_push(const uint8_t *data, uint32_t len) {
    __disable_irq();

    uint32_t space_available =
        (RING_BUFFER_SIZE + rb_tail - rb_head - 1)
        % RING_BUFFER_SIZE;

    if (len > space_available) {
        __enable_irq();
        return false;
    }

    uint32_t first_part = RING_BUFFER_SIZE - rb_head;

    if (first_part > len) {
        first_part = len;
    }

    memcpy(
        &ring_buffer[rb_head],
        data,
        first_part
    );

    uint32_t second_part = len - first_part;

    if (second_part > 0) {
        memcpy(
            ring_buffer,
            data + first_part,
            second_part
        );
    }

    rb_head = (rb_head + len) % RING_BUFFER_SIZE;

    __enable_irq();
    return true;
}
*/