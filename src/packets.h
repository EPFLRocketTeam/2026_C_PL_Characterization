#pragma once

#include <stdint.h>

#include "ADXL371.h"

enum SensorID : uint8_t {
    ID_ADXL371_MAIN  = 0x01,
    ID_ADXL371_SAT   = 0x02,
    ID_LSM6_MAIN     = 0x03,
    ID_LSM6_SAT      = 0x04,
    ID_BME280_MAIN   = 0x05,
    ID_BME280_SAT    = 0x06,
    ID_MIC           = 0x07,
    PFM              = 0x08,
    EOF_             = 0xFF
};

struct CHUNK_HEADER {
  uint8_t sync_word;    // Magic number to find the start of a packet (e.g., 0xAAAA) ""Why not reduce to 0xAA with uint8_t"" 
  uint8_t sensor_type;  // ID for the sensor (e.g., 0x01 for ADXL371_MAIN)
  uint32_t timestamp;   // Timestamp of the block, can reconstruct timestamp of each measurement later
  size_t payload_len;   // How many bytes are in the attached buffer
} __attribute__((packed));

// Accelerometer
//==============================================================
// ADXL371 FIFO word : 2 bytes -> one int16_t sample
//==============================================================
constexpr uint16_t ADXL371_PACKET_SAMPLES = 100;  // One adxl_packet stores 3 FIFO samples (x,y,z) ==> 300 FIFO samples expected
struct ADXL371_PACKET {
  CHUNK_HEADER header;
  TRIPLET data[ADXL371_PACKET_SAMPLES];  // x, y, z acceleration
} __attribute__((packed));

//==============================================================
// LSM6DOS32 FIFO word : 7 bytes
//                        - 1 bytes Tag
//                        - 2 bytes X-axis measure
//                        - 2 bytes Y-axis
//                        - 2 byets Z-axis
//==============================================================
constexpr uint8_t LSM_PACKET_SAMPLES = 150;  // One LSM_packet stores 2 FIFO words ==> 300 FIFO words expected
struct LSM_FIFO_DATA {
    int16_t Ax;
    int16_t Ay;
    int16_t Az;
    int16_t Wx;
    int16_t Wy;
    int16_t Wz;
} __attribute__((packed));

struct LSM_PACKET {
    CHUNK_HEADER header;
    LSM_FIFO_DATA data[LSM_PACKET_SAMPLES];
    int16_t tmp;
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
    int16_t left_samples[MIC_CHUNK_SIZE];   // Left micro signal
    int16_t right_samples[MIC_CHUNK_SIZE];  // Right micro signal
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