#pragma once

#include <stdint.h>
#include <string.h>
#include "diagnostics.h"

#include "ADXL371.h"

// Logical IDs 1..7 index counters; wire IDs 0x11..0x14 carry high-rate data.
enum SensorID : uint8_t {
    ID_ADXL371_MAIN  = 0x01,
    ID_ADXL371_SAT   = 0x02,
    ID_LSM_MAIN      = 0x03,
    ID_LSM_SAT       = 0x04,
    ID_BME280_MAIN   = 0x05,
    ID_BME280_SAT    = 0x06,
    ID_MIC           = 0x07,
    PFM              = 0x08,
    // On-disk high-rate packet IDs. Keep these values for parser compatibility.
    PACKET_ADXL_MAIN = 0x11,
    PACKET_ADXL_SAT  = 0x12,
    PACKET_LSM_MAIN  = 0x13,
    PACKET_LSM_SAT   = 0x14,
    DIAG_HEALTH_V1   = 0x30,
    DIAG_LSM_REG_V1  = 0x31,
    DIAG_ADXL_TRACE_V1 = 0x32,
    DIAG_SD_TRACE_V1 = 0x33,
    EOF_             = 0xFF
};

struct CHUNK_HEADER {
  uint8_t sync_word;    // 0xAA
  uint8_t sensor_type;  // On-disk packet ID
  uint32_t timestamp;   // micros() service/dequeue timestamp, modulo 2^32
  uint32_t payload_len;   // How many bytes are in the attached buffer
} __attribute__((packed));

// Current acquisition packet layout (unchanged on disk).
constexpr uint16_t LSM_MAX_FIFO_RECORDS = 300;
constexpr uint16_t ADXL_MAX_BUFFER_SAMPLES = 170;
constexpr uint16_t ADXL_SAVED_SAMPLES = 100; // 300 axis words per normal acquisition.

enum AcquisitionFlags : uint16_t {
    ACQ_IRQ_VALID = 0x0001,
    ACQ_OVERRUN = 0x0002,
    ACQ_RESET = 0x0004,
    ACQ_IO_ERROR = 0x0008,
    ACQ_BAD_MARKER = 0x0010,
    ACQ_PARTIAL_TAIL = 0x0020,
    ACQ_BAD_TAG = 0x0040,
    ACQ_CAPACITY_ERROR = 0x0080,
    ACQ_TEMP_INVALID = 0x0100
};

struct AcquisitionMetadata {
    uint32_t sequence;          // Increments even if enqueue fails.
    uint32_t epoch;             // LSM: increments on recovery reset.
    uint32_t irq_timestamp_us;  // Only meaningful with ACQ_IRQ_VALID.
    uint32_t first_acc_index;   // LSM sample index within epoch; ADXL unused.
    uint32_t first_gyro_index;  // LSM sample index within epoch; ADXL unused.
    uint16_t fifo_entries_before;
    uint16_t record_count;      // ADXL XYZ triplets; LSM tagged 7-byte records.
    uint16_t flags;
    int16_t temperature_raw;    // LSM register temperature; ADXL unused.
} __attribute__((packed));

struct AdxlAcquisitionPacket {
    CHUNK_HEADER header;        // Timestamp = service start micros().
    AcquisitionMetadata meta;
    TRIPLET samples[ADXL_MAX_BUFFER_SAMPLES];
} __attribute__((packed));

struct LsmAcquisitionPacket {
    CHUNK_HEADER header;
    AcquisitionMetadata meta;
    uint8_t records[LSM_MAX_FIFO_RECORDS][7];
} __attribute__((packed));

static_assert(sizeof(AcquisitionMetadata) == 28, "Acquisition metadata binary layout changed");


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

// All producers and the consumer in this firmware run in the main loop.
// Do NOT call this function from an ISR or a second thread without redesigning
// ownership. Audio ISRs write their own AudioRecordQueue, not this ring.
inline bool ring_buffer_push(const uint8_t *data, uint32_t len) {
    const uint32_t head = rb_head;
    const uint32_t used = (RING_BUFFER_SIZE + head - rb_tail) % RING_BUFFER_SIZE;
    uint8_t id = (data && len >= 2) ? data[1] : 0;
    // High-rate on-disk IDs map to logical sensor counters 1..4.
    if (id >= PACKET_ADXL_MAIN && id <= PACKET_LSM_SAT) id -= 0x10;
    if (data == nullptr || len > RING_BUFFER_SIZE - used - 1) {
        diag_inc(acquisition_diag.ring_drop_packets);
        if (id > 0 && id < 8) diag_inc(acquisition_diag.sensor[id].ring_drops);
        return false;
    }
    const uint32_t first = len < RING_BUFFER_SIZE - head ? len : RING_BUFFER_SIZE - head;
    memcpy(ring_buffer + head, data, first);
    if (len > first) memcpy(ring_buffer, data + first, len - first);
    rb_head = (head + len) % RING_BUFFER_SIZE;
    diagnostic_max(acquisition_diag.ring_high_water_bytes, used + len);
    if (id > 0 && id < 8) diag_inc(acquisition_diag.sensor[id].queued_packets);
    return true;
}
