#pragma once
#include <Arduino.h>

// Only IRQ counters are changed from ISR. Other counters are main-loop owned.
// Persistent health packets are always active; Serial formatting is optional.
struct SensorDiagnostics {
    volatile uint32_t interrupts = 0;
    volatile uint32_t overwritten_irq_timestamps = 0;
    uint32_t services = 0;
    uint32_t skipped_services = 0;
    uint32_t empty_services = 0;
    uint32_t pause_calls = 0;
    uint32_t level_services = 0;
    uint32_t records_read = 0;
    uint32_t records_queued = 0;
    uint32_t fifo_overruns = 0;
    uint32_t fifo_resets = 0;
    uint32_t fifo_recovery_failures = 0;
    uint32_t unknown_tags = 0;
    uint32_t temp_read_errors = 0;
    uint32_t max_service_latency_us = 0;
    uint32_t queued_packets = 0;
    uint32_t ring_drops = 0;
    uint32_t too_few_entries = 0;
    uint32_t invalid_fifo_request = 0;
    uint32_t marker_failures = 0;
    uint32_t drain_failures = 0;
    uint32_t discarded_words = 0;
    uint32_t incomplete_lsm_batches = 0;
    uint32_t lsm_io_errors = 0;
    uint32_t min_entries = UINT32_MAX;
    uint32_t max_entries = 0;
    uint32_t entries_eq_299 = 0;
    uint32_t entries_ge_510 = 0;
    uint32_t max_pause_call_us = 0;
    uint64_t sum_pause_call_us = 0;
};

struct AcquisitionDiagnostics {
    SensorDiagnostics sensor[8]; // SensorID 1 through 7
    uint32_t ring_high_water_bytes = 0;
    uint32_t ring_drop_packets = 0;
    uint32_t sd_write_calls = 0;
    uint32_t sd_short_writes = 0;
    uint32_t sd_max_write_us = 0;
    uint32_t sd_close_failures = 0;
    uint32_t sd_headroom_cache_hits = 0;
    uint32_t sd_headroom_cooldown_skips = 0;
    uint32_t sd_normal_write_calls = 0;
    uint32_t sd_normal_max_write_us = 0;
    uint32_t sd_normal_budget_exceeded = 0;
    uint32_t sd_headroom_probes = 0;
    uint32_t sd_headroom_drains = 0;
    uint32_t sd_headroom_deferrals = 0;
    uint32_t sd_forced_low_headroom = 0;
    uint32_t sd_busy_skips = 0;
    uint32_t sd_deferred_calls = 0;
    uint32_t sd_forced_pending_writes = 0;
    uint32_t sd_max_defer_us = 0;
    uint64_t sd_requested_bytes = 0;
};

extern AcquisitionDiagnostics acquisition_diag;

// Counters deliberately saturate rather than silently wrapping during long runs.
inline void diag_inc(uint32_t &v) { if (v != UINT32_MAX) ++v; }
inline void diag_inc(volatile uint32_t &v) { if (v != UINT32_MAX) ++v; }
inline void diag_add(uint32_t &v, uint32_t n) {
    v = (n > UINT32_MAX - v) ? UINT32_MAX : v + n;
}
inline void diag_add(uint64_t &v, uint64_t n) {
    v = (n > UINT64_MAX - v) ? UINT64_MAX : v + n;
}

// Versioned binary diagnostic stream; independent of DEBUG_/Serial.
void service_diagnostic_logging();
void finalize_diagnostic_logging(uint8_t reason); // 1 normal, 2 power fail
void reset_diagnostic_logging_for_test();

inline void diagnostic_max(uint32_t &value, uint32_t candidate) {
    if (candidate > value) value = candidate;
}

// Consume the flag BEFORE service. Restore the original interrupt mask.
inline bool take_sensor_interrupt(volatile bool &pending,
                                  volatile uint32_t &irq_time,
                                  uint32_t &timestamp) {
    uint32_t saved_mask;
#if defined(__arm__)
    // Cortex-M7 / Teensy 4.x: avoid depending on optional CMSIS wrappers.
    __asm__ volatile("mrs %0, primask" : "=r" (saved_mask) :: "memory");
#else
    saved_mask = __get_PRIMASK(); // Host-test stub.
#endif
    __disable_irq();
    const bool ready = pending;
    if (ready) {
        timestamp = irq_time;
        pending = false;
    }
    if ((saved_mask & 1U) == 0) __enable_irq();
    return ready;
}

void print_acquisition_diagnostics();

constexpr uint8_t ADXL_TRACE_CAPACITY = 40;
constexpr uint8_t SD_TRACE_CAPACITY = 64;
struct AdxlTrace {
    uint32_t sequence, start_us, discarded;
    uint16_t before, after_payload, flags, bad_word, first9, last9;
    int16_t first_bad_word;
    uint8_t id;
};
struct SdWriteTrace { uint32_t start_us, duration_us, requested, written, ring_used; uint8_t pending_mask, reason; };
struct LsmRegisterSnapshot {
    uint8_t values[14] = {};
    uint16_t valid_mask = 0;
    bool captured = false;
};
struct BenchDiagnostics {
    AdxlTrace adxl[2][ADXL_TRACE_CAPACITY] = {};
    uint32_t adxl_omitted[2] = {};
    uint8_t adxl_count[2] = {};
    SdWriteTrace sd[SD_TRACE_CAPACITY] = {}, sd_worst = {};
    uint32_t sd_long_calls = 0, sd_omitted = 0;
    uint8_t sd_count = 0;
    LsmRegisterSnapshot lsm[2];
};
extern BenchDiagnostics bench_diag;
extern const uint8_t lsm_snapshot_registers[14];
void trace_adxl(uint8_t id, uint32_t sequence, uint32_t start_us,
                uint16_t before, uint16_t after_payload, uint32_t discarded,
                uint16_t flags, int16_t first_bad_word, uint16_t bad_word,
                const uint8_t *raw, bool raw_valid);
void trace_sd_write(uint32_t start, uint32_t elapsed, uint32_t requested, uint32_t written, uint32_t ring_used = 0, uint8_t pending_mask = 0, uint8_t reason = 0);
void print_bench_diagnostics();
