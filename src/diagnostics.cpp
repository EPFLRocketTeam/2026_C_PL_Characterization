#include "diagnostics.h"
#include "debug.h"

AcquisitionDiagnostics acquisition_diag;

void print_acquisition_diagnostics() {
#if DEBUG_
    Serial.println("\nACQUISITION DIAGNOSTICS (cumulative)");
    Serial.println("id,irq,irq_timestamp_overwrites,services,queued,ring_drops,latency_max_us,entries_min,entries_max,entries_299,entries_ge510,too_few,bad_request,bad_marker,drain_fail,discarded_words,lsm_incomplete,lsm_io_errors,pause_max_us,pause_mean_us,level_services,records_read,records_queued,fifo_overruns,fifo_resets,recovery_failures,unknown_tags,temp_read_errors,skipped_services,empty_services,pause_calls");
    for (uint8_t id = 1; id <= 7; ++id) {
        const SensorDiagnostics &d = acquisition_diag.sensor[id];
        Serial.print(id); Serial.print(',');
        Serial.print(d.interrupts); Serial.print(',');
        Serial.print(d.overwritten_irq_timestamps); Serial.print(',');
        Serial.print(d.services); Serial.print(',');
        Serial.print(d.queued_packets); Serial.print(',');
        Serial.print(d.ring_drops); Serial.print(',');
        Serial.print(d.max_service_latency_us); Serial.print(',');
        Serial.print(d.min_entries == UINT32_MAX ? 0 : d.min_entries); Serial.print(',');
        Serial.print(d.max_entries); Serial.print(',');
        Serial.print(d.entries_eq_299); Serial.print(',');
        Serial.print(d.entries_ge_510); Serial.print(',');
        Serial.print(d.too_few_entries); Serial.print(',');
        Serial.print(d.invalid_fifo_request); Serial.print(',');
        Serial.print(d.marker_failures); Serial.print(',');
        Serial.print(d.drain_failures); Serial.print(',');
        Serial.print(d.discarded_words); Serial.print(',');
        Serial.print(d.incomplete_lsm_batches); Serial.print(',');
        Serial.print(d.lsm_io_errors); Serial.print(',');
        Serial.print(d.max_pause_call_us); Serial.print(',');
        Serial.print(d.pause_calls ? (uint32_t)(d.sum_pause_call_us / d.pause_calls) : 0); Serial.print(',');
        Serial.print(d.level_services); Serial.print(',');
        Serial.print(d.records_read); Serial.print(',');
        Serial.print(d.records_queued); Serial.print(',');
        Serial.print(d.fifo_overruns); Serial.print(',');
        Serial.print(d.fifo_resets); Serial.print(',');
        Serial.print(d.fifo_recovery_failures); Serial.print(',');
        Serial.print(d.unknown_tags); Serial.print(',');
        Serial.print(d.temp_read_errors); Serial.print(',');
        Serial.print(d.skipped_services); Serial.print(',');
        Serial.print(d.empty_services); Serial.print(',');
        Serial.println(d.pause_calls);
    }
    Serial.print("ring_high_water_bytes="); Serial.println(acquisition_diag.ring_high_water_bytes);
    Serial.print("ring_drop_packets="); Serial.println(acquisition_diag.ring_drop_packets);
    Serial.print("sd_write_calls="); Serial.println(acquisition_diag.sd_write_calls);
    Serial.print("sd_short_writes="); Serial.println(acquisition_diag.sd_short_writes);
    Serial.print("sd_max_write_us="); Serial.println(acquisition_diag.sd_max_write_us);
    Serial.print("sd_close_failures="); Serial.println(acquisition_diag.sd_close_failures);
    Serial.print("sd_headroom_cache_hits="); Serial.println(acquisition_diag.sd_headroom_cache_hits);
    Serial.print("sd_headroom_cooldown_skips="); Serial.println(acquisition_diag.sd_headroom_cooldown_skips);
    Serial.print("sd_normal_write_calls="); Serial.println(acquisition_diag.sd_normal_write_calls);
    Serial.print("sd_normal_max_write_us="); Serial.println(acquisition_diag.sd_normal_max_write_us);
    Serial.print("sd_normal_budget_exceeded="); Serial.println(acquisition_diag.sd_normal_budget_exceeded);
    Serial.print("sd_headroom_probes="); Serial.println(acquisition_diag.sd_headroom_probes);
    Serial.print("sd_headroom_drains="); Serial.println(acquisition_diag.sd_headroom_drains);
    Serial.print("sd_headroom_deferrals="); Serial.println(acquisition_diag.sd_headroom_deferrals);
    Serial.print("sd_forced_low_headroom="); Serial.println(acquisition_diag.sd_forced_low_headroom);
    Serial.print("sd_busy_skips="); Serial.println(acquisition_diag.sd_busy_skips);
    print_bench_diagnostics();
#endif
}

// Consolidated bounded bench diagnostic traces and optional Serial formatting.
#include "accelerometer.h"
#include "debug.h"

BenchDiagnostics bench_diag;
const uint8_t lsm_snapshot_registers[14] = {
    0x07, 0x08, 0x09, 0x0a, 0x0d, 0x10, 0x11,
    0x12, 0x13, 0x15, 0x16, 0x17, 0x18, 0x63
};

void capture_lsm_registers(LSM6DSO32Sensor *sensor, uint8_t id) {
    if (!sensor || id < 3 || id > 4) return;
    LsmRegisterSnapshot &s = bench_diag.lsm[id-3];
    s = LsmRegisterSnapshot{};
    s.captured = true;
    for (uint8_t i = 0; i < 14; ++i)
        if (sensor->IO_Read(&s.values[i], lsm_snapshot_registers[i], 1) == LSM6DSO32_OK)
            s.valid_mask |= (uint16_t)1 << i;
}

void trace_adxl(uint8_t id, uint32_t sequence, uint32_t start_us,
                uint16_t before, uint16_t after_payload, uint32_t discarded,
                uint16_t flags, int16_t first_bad_word, uint16_t bad_word,
                const uint8_t *raw, bool raw_valid) {
#if !ACQ_BENCH_TRACES
    (void)id; (void)sequence; (void)start_us; (void)before;
    (void)after_payload; (void)discarded; (void)flags; (void)first_bad_word;
    (void)bad_word; (void)raw; (void)raw_valid;
    return;
#endif
    if (id < 1 || id > 2) return;
    // First 4 packet attempts plus subsequent faults. Fixed RAM; no live output.
    if (sequence >= 4 && !(flags & 0xde)) return;
    const uint8_t index = id-1;
    if (bench_diag.adxl_count[index] >= ADXL_TRACE_CAPACITY) {
        diag_inc(bench_diag.adxl_omitted[index]); return;
    }
    uint16_t first9 = 0, last9 = 0;
    if (raw_valid) for (uint8_t i = 0; i < 9; ++i) {
        first9 |= (uint16_t)(raw[i*2+1] & 1) << i;
        last9 |= (uint16_t)(raw[(291+i)*2+1] & 1) << i;
    }
    bench_diag.adxl[index][bench_diag.adxl_count[index]++] = {
        sequence, start_us, discarded, before, after_payload, flags,
        bad_word, first9, last9, first_bad_word, id
    };
}

void trace_sd_write(uint32_t start, uint32_t elapsed, uint32_t requested, uint32_t written, uint32_t ring_used, uint8_t pending_mask, uint8_t reason) {
#if !ACQ_BENCH_TRACES
    (void)start; (void)elapsed; (void)requested; (void)written;
    (void)ring_used; (void)pending_mask; (void)reason;
    return;
#endif
    SdWriteTrace row{start, elapsed, requested, written, ring_used, pending_mask, reason};
    if (elapsed > bench_diag.sd_worst.duration_us) bench_diag.sd_worst = row;
    if (elapsed < 10000) return;
    diag_inc(bench_diag.sd_long_calls);
    if (bench_diag.sd_count < SD_TRACE_CAPACITY)
        bench_diag.sd[bench_diag.sd_count++] = row;
    else diag_inc(bench_diag.sd_omitted);
}

void print_bench_diagnostics() {
#if DEBUG_
    Serial.print("\nBENCH_BUILD=ACQ_MISSION_SD_BLOCK"); Serial.println(ACQ_SD_BLOCK_BYTES);
    Serial.print("sd_admission_budget_us="); Serial.println(SD_ADMISSION_BUDGET_US);
    Serial.print("lsm_proactive_interval_us="); Serial.println(LSM_PROACTIVE_INTERVAL_US);
    Serial.println("LSM_REGISTER_SNAPSHOT (captured immediately after FIFO start)");
    Serial.println("id,address_hex,value_hex,read_ok");
    for (uint8_t j = 0; j < 2; ++j) {
        const LsmRegisterSnapshot &s = bench_diag.lsm[j];
        if (!s.captured) continue;
        for (uint8_t i = 0; i < 14; ++i) {
            Serial.print(j+3); Serial.print(',');
            Serial.print(lsm_snapshot_registers[i], HEX); Serial.print(',');
            Serial.print(s.values[i], HEX); Serial.print(',');
            Serial.println((s.valid_mask >> i) & 1);
        }
        if (s.valid_mask & (1U << 13)) {
            const int16_t fine = s.values[13] < 128 ? s.values[13] : (int16_t)s.values[13] - 256;
            Serial.print("lsm_freq_fine_signed_id="); Serial.print(j+3);
            Serial.print(",value="); Serial.println(fine);
        }
    }
    Serial.println("ADXL_READ_TRACE (first 4 attempts and bounded fault records)");
    Serial.println("id,sequence,start_us,entries_before,entries_after_300_words_or_FFFF,discarded_words,flags_hex,first_bad_word,bad_raw_word_hex,first9_ssi_hex,last9_ssi_hex");
    for (uint8_t j = 0; j < 2; ++j) {
        for (uint8_t i = 0; i < bench_diag.adxl_count[j]; ++i) {
            const AdxlTrace &r = bench_diag.adxl[j][i];
            Serial.print(r.id); Serial.print(','); Serial.print(r.sequence); Serial.print(',');
            Serial.print(r.start_us); Serial.print(','); Serial.print(r.before); Serial.print(',');
            Serial.print(r.after_payload); Serial.print(','); Serial.print(r.discarded); Serial.print(',');
            Serial.print(r.flags, HEX); Serial.print(','); Serial.print(r.first_bad_word); Serial.print(',');
            Serial.print(r.bad_word, HEX); Serial.print(','); Serial.print(r.first9, HEX); Serial.print(',');
            Serial.println(r.last9, HEX);
        }
        Serial.print("adxl_trace_omitted_id="); Serial.print(j+1);
        Serial.print(",count="); Serial.println(bench_diag.adxl_omitted[j]);
    }
    Serial.println("SD_LONG_WRITES (at least 10000 us; raw uint32 micros timestamps)");
    Serial.println("start_us,duration_us,requested_bytes,written_bytes,ring_used_bytes,pending_mask,reason");
    for (uint8_t i = 0; i < bench_diag.sd_count; ++i) {
        const SdWriteTrace &r = bench_diag.sd[i];
        Serial.print(r.start_us); Serial.print(','); Serial.print(r.duration_us); Serial.print(',');
        Serial.print(r.requested); Serial.print(','); Serial.print(r.written); Serial.print(',');
        Serial.print(r.ring_used); Serial.print(','); Serial.print(r.pending_mask); Serial.print(','); Serial.println(r.reason);
    }
    Serial.print("sd_long_calls="); Serial.println(bench_diag.sd_long_calls);
    Serial.print("sd_trace_omitted="); Serial.println(bench_diag.sd_omitted);
    Serial.print("sd_worst_start_us="); Serial.println(bench_diag.sd_worst.start_us);
    Serial.print("sd_worst_duration_us="); Serial.println(bench_diag.sd_worst.duration_us);
    Serial.print("sd_worst_pending_mask="); Serial.println(bench_diag.sd_worst.pending_mask);
    Serial.print("sd_worst_ring_used_bytes="); Serial.println(bench_diag.sd_worst.ring_used);
    Serial.print("sd_deferred_calls="); Serial.println(acquisition_diag.sd_deferred_calls);
    Serial.print("sd_forced_pending_writes="); Serial.println(acquisition_diag.sd_forced_pending_writes);
    Serial.print("sd_max_defer_us="); Serial.println(acquisition_diag.sd_max_defer_us);
    Serial.print("sd_mean_requested_bytes=");
    Serial.println(acquisition_diag.sd_write_calls ? (uint32_t)(acquisition_diag.sd_requested_bytes / acquisition_diag.sd_write_calls) : 0);
#endif
}


// Persistent diagnostic packets. All producers and serialization run
// in the foreground, through the same ring buffer as sensor packets.
#include "packets.h"
#include "bench_config.h"
#include <string.h>

namespace {
constexpr uint32_t DIAG_INTERVAL_MS = DIAGNOSTIC_PERIOD_MS;
constexpr size_t HEALTH_BYTES = 12U + 7U*(29U*4U+8U) + 19U*4U+8U+16U+24U;
uint32_t snapshot_sequence = 0;
uint32_t last_snapshot_ms = 0;
uint32_t next_retry_ms = 0;
uint32_t diag_failed_pushes = 0;
uint8_t sent_lsm = 0, sent_adxl[2] = {}, sent_sd = 0;
bool streaming_details = false;
uint8_t staging[sizeof(CHUNK_HEADER)+HEALTH_BYTES];
uint8_t *out;
void byte(uint8_t v) { *out++ = v; }
void u16(uint16_t v) { byte(uint8_t(v)); byte(uint8_t(v>>8)); }
void u32(uint32_t v) { for (uint8_t i=0;i<4;++i) byte(uint8_t(v>>(8*i))); }
void u64(uint64_t v) { for (uint8_t i=0;i<8;++i) byte(uint8_t(v>>(8*i))); }
bool write_diag_packet(uint8_t type, const uint8_t *payload, uint32_t len) {
    // Never compete with acquisition when ring pressure is already high.
    if (len + sizeof(CHUNK_HEADER) > RING_BUFFER_SIZE - 1U ||
        (RING_BUFFER_SIZE + rb_head - rb_tail) % RING_BUFFER_SIZE > RING_BUFFER_SIZE - 8192U) return false;
    CHUNK_HEADER h{};
    h.sync_word=0xaa;
    h.sensor_type=type;
    h.timestamp=micros();
    h.payload_len=len;
    // One contiguous packet: failure cannot leave orphaned headers in the ring.
    // Reuse static staging storage owned by the foreground serializer.
    if (len > HEALTH_BYTES) return false;
    memcpy(staging, &h, sizeof(h));
    memcpy(staging + sizeof(h), payload, len);
    return ring_buffer_push(staging, sizeof(h)+len);
}
uint8_t detail[48];
bool send_detail(uint8_t type) {
    const uint32_t len=uint32_t(out-detail);
    if (!write_diag_packet(type,detail,len)) { diag_inc(diag_failed_pushes); return false; }
    return true;
}
bool send_health(uint8_t reason) {
    // Serialize in a private buffer. No heap, Serial, or filesystem calls.
    uint8_t bytes[HEALTH_BYTES];
    out=bytes;
    byte(1); byte(reason); byte(7); byte(29);
    u32(snapshot_sequence); u32(millis());
    for (uint8_t id=1;id<=7;++id) {
        const SensorDiagnostics &d=acquisition_diag.sensor[id];
        u32(d.interrupts);
        u32(d.overwritten_irq_timestamps);
        u32(d.services);
        u32(d.skipped_services);
        u32(d.empty_services);
        u32(d.pause_calls);
        u32(d.level_services);
        u32(d.records_read);
        u32(d.records_queued);
        u32(d.fifo_overruns);
        u32(d.fifo_resets);
        u32(d.fifo_recovery_failures);
        u32(d.unknown_tags);
        u32(d.temp_read_errors);
        u32(d.max_service_latency_us);
        u32(d.queued_packets);
        u32(d.ring_drops);
        u32(d.too_few_entries);
        u32(d.invalid_fifo_request);
        u32(d.marker_failures);
        u32(d.drain_failures);
        u32(d.discarded_words);
        u32(d.incomplete_lsm_batches);
        u32(d.lsm_io_errors);
        u32(d.min_entries == UINT32_MAX ? 0 : d.min_entries);
        u32(d.max_entries);
        u32(d.entries_eq_299);
        u32(d.entries_ge_510);
        u32(d.max_pause_call_us);
        u64(d.sum_pause_call_us);
    }
    u32(acquisition_diag.ring_high_water_bytes);
    u32(acquisition_diag.ring_drop_packets);
    u32(acquisition_diag.sd_write_calls);
    u32(acquisition_diag.sd_short_writes);
    u32(acquisition_diag.sd_max_write_us);
    u32(acquisition_diag.sd_close_failures);
    u32(acquisition_diag.sd_headroom_cache_hits);
    u32(acquisition_diag.sd_headroom_cooldown_skips);
    u32(acquisition_diag.sd_normal_write_calls);
    u32(acquisition_diag.sd_normal_max_write_us);
    u32(acquisition_diag.sd_normal_budget_exceeded);
    u32(acquisition_diag.sd_headroom_probes);
    u32(acquisition_diag.sd_headroom_drains);
    u32(acquisition_diag.sd_headroom_deferrals);
    u32(acquisition_diag.sd_forced_low_headroom);
    u32(acquisition_diag.sd_busy_skips);
    u32(acquisition_diag.sd_deferred_calls);
    u32(acquisition_diag.sd_forced_pending_writes);
    u32(acquisition_diag.sd_max_defer_us);
    u64(acquisition_diag.sd_requested_bytes);
    u32(diag_failed_pushes);
    u32(ACQ_SD_BLOCK_BYTES);
    u32(SD_ADMISSION_BUDGET_US);
    u32(LSM_PROACTIVE_INTERVAL_US);
    u32(bench_diag.sd_long_calls);
    u32(bench_diag.sd_omitted);
    u32(bench_diag.adxl_omitted[0]);
    u32(bench_diag.adxl_omitted[1]);
    u32(bench_diag.sd_worst.start_us);
    u32(bench_diag.sd_worst.duration_us);
    if (size_t(out-bytes)!=HEALTH_BYTES) return false;
    if (!write_diag_packet(DIAG_HEALTH_V1,bytes,HEALTH_BYTES)) {
        diag_inc(diag_failed_pushes);
        return false;
    }
    diag_inc(snapshot_sequence);
    return true;
}
bool flush_one_detail() {
    if (sent_lsm < 2) {
        uint8_t j=sent_lsm;
        const LsmRegisterSnapshot &s=bench_diag.lsm[j];
        if (!s.captured) { ++sent_lsm; return true; }
        out=detail; byte(1); byte(uint8_t(j+3)); u16(s.valid_mask);
        for (uint8_t i=0;i<14;++i) byte(s.values[i]);
        if (!send_detail(DIAG_LSM_REG_V1)) return false;
        ++sent_lsm; return true;
    }
#if ACQ_BENCH_TRACES
    for (uint8_t j=0;j<2;++j) {
        if (sent_adxl[j]>=bench_diag.adxl_count[j]) continue;
        const AdxlTrace &r=bench_diag.adxl[j][sent_adxl[j]];
        out=detail; byte(1); byte(r.id);
        u32(r.sequence); u32(r.start_us); u32(r.discarded);
        u16(r.before); u16(r.after_payload); u16(r.flags);
        u16(r.bad_word); u16(r.first9); u16(r.last9);
        u16(uint16_t(r.first_bad_word));
        if (!send_detail(DIAG_ADXL_TRACE_V1)) return false;
        ++sent_adxl[j]; return true;
    }
    if (sent_sd < bench_diag.sd_count) {
        const SdWriteTrace &r=bench_diag.sd[sent_sd];
        out=detail; byte(1); byte(r.pending_mask); byte(r.reason); byte(0);
        u32(r.start_us); u32(r.duration_us); u32(r.requested);
        u32(r.written); u32(r.ring_used);
        if (!send_detail(DIAG_SD_TRACE_V1)) return false;
        ++sent_sd; return true;
    }
#endif
    return false;
}
}

void service_diagnostic_logging() {
    const uint32_t now=millis();
    if (uint32_t(now-last_snapshot_ms)>=DIAG_INTERVAL_MS &&
        int32_t(now-next_retry_ms)>=0) {
        // Advance only after a successful snapshot to allow a retry under pressure.
        if (send_health(0)) {
            last_snapshot_ms=now;
            streaming_details=true;
        } else {
            // Avoid serializing ~1 KiB repeatedly under ring pressure.
            next_retry_ms=now+250U;
        }
    }
    // At most one small trace packet per pass; leave the acquisition/SdFat
    // scheduler in charge of physical write timing.
    if (streaming_details) {
        if (!flush_one_detail()) streaming_details=false;
    }
}

void finalize_diagnostic_logging(uint8_t reason) {
    // On normal close and PFM, record the last available diagnostics before EOF.
    // Bounded traces and startup register snapshots are emitted once.
    // No Serial dependency, no new SD file.
    for (uint16_t attempt=0;attempt<160;++attempt) {
        if (!flush_one_detail()) break;
    }
    send_health(reason);
}

void reset_diagnostic_logging_for_test() {
    snapshot_sequence=last_snapshot_ms=next_retry_ms=diag_failed_pushes=0;
    sent_lsm=sent_sd=sent_adxl[0]=sent_adxl[1]=0;
    streaming_details=false;
}
