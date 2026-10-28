#include "accelerometer.h"
#include "diagnostics.h"

volatile bool adxl_main_int = false;
volatile bool adxl_sat_int = false;

volatile uint32_t adxl_main_timestamp = 0;
volatile uint32_t adxl_sat_timestamp = 0;

volatile bool lsm_main_int = false;
volatile bool lsm_sat_int = false;

volatile uint32_t lsm_main_timestamp = 0;
volatile uint32_t lsm_sat_timestamp = 0;



//=================================================
// ISRs
//=================================================
FASTRUN void adxl_main_ISR() {
    diag_inc(acquisition_diag.sensor[ID_ADXL371_MAIN].interrupts);
    if (adxl_main_int) diag_inc(acquisition_diag.sensor[ID_ADXL371_MAIN].overwritten_irq_timestamps);
    adxl_main_timestamp = micros();
    adxl_main_int = true;
}

FASTRUN void adxl_sat_ISR() {
    diag_inc(acquisition_diag.sensor[ID_ADXL371_SAT].interrupts);
    if (adxl_sat_int) diag_inc(acquisition_diag.sensor[ID_ADXL371_SAT].overwritten_irq_timestamps);
    adxl_sat_timestamp = micros();
    adxl_sat_int = true;
}

FASTRUN void lsm_main_ISR() {
    diag_inc(acquisition_diag.sensor[ID_LSM_MAIN].interrupts);
    if (lsm_main_int) diag_inc(acquisition_diag.sensor[ID_LSM_MAIN].overwritten_irq_timestamps);
    lsm_main_timestamp = micros();
    lsm_main_int = true;
}

FASTRUN void lsm_sat_ISR() {
    diag_inc(acquisition_diag.sensor[ID_LSM_SAT].interrupts);
    if (lsm_sat_int) diag_inc(acquisition_diag.sensor[ID_LSM_SAT].overwritten_irq_timestamps);
    lsm_sat_timestamp = micros();
    lsm_sat_int = true;
}

//=================================================
// ADXL371
//=================================================
bool setup_adxl371(ADXL371class *accel) {
    if (accel == nullptr) return false;

    accel->begin();
    
    if (!accel->isConnected()) {
        #if DEBUG_
        accel->printDevice();
#if DEBUG_
        Serial.println("ADXL371 is not connected");
#endif
        #endif
        return false; // Sensor is missing or SPI is dead
    }
    
    if (!accel->reset()) {
        #if DEBUG_
        accel->printDevice();
#if DEBUG_
        Serial.println("ADXL371 reset or post-reset ID check failed.");
#endif
        #endif
        return false;
    }


    #if DEBUG_
    accel->printDevice();
    #endif
    // Configure sensor filtering and FIFO workaround.
    accel->setOperatingMode(STANDBY);           // Disable measur for setup
    accel->setOdr(ODR_5120Hz);                  //output data rate
    accel->setBandwidth(BW_1280Hz);             //low-pass filter cutoff frequency
    accel->enableLowNoiseOperation(true);

    accel->disableHighPassFilter(true);         //keep low acceleration in signal
    accel->disableLowPassFilter(false);         //filter to higher frequency (Nyquist-Shanon) 
    accel->setFilterSettling(FSP_4_OVER_ODR);   //short settling time
    // accel->setOperatingMode(FULL_BANDWIDTH);    // Start calibration measure
    
    // Critical FIFO config with workaround procedure
    accel->setOperatingMode(STANDBY);           // Stop measure
    accel->enableExternalTrigger(true);         // Enable external trigger (block ADC)
    accel->setFifoMode(FIFO_DISABLED);          // Clear FIFO
    accel->setFifoSamples(3 * ADXL_SAVED_SAMPLES);                 // set FIFO watermarks
    accel->setFifoFormat(XYZ);                  // set data format
    accel->selectInt1Function(FIFO_FULL);       // int1 setup (when watermarks reached)     
    accel->setFifoMode(STREAM);                 // continious FIFO writing
    accel->setOperatingMode(FULL_BANDWIDTH);    // Ready to satrt (waiting trigger)

    return true;
}

void start_adxl371(ADXL371class *accel, uint8_t interrupt_pin, void (*isr)()) {
    pinMode(interrupt_pin, INPUT);
    accel->getStatus();
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr, RISING);

    // Clear any old status before enabling the interrupt.
    accel->enableExternalTrigger(false);    // Deblock ADC -> start measure
    //accel->setOperatingMode(FULL_BANDWIDTH); 
}




//=================================================
// LSM6DSO32
//=================================================
bool setup_lsm(LSM6DSO32Sensor *accel) {
    if (accel == nullptr) return false;

    // Initialize interface
    if (accel->begin() != LSM6DSO32_OK) {
        #if DEBUG_
#if DEBUG_
        Serial.println("LSM6DSO32 begin() failed.");
#endif
        #endif
        return false;
    }
    // Explicitly check physical sensor presence
    uint8_t id = 0;
    if (accel->IO_Read(&id, 0x0F, 1) != 0) {
        #if DEBUG_
#if DEBUG_
        Serial.println("WHO_AM_I read ERROR");
#endif
        #endif
        return false;
    }
    #if DEBUG_
#if DEBUG_
    Serial.print("WHO_AM_I = 0x");
#endif
#if DEBUG_
    Serial.println(id, HEX);
#endif
    #endif

    if (id != 0x6C) {
        #if DEBUG_
#if DEBUG_
        Serial.println("LSM6DSO32 NOT DETECTED");
#endif
        #endif
        return false;
    }
    #if DEBUG_
#if DEBUG_
    Serial.println("LSM6DSO32 initialized successfully."); 
#endif
    #endif

    // Preserve the established LSM sampling settings.
    // 0x10 : Accel ODR=6667Hz 1010b, FS=+/-32g 01b LFP2_EN 0b + 0b -> 0xA4
    // 0x11 : Gyro ODR=6667Hz 1010b FS=+/-2000dps 11b LFP2 0b + 0b -> 0xAC
    // 0x12 : Enable multiregister access
    uint8_t ctrl_regs[3] = {0xA4, 0xAC, 0x44};
    if (accel->IO_Write(ctrl_regs, 0x10, 3) != 0) return false;

    // FIFO config
    // 0x07 : Set watermark à 300
    // 0x08 : Set watermark
    // 0x09 : Set FIFO X/G BDR => 0xAA
    // 0x0A : Set FIFO Bypass mode to clean memory
    uint8_t fifo_regs[4] = {0x2C, 0x01, 0xAA, 0x00};
    if (accel->IO_Write(fifo_regs, 0x07, 4) != 0) {
        #if DEBUG_
#if DEBUG_
        Serial.println("FIFO Config FAILED");
#endif
        #endif
        return false;
    }

    // Set int1 to trigger on watermark threshold

    uint8_t int1_ctrl = 0;
    if (accel->IO_Read(&int1_ctrl, 0x0D, 1) != 0) return false;
    int1_ctrl |= (1 << 3); 
    if (accel->IO_Write(&int1_ctrl, 0x0D, 1) != 0) {
        #if DEBUG_
#if DEBUG_
        Serial.println("INT1_CTRL write ERROR");
#endif
        #endif
        return false;
    }

    #if DEBUG_
    uint8_t check = 0;
    if (accel->IO_Read(&check, 0x0D, 1) == 0) {
#if DEBUG_
        Serial.print("INT1_CTRL after = 0x");
#endif
#if DEBUG_
        Serial.println(check, HEX);
#endif
    }
    #endif

    return true;
}

void start_lsm(LSM6DSO32Sensor *accel, uint8_t interrupt_pin, void (*isr)()) {
    if (accel == nullptr || isr == nullptr) return;

    // Attach interrupt pin
    pinMode(interrupt_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr, RISING);

    // Start data acquisition in FIFO
    uint8_t mode = 0x06; // Continuous FIFO; no stop at watermark.
    if (accel->IO_Write(&mode, 0x0A, 1) != LSM6DSO32_OK) {
        #if DEBUG_
#if DEBUG_
        Serial.println("FIFO MODE fail !");
#endif
        #endif
        return;
    }

    // Successful startup is recorded by the register snapshot; no live printing.
}










// Continuous high-rate acquisition and FIFO headroom accounting.
#include "accelerometer.h"
#include "accelerometer.h"

namespace {
struct StreamState {
    uint32_t sequence = 0, epoch = 0, acc_index = 0, gyro_index = 0;
    bool needs_recovery = false;
    bool probed_overrun = false;
    bool sd_cache_known = false, sd_cache_safe = false;
    uint32_t sd_snapshot_us = 0;
    uint16_t sd_snapshot_entries = 0;
};
StreamState states[5]; // Logical sensor IDs 1..4.
AdxlAcquisitionPacket adxl;
LsmAcquisitionPacket lsm;

void init_meta(CHUNK_HEADER &h, AcquisitionMetadata &m, uint8_t logical_id,
               uint32_t started, uint32_t irq, bool irq_valid) {
    StreamState &s = states[logical_id];
    h.sync_word = 0xaa;
    h.sensor_type = logical_id + 0x10; // Stable recorded packet IDs 0x11..0x14.
    h.timestamp = started;
    h.payload_len = sizeof(m);
    memset(&m, 0, sizeof(m));
    m.sequence = s.sequence++;
    m.epoch = s.epoch;
    m.first_acc_index = s.acc_index;
    m.first_gyro_index = s.gyro_index;
    m.irq_timestamp_us = irq_valid ? irq : 0;
    m.flags = irq_valid ? ACQ_IRQ_VALID : 0;
}

bool lsm_status(LSM6DSO32Sensor *sensor, uint16_t &entries, uint8_t &flags) {
    uint8_t status[2] = {};
    if (sensor->IO_Read(status, 0x3a, 2) != LSM6DSO32_OK) return false;
    entries = (uint16_t)status[0] | ((uint16_t)(status[1] & 3) << 8);
    flags = status[1];
    return true;
}

void recover_lsm(LSM6DSO32Sensor *sensor, uint8_t id, uint16_t reason) {
    SensorDiagnostics &d = acquisition_diag.sensor[id];
    StreamState &s = states[id];
    lsm.meta.record_count = 0;
    lsm.meta.flags |= reason | ACQ_RESET;
    lsm.header.payload_len = sizeof(lsm.meta);
    ring_buffer_push((const uint8_t*)&lsm, sizeof(CHUNK_HEADER) + sizeof(lsm.meta));
    const bool bypass_ok = sensor->Write_Reg(0x0a, 0x00) == LSM6DSO32_OK;
    const bool stream_ok = sensor->Write_Reg(0x0a, 0x06) == LSM6DSO32_OK;
    diag_inc(d.fifo_resets);
    s.needs_recovery = !(bypass_ok && stream_ok);
    s.probed_overrun = false;
    if (s.needs_recovery) diag_inc(d.fifo_recovery_failures);
    ++s.epoch;
    s.acc_index = s.gyro_index = 0;
}
}

bool lsm_recovery_pending(uint8_t id) {
    return id >= ID_LSM_MAIN && id <= ID_LSM_SAT && states[id].needs_recovery;
}

// The cache is valid only for the default budget. A failure is cached unsafe
// until a FIFO service invalidates it; repeated SD attempts cannot hammer SPI.
bool lsm_cached_headroom_valid(uint8_t id) {
    if (id < ID_LSM_MAIN || id > ID_LSM_SAT) return false;
    StreamState &s = states[id];
    if (!s.sd_cache_known || !s.sd_cache_safe || s.needs_recovery || s.probed_overrun)
        return false;
    const uint32_t age = micros() - s.sd_snapshot_us;
    const uint64_t growth = (uint64_t(age + uint64_t(SD_ADMISSION_BUDGET_US)) *
                            LSM_SD_INPUT_RATE_HZ + 999999ULL) / 1000000ULL;
    // Two extra records account for accel/gyro arrivals on the same time slot.
    s.sd_cache_safe = uint64_t(s.sd_snapshot_entries) + growth + 2U < LSM_SD_SAFE_CAPACITY;
    return s.sd_cache_safe;
}

bool lsm_safe_for_sd(LSM6DSO32Sensor *sensor, uint8_t id, uint32_t budget_us) {
    if (!sensor || id < ID_LSM_MAIN || id > ID_LSM_SAT) return false;
    StreamState &s = states[id];
    s.sd_cache_known = budget_us == SD_ADMISSION_BUDGET_US;
    s.sd_cache_safe = false;
    if (s.needs_recovery || s.probed_overrun) return false;
    uint16_t entries = 0;
    uint8_t status = 0;
    s.sd_snapshot_us = micros(); // age from BEFORE the SPI transfer
    diag_inc(acquisition_diag.sd_headroom_probes);
    if (!lsm_status(sensor, entries, status)) return false;
    if (status & 0x48U) { s.probed_overrun = true; return false; }
    s.sd_snapshot_entries = entries;
    if (s.sd_cache_known) {
        s.sd_cache_safe = true;
        return lsm_cached_headroom_valid(id);
    }
    const uint64_t growth = ((uint64_t(budget_us) + uint32_t(micros() - s.sd_snapshot_us)) *
                            LSM_SD_INPUT_RATE_HZ + 999999ULL) / 1000000ULL;
    return uint64_t(entries) + growth + 2U < LSM_SD_SAFE_CAPACITY;
}

bool lsm_cached_safe_for_sd(LSM6DSO32Sensor *sensor, uint8_t id) {
    if (!sensor || id < ID_LSM_MAIN || id > ID_LSM_SAT) return false;
    if (!states[id].sd_cache_known) return lsm_safe_for_sd(sensor, id);
    diag_inc(acquisition_diag.sd_headroom_cache_hits);
    if (lsm_cached_headroom_valid(id)) return true;
    // Cached headroom expired: refresh once from the FIFO status registers
    // before scheduling an expensive optional drain.
    return lsm_safe_for_sd(sensor, id);
}

void log_adxl371_fifo(ADXL371class *sensor, uint32_t irq, uint8_t id, bool irq_valid) {
    if (!sensor || id < 1 || id > 2) return;
    SensorDiagnostics &d = acquisition_diag.sensor[id];
    const uint32_t started = micros();
    diag_inc(d.services);
    if (irq_valid) diagnostic_max(d.max_service_latency_us, started - irq);
    else diag_inc(d.level_services);

    // Register-only preflight: do not stop/reset the ADC for an empty/stale IRQ.
    const uint16_t preflight = sensor->getFifoEntries();
    if (preflight < 300) {
        diag_inc(d.too_few_entries);
        diag_inc(d.skipped_services);
        if (preflight == 0) diag_inc(d.empty_services);
        return;
    }
    init_meta(adxl.header, adxl.meta, id, started, irq, irq_valid);
    uint8_t raw[600] = {};
    uint16_t before = 0;
    uint32_t discarded = 0;
    const uint32_t pause_start = micros();
    sensor->enableExternalTrigger(true);
    const uint8_t status = sensor->getStatus();
    // Restore the earlier fixed-size read: only this first 300-word block is data.
    const AdxlFifoResult result = sensor->readFifoRawData(raw, sizeof(raw), before);
    // drainFifo() reads occupancy itself. Avoid a redundant diagnostic-only
    // SPI transaction while the ADXL ADC is paused.
    const bool drained = sensor->drainFifo(&discarded);
    sensor->setOperatingMode(STANDBY);
    sensor->setFifoMode(FIFO_DISABLED);
    sensor->setFifoMode(STREAM);
    sensor->setOperatingMode(FULL_BANDWIDTH);
    sensor->getStatus();
    sensor->enableExternalTrigger(false);
    const uint32_t pause_us = micros() - pause_start;
    diag_inc(d.pause_calls);
    diagnostic_max(d.max_pause_call_us, pause_us);
    diag_add(d.sum_pause_call_us, pause_us);
    adxl.meta.fifo_entries_before = before;
    if (before < d.min_entries) d.min_entries = before;
    diagnostic_max(d.max_entries, before);
    if (before == 299) diag_inc(d.entries_eq_299);
    if (before >= 510) diag_inc(d.entries_ge_510);
    bool ok = result == AdxlFifoResult::Ok && drained;
    if (!drained) { diag_inc(d.drain_failures); adxl.meta.flags |= ACQ_IO_ERROR; }
    if (result == AdxlFifoResult::TooFewEntries) {
        diag_inc(d.too_few_entries);
        adxl.meta.flags |= ACQ_IO_ERROR;
        adxl.meta.flags &= ~ACQ_IRQ_VALID;
        adxl.meta.irq_timestamp_us = 0;
    } else if (result != AdxlFifoResult::Ok) {
        diag_inc(d.invalid_fifo_request);
        adxl.meta.flags |= ACQ_CAPACITY_ERROR;
    }
    if (status & FIFO_OVR) {
        diag_inc(d.fifo_overruns); adxl.meta.flags |= ACQ_OVERRUN; ok = false;
    }
    // Decode/inspect after resuming. Keep strict SSI validation; never rotate axes.
    int16_t first_bad = -1;
    uint16_t bad_word = 0;
    if (result == AdxlFifoResult::Ok) {
        for (uint16_t w = 0; w < 300; ++w) {
            if ((raw[2*w+1] & 1) != (w % 3 == 0)) {
                first_bad = w;
                bad_word = ((uint16_t)raw[2*w] << 8) | raw[2*w+1];
                break;
            }
        }
        if (first_bad >= 0) {
            diag_inc(d.marker_failures); adxl.meta.flags |= ACQ_BAD_MARKER; ok = false;
        }
    }
    if (ok && sensor->decodeFifoData(raw, adxl.samples, ADXL_MAX_BUFFER_SAMPLES) != AdxlFifoResult::Ok) {
        diag_inc(d.invalid_fifo_request); adxl.meta.flags |= ACQ_CAPACITY_ERROR; ok = false;
    }
    if (ok) {
        adxl.meta.record_count = 100;
        diag_add(d.records_read, 100);
    } else if (result == AdxlFifoResult::Ok) {
        discarded += 300;
    }
    diag_add(d.discarded_words, discarded);
    trace_adxl(id, adxl.meta.sequence, started, before, 0xffffU, discarded,
               adxl.meta.flags, first_bad, bad_word, raw,
               result == AdxlFifoResult::Ok);
    adxl.header.payload_len += adxl.meta.record_count * sizeof(TRIPLET);
    if (ring_buffer_push((const uint8_t*)&adxl, sizeof(CHUNK_HEADER) + adxl.header.payload_len))
        diag_add(d.records_queued, adxl.meta.record_count);
}

void log_lsm_fifo(LSM6DSO32Sensor *sensor, uint32_t irq, uint8_t id, bool irq_valid) {
    if (!sensor || id < 3 || id > 4) return;
    SensorDiagnostics &d = acquisition_diag.sensor[id];
    StreamState &s = states[id];
    s.sd_cache_known = false; // Reads, reset and recovery all invalidate occupancy.
    const uint32_t started = micros();
    diag_inc(d.services);
    if (irq_valid) diagnostic_max(d.max_service_latency_us, started - irq);
    else diag_inc(d.level_services);
    init_meta(lsm.header, lsm.meta, id, started, irq, irq_valid);
    if (s.needs_recovery) { recover_lsm(sensor, id, ACQ_IO_ERROR); return; }
    uint16_t before = 0;
    uint8_t status = 0;
    if (!lsm_status(sensor, before, status)) {
        diag_inc(d.lsm_io_errors); recover_lsm(sensor, id, ACQ_IO_ERROR); return;
    }
    lsm.meta.fifo_entries_before = before;
    if (before < d.min_entries) d.min_entries = before;
    diagnostic_max(d.max_entries, before);
    // OVR_IA bit 6 and OVR_LATCHED bit 3. Reading status can clear the latch.
    if (s.probed_overrun || (status & 0x48) || before > 512) {
        diag_inc(d.fifo_overruns);
        diag_add(d.discarded_words, before);
        recover_lsm(sensor, id, ACQ_OVERRUN); return;
    }
    if (before == 0) {
        // A successful, empty FIFO status read is already a safe occupancy
        // observation: retain it so SD may progress during the service cooldown.
        // Use service-start time conservatively (earlier than the SPI read).
        s.sd_snapshot_us = started;
        s.sd_snapshot_entries = 0;
        s.sd_cache_known = true;
        s.sd_cache_safe = true;
        diag_inc(d.empty_services);
        diag_inc(d.skipped_services);
        --s.sequence;
        return; // No packet attempt, no sequence gap.
    }
    if (before < 300) {
        lsm.meta.flags &= ~ACQ_IRQ_VALID;
        lsm.meta.irq_timestamp_us = 0;
    }
    const uint16_t count = before < LSM_MAX_FIFO_RECORDS ? before : LSM_MAX_FIFO_RECORDS;
    uint32_t acc_count = 0, gyro_count = 0;
    for (uint16_t i = 0; i < count; ++i) {
        if (sensor->IO_Read(lsm.records[i], 0x78, 7) != LSM6DSO32_OK) {
            diag_inc(d.lsm_io_errors);
            diag_add(d.discarded_words, before);
            recover_lsm(sensor, id, ACQ_IO_ERROR); return;
        }
        const uint8_t tag = (lsm.records[i][0] >> 3) & 0x1f;
        if (tag == 1) ++gyro_count;
        else if (tag == 2) ++acc_count;
        else {
            diag_inc(d.unknown_tags);
            diag_add(d.discarded_words, before);
            recover_lsm(sensor, id, ACQ_BAD_TAG); return;
        }
    }
    // A long SPI operation must not hide an overrun occurring mid-read.
    uint16_t after = 0;
    const uint32_t after_status_started_us = micros();
    if (!lsm_status(sensor, after, status)) {
        diag_inc(d.lsm_io_errors);
        diag_add(d.discarded_words, count);
        recover_lsm(sensor, id, ACQ_IO_ERROR); return;
    }
    if ((status & 0x48) || after > 512) {
        diag_inc(d.fifo_overruns);
        diag_add(d.discarded_words, count + after);
        recover_lsm(sensor, id, ACQ_OVERRUN); return;
    }
    // This status read is ALREADY required to detect mid-read overrun.
    // Reuse its observed occupancy for the SD admission prediction.  The
    // Reuse the status read needed for overrun detection as an SD headroom
    // estimate. Starting the clock before the SPI transfer is conservative.
    s.sd_snapshot_us = after_status_started_us;
    s.sd_snapshot_entries = after;
    s.sd_cache_known = true;
    s.sd_cache_safe = true;
    uint8_t temp[2];
    if (sensor->IO_Read(temp, 0x20, 2) == LSM6DSO32_OK) {
        lsm.meta.temperature_raw = (int16_t)((uint16_t)temp[0] | ((uint16_t)temp[1] << 8));
    } else {
        diag_inc(d.temp_read_errors);
        lsm.meta.flags |= ACQ_TEMP_INVALID;
    }
    lsm.meta.record_count = count;
    lsm.header.payload_len += count * 7U;
    diag_add(d.records_read, count);
    // Advance even on ring overflow; the next packet then exposes the gap.
    s.acc_index += acc_count;
    s.gyro_index += gyro_count;
    if (ring_buffer_push((const uint8_t*)&lsm, sizeof(CHUNK_HEADER) + lsm.header.payload_len))
        diag_add(d.records_queued, count);
    // No bypass/reset here. Unread words and arrivals remain in the FIFO.
}
