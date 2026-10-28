#include "sd_card.h"

SdFs sd;
FsFile file;

DMAMEM uint8_t ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t rb_head = 0;
volatile uint32_t rb_tail = 0;

uint64_t total_bytes_written = 0;
namespace {
alignas(32) uint8_t sd_stage[SD_BLOCK_BYTES];
bool sd_deferred = false;
uint32_t sd_defer_started = 0;
uint32_t queued_bytes() { return (RING_BUFFER_SIZE + rb_head - rb_tail) % RING_BUFFER_SIZE; }
uint32_t next_write_size() { return SD_BLOCK_BYTES - (uint32_t)(total_bytes_written % SD_BLOCK_BYTES); }
}

bool setup_sd() {
    if(!sd.begin(SdioConfig(FIFO_SDIO))){
        #if DEBUG_
        sd.initErrorPrint();
        #endif
        return false;
    }

    return true;
}

bool setup_file() {
    uint32_t idx = 0;
    char file_name[24];
    do {
        snprintf(file_name, sizeof(file_name), "data_%04u.bin", idx++);
    } while(sd.exists(file_name));

    if(!file.open(file_name, O_WRITE | O_CREAT | O_TRUNC)){
        #if DEBUG_
        Serial.println("File opening failed");
        #endif
        file.close();
        return false;
    }

    if (!file.preAllocate(FILE_SIZE)) {
        #if DEBUG_
        Serial.println("File pre allocation failed");
        #endif
        file.close();
        return false;
    }

    total_bytes_written = 0;
    sd_deferred = false;
    return true;
}

bool close_sd(enum END_STATES end) {
    // Producers have stopped. Drain whole blocks and the final partial tail.
    // Bounded even if the device repeatedly reports tiny positive writes.
    // One attempt per sector/block, plus slack for partial writes. A fixed
    // 256-pass limit would prematurely abort a near-full 350 KiB ring when
    // using 512-byte sector submissions.
    constexpr uint32_t REQUIRED_CLOSE_ATTEMPTS =
        (RING_BUFFER_SIZE + SD_BLOCK_BYTES - 1U) / SD_BLOCK_BYTES + 16U;
    constexpr uint32_t MAX_CLOSE_ATTEMPTS =
        REQUIRED_CLOSE_ATTEMPTS > 256U ? REQUIRED_CLOSE_ATTEMPTS : 256U;
    for (uint32_t attempt = 0; rb_head != rb_tail && attempt < MAX_CLOSE_ATTEMPTS; ++attempt) {
        const uint32_t old_tail = rb_tail;
        write_buffer_to_sd(true);
        if (rb_tail == old_tail) break;
    }
    if (rb_head != rb_tail) {
        diag_inc(acquisition_diag.sd_close_failures);
        file.close();
        return false; // Never append a clean terminator after unwritten data.
    }
    // --- INJECT TERMINATION PACKET ---
    CHUNK_HEADER end_header;
    end_header.sync_word = 0xAA;
    if (end == PFM_END) {
        end_header.sensor_type = PFM;
    } else {
        end_header.sensor_type = EOF_;
    }
    end_header.timestamp = micros();
    end_header.payload_len = 0;
    
    bool ok = file.write((const uint8_t*)&end_header, sizeof(CHUNK_HEADER)) == sizeof(CHUNK_HEADER);

    // Safely close file
    if (end != PFM_END && !file.truncate()) ok = false;
    if (!file.close()) ok = false;
    if (!ok) diag_inc(acquisition_diag.sd_close_failures);
    return ok;
}

bool sd_ready_for_write() {
    if (!file.isBusy()) return true;
    diag_inc(acquisition_diag.sd_busy_skips);
    return false;
}

bool sd_write_due() { return queued_bytes() >= next_write_size(); }
bool sd_at_pressure() { return queued_bytes() >= SD_PRESSURE_BYTES; }

void write_buffer_to_sd(bool force, uint8_t pending_mask) {
    const uint32_t used = queued_bytes();
    const uint32_t block = next_write_size();
    if (!used || (!force && used < block)) return;
    if (!force && !sd_ready_for_write()) return;
    const uint32_t now = micros();
    if (!force && pending_mask) {
        if (!sd_deferred) { sd_deferred = true; sd_defer_started = now; }
        const uint32_t waited = now - sd_defer_started; // wrap-safe
        diagnostic_max(acquisition_diag.sd_max_defer_us, waited);
        diag_inc(acquisition_diag.sd_deferred_calls);
        return; // Normal acquisition never forces a write ahead of pending sensors.
    }
    if (sd_deferred) diagnostic_max(acquisition_diag.sd_max_defer_us, now - sd_defer_started);
    sd_deferred = false;
    const uint32_t requested = used < block ? used : block;
    const uint32_t tail = rb_tail;
    const uint32_t first = requested < RING_BUFFER_SIZE-tail ? requested : RING_BUFFER_SIZE-tail;
    memcpy(sd_stage, ring_buffer+tail, first);
    if (requested > first) memcpy(sd_stage+first, ring_buffer, requested-first);
    const uint32_t started = micros();
    const size_t written = file.write(sd_stage, requested);
    const uint32_t elapsed = micros() - started;
    diag_inc(acquisition_diag.sd_write_calls);
    diag_add(acquisition_diag.sd_requested_bytes, requested);
    diagnostic_max(acquisition_diag.sd_max_write_us, elapsed);
    if (!force) {
        diag_inc(acquisition_diag.sd_normal_write_calls);
        diagnostic_max(acquisition_diag.sd_normal_max_write_us, elapsed);
        if (elapsed > SD_ADMISSION_BUDGET_US) diag_inc(acquisition_diag.sd_normal_budget_exceeded);
    }
    trace_sd_write(started, elapsed, requested, (uint32_t)written, used,
                   pending_mask, (force ? 1 : 0));
    if (written > requested) {
        diag_inc(acquisition_diag.sd_short_writes);
        return;
    }
    rb_tail = (tail + (uint32_t)written) % RING_BUFFER_SIZE;
    total_bytes_written += written;
    if (written != requested) diag_inc(acquisition_diag.sd_short_writes);
    // Never retry here; the next main-loop pass services sensors first.
}

bool reached_file_end() {
    return (total_bytes_written >= MAX_SAFE_BYTES);
}
