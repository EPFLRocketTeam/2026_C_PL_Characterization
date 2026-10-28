#include "acquisition_scheduler.h"
#include "accelerometer.h"
#include "sd_card.h"
#include "main.h"

extern ADXL371class adxl_main, adxl_sat;
extern LSM6DSO32Sensor lsm_main, lsm_sat;
extern bool has_adxl_main, has_adxl_sat, has_lsm_main, has_lsm_sat;
extern volatile bool pfm_triggered;

namespace {
struct LsmLane {
    LSM6DSO32Sensor *sensor;
    volatile bool *pending;
    volatile uint32_t *irq_time;
    bool *enabled;
    uint8_t id, pin;
    bool serviced = false;
    uint32_t last_service = 0;
    uint32_t retry_at_us = 0;
};
LsmLane lanes[] = {
    {&lsm_main, &lsm_main_int, &lsm_main_timestamp, &has_lsm_main, ID_LSM_MAIN, INT_PIN_LSM_Main},
    {&lsm_sat, &lsm_sat_int, &lsm_sat_timestamp, &has_lsm_sat, ID_LSM_SAT, INT_PIN_LSM_Sat}
};
uint8_t next_optional = 0;

void service_adxls() {
    uint32_t timestamp = 0;
    if (has_adxl_main && take_sensor_interrupt(adxl_main_int, adxl_main_timestamp, timestamp))
        log_adxl371_fifo(&adxl_main, timestamp, ID_ADXL371_MAIN, true);
    if (has_adxl_sat && take_sensor_interrupt(adxl_sat_int, adxl_sat_timestamp, timestamp))
        log_adxl371_fifo(&adxl_sat, timestamp, ID_ADXL371_SAT, true);
}

bool lsm_pending(const LsmLane &lane) {
    return *lane.pending || digitalRead(lane.pin) == HIGH || lsm_recovery_pending(lane.id);
}

uint8_t pending_mask() {
    uint8_t mask = 0;
    if (has_adxl_main && adxl_main_int) mask |= 1;
    if (has_adxl_sat && adxl_sat_int) mask |= 2;
    for (uint8_t i = 0; i < 2; ++i)
        if (*lanes[i].enabled && lsm_pending(lanes[i])) mask |= 4U << i;
    return mask;
}

void service_lsm(LsmLane &lane) {
    uint32_t timestamp = 0;
    // Consume a pending IRQ even for a proactive service. Otherwise it becomes
    // a stale request after the drain. New IRQs during the service remain pending.
    const bool irq = take_sensor_interrupt(*lane.pending, *lane.irq_time, timestamp);
    log_lsm_fifo(lane.sensor, timestamp, lane.id, irq);
    lane.last_service = micros();
    lane.retry_at_us = lane.last_service + LSM_PROACTIVE_INTERVAL_US;
    lane.serviced = true;
}
}

void service_acquisition_pass() {
    service_adxls();
    for (auto &lane : lanes) {
        if (pfm_triggered) return;
        service_adxls();
        if (*lane.enabled && lsm_pending(lane)) service_lsm(lane);
        service_adxls();
    }
}

void service_sd_if_ready() {
    if (pfm_triggered || !sd_write_due()) return;
    // Card busy: no optional SPI status probes or FIFO drains.
    if (!sd_ready_for_write()) return;
    service_adxls();
    if (pfm_triggered || pending_mask()) return; // urgent LSMs go through normal service

    for (uint8_t offset = 0; offset < 2; ++offset) {
        const uint8_t index = (next_optional + offset) % 2;
        auto &lane = lanes[index];
        if (!*lane.enabled) continue;
        // Check the cheap cache first; it never touches SPI.
        if (lsm_cached_headroom_valid(lane.id)) continue;
        const uint32_t now = micros();
        // During an unexpired cooldown, do no additional SPI register probes.
        // A signed delta is wrap-safe because the deadline is <2^31 us away.
        if (lane.serviced && int32_t(now - lane.retry_at_us) < 0) {
            diag_inc(acquisition_diag.sd_headroom_cooldown_skips);
            return;
        }
        // After expiry, perform one *fresh* FIFO occupancy read before
        // deciding whether a drain is truly required.
        if (lsm_cached_safe_for_sd(lane.sensor, lane.id)) continue;
        diag_inc(acquisition_diag.sd_headroom_deferrals);
        service_adxls();
        if (pfm_triggered || pending_mask()) return;
        diag_inc(acquisition_diag.sd_headroom_drains);
        service_lsm(lane);
        service_adxls();
        next_optional = index ^ 1U;
        // At most one optional drain per pass. Revisit both headroom estimates
        // on a later pass; do not write using a pre-drain snapshot of the other LSM.
        return;
    }
    service_adxls();
    if (pfm_triggered || pending_mask()) return;
    // ADXL servicing may have consumed headroom since the preceding checks.
    // These are arithmetic-only checks: neither SPI reads nor drains here.
    for (auto &lane : lanes)
        if (*lane.enabled && !lsm_cached_headroom_valid(lane.id)) return;
    write_buffer_to_sd(false, pending_mask()); // also checks card busy again
}
