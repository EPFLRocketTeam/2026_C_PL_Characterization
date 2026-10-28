#pragma once

#include <math.h>

#include "ADXL371.h"
#include "LSM6DSO32Sensor.h"

#include "packets.h"
#include "debug.h"
#include "bench_config.h"

#define INT_PIN_ADXL_Main 32
#define INT_PIN_ADXL_Sat 18

#define INT_PIN_LSM_Main 9
#define INT_PIN_LSM_Sat 15

extern volatile bool adxl_main_int;
extern volatile bool adxl_sat_int;

extern volatile bool lsm_main_int;
extern volatile bool lsm_sat_int;

extern volatile uint32_t adxl_main_timestamp;
extern volatile uint32_t adxl_sat_timestamp;

extern volatile uint32_t lsm_main_timestamp;
extern volatile uint32_t lsm_sat_timestamp;

FASTRUN void adxl_main_ISR();
FASTRUN void adxl_sat_ISR();

FASTRUN void lsm_main_ISR();
FASTRUN void lsm_sat_ISR();

//=================================================
// ADXL371
//=================================================
bool setup_adxl371(ADXL371class *accel);
void start_adxl371(ADXL371class *accel, uint8_t interrupt_pin, void (*isr)());
void log_adxl371_fifo(ADXL371class *accel, uint32_t timestamp, uint8_t sensor_id, bool irq_valid = true);

//=================================================
// LSM6DOS32
//=================================================
bool setup_lsm(LSM6DSO32Sensor *accel);
void start_lsm(LSM6DSO32Sensor *accel, uint8_t interrupt_pin, void (*isr)());
void log_lsm_fifo(LSM6DSO32Sensor *accel, uint32_t timestamp, uint8_t sensor_id, bool irq_valid = true);

bool lsm_recovery_pending(uint8_t sensor_id);

void capture_lsm_registers(LSM6DSO32Sensor *sensor, uint8_t id);

// FIFO status sampled before SD admission; caller should proactively drain if false.
// Conservative capacity budget and configured maximum aggregate accel+gyro input.
constexpr uint16_t LSM_SD_SAFE_CAPACITY = 420;
constexpr uint32_t LSM_SD_INPUT_RATE_HZ = 13334;

bool lsm_safe_for_sd(LSM6DSO32Sensor *sensor, uint8_t id,
                     uint32_t budget_us = SD_ADMISSION_BUDGET_US);

// Cache ages at the configured aggregate FIFO rate; services invalidate it.
bool lsm_cached_safe_for_sd(LSM6DSO32Sensor *sensor, uint8_t id);
bool lsm_cached_headroom_valid(uint8_t id);
