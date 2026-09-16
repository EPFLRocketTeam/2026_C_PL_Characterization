#pragma once

#include <math.h>

#include "ADXL371.h"
#include "LSM6DSO32Sensor.h"

#include "packets.h"
#include "debug.h"

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
void print_adxl371_accel(ADXL371class *accel);
void log_adxl371_fifo(ADXL371class *accel, uint32_t timestamp, uint8_t sensor_id);
void print_adxl371_diagnostics();

//=================================================
// LSM6DOS32
//=================================================
bool setup_lsm(LSM6DSO32Sensor *accel);
void start_lsm(LSM6DSO32Sensor *accel, uint8_t interrupt_pin, void (*isr)());
int16_t gatRawTmp(LSM6DSO32Sensor *accel);
void print_lsm_accel(LSM6DSO32Sensor *accel);
void print_lsm_gyro(LSM6DSO32Sensor *accel);
void print_lsm_temperature(LSM6DSO32Sensor *accel);
void log_lsm_data(LSM6DSO32Sensor *accel, uint32_t timestamp, uint8_t sensor_id);
void print_lsm_diagnostics(LSM6DSO32Sensor *accel);