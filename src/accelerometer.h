#pragma once

#include <math.h>

#include "ADXL371.h"
#include"Adafruit_LSM6DSO32.h"

#include "packets.h"
#include "debug.h"

#define INT_PIN_ADXL_Main 32
#define INT_PIN_ADXL_Sat 18

#define INT_PIN_LSM_Main 9
#define INT_PIN_LSM_Sat 15

extern volatile bool adxl_main_int;
extern volatile bool adxl_sat_int;

extern volatile bool alsm_main_int;
extern volatile bool alsm_sat_int;

extern volatile uint32_t adxl_main_timestamp;
extern volatile uint32_t adxl_sat_timestamp;

extern volatile uint32_t lsm_main_timestamp;
extern volatile uint32_t lsm_sat_timestamp;

FASTRUN void adxl_main_ISR();
FASTRUN void adxl_sat_ISR();

FASTRUN void lsm_main_ISR();
FASTRUN void lsm_sat_ISR();

bool setup_adxl371(ADXL371class *accel);
void start_adxl371(ADXL371class *accel, uint8_t interrupt_pin, void (*isr)());
void print_adxl371_accel(ADXL371class *accel);
void log_adxl371_fifo(ADXL371class *accel, uint32_t timestamp, uint8_t sensor_id);
void print_adxl371_diagnostics();

bool setup_adxl371(Adafruit_LSM6DSO32 *accel);
void start_adxl371(Adafruit_LSM6DSO32 *accel, uint8_t interrupt_pin, void (*isr)());
void print_adxl371_accel(Adafruit_LSM6DSO32 *accel);