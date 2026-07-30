#pragma once

#include <math.h>

#include "ADXL372.h"

#include "packets.h"
#include "debug.h"

#define INT_PIN_Accel_Main 2
#define INT_PIN_Accel_Sat_1 28
#define INT_PIN_Accel_Sat_2 17

extern volatile bool accel_main_int;
extern volatile bool accel_sat_1_int;
extern volatile bool accel_sat_2_int;

extern volatile uint32_t accel_main_timestamp;
extern volatile uint32_t accel_sat_1_timestamp;
extern volatile uint32_t accel_sat_2_timestamp;

FASTRUN void accel_main_ISR();
FASTRUN void accel_sat_1_ISR();
FASTRUN void accel_sat_2_ISR();

bool setup_adxl372(ADXL372class *accel);
void start_adxl372(ADXL372class *accel, uint8_t interrupt_pin, void (*isr)());
void print_adxl372_accel(ADXL372class *accel);
void log_adxl372_fifo(ADXL372class *accel, uint32_t timestamp, uint8_t sensor_id);
void print_adxl372_diagnostics();