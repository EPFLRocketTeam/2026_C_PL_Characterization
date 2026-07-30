#pragma once

#include "Arduino.h"

#define BUZZER_PIN 6

void setup_buzzer();
void beep(uint32_t delay_ms);
void beeps(uint32_t delay_ms, uint16_t amount);
void conditional_beeps(bool condition, uint32_t true_delay_ms, uint16_t true_amount, uint32_t false_delay_ms, uint16_t false_amount);