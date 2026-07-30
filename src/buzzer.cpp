#include "buzzer.h"

void setup_buzzer() {
    digitalWrite(BUZZER_PIN, LOW);
    pinMode(BUZZER_PIN, OUTPUT);
}

void beep(uint32_t delay_ms) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(delay_ms);
    digitalWrite(BUZZER_PIN, LOW);
}

void beeps(uint32_t delay_ms, uint16_t amount) {
    for (uint16_t i = 0; i < amount; i++) {
        beep(delay_ms);
        delay(delay_ms);
    }
}

void conditional_beeps(bool condition, uint32_t true_delay_ms, uint16_t true_amount, uint32_t false_delay_ms, uint16_t false_amount) {
    if (condition) {
        beeps(true_delay_ms, true_amount);
    } else {
        beeps(false_delay_ms, false_amount);
    }
}