#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <string>
#define FASTRUN
#define DMAMEM
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define RISING 1
#define FALLING 2
#define HEX 16
#define BUILTIN_SDCARD 254
using byte = uint8_t;
extern uint32_t fake_mask, fake_time;
extern volatile uint32_t SCB_AIRCR;
inline uint32_t __get_PRIMASK() { return fake_mask; }
inline void __disable_irq() { fake_mask = 1; }
inline void __enable_irq() { fake_mask = 0; }
inline void __set_PRIMASK(uint32_t v) { fake_mask = v; }
inline uint32_t micros() { return ++fake_time; }
inline uint32_t millis() { return fake_time / 1000; }
inline void delay(uint32_t ms) { fake_time += ms * 1000; }
inline void digitalWrite(int,int) {}
inline int digitalRead(int) { return 0; }
inline void pinMode(int,int) {}
inline int digitalPinToInterrupt(int p) { return p; }
inline void attachInterrupt(int,void(*)(),int) {}
inline void detachInterrupt(int) {}
struct FakeSerial {
    void begin(int) {}
    explicit operator bool() const { return true; }
    template<class T> void print(T) {}
    template<class T> void print(T,int) {}
    template<class T> void println(T) {}
    template<class T> void println(T,int) {}
    void println() {}
};
extern FakeSerial Serial;
