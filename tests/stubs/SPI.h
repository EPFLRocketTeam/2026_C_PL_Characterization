#pragma once
#include "Arduino.h"
#include <deque>
#define MSBFIRST 1
#define SPI_MODE0 0
struct SPISettings { SPISettings() {} SPISettings(uint32_t,int,int) {} };
class SPIClass {
public:
    uint8_t regs[128] = {};
    uint32_t write_count = 0;
    std::deque<uint8_t> fifo;
    bool command = true, reading = false;
    uint8_t address = 0;
    void begin() {}
    void end() {}
    void beginTransaction(SPISettings) { command = true; }
    void endTransaction() {}
    uint8_t transfer(uint8_t v) {
        if (command) { address = v >> 1; reading = v & 1; command = false; return 0; }
        if (reading) {
            if (address == 0x42) {
                if (fifo.empty()) return 0;
                uint8_t out = fifo.front(); fifo.pop_front(); return out;
            }
            uint8_t a = address++;
            if (a == 6) return (fifo.size() / 2) >> 8;
            if (a == 7) return (fifo.size() / 2) & 255;
            return regs[a];
        }
        ++write_count;
        regs[address++] = v;
        return 0;
    }
};
extern SPIClass SPI, SPI1;
