#pragma once
#include "Arduino.h"
#include <vector>
#include <algorithm>
#define O_WRITE 1
#define O_CREAT 2
#define O_TRUNC 4
constexpr int FIFO_SDIO = 1;
struct SdioConfig { explicit SdioConfig(int) {} };
struct SdFs {
    bool begin(SdioConfig) { return true; }
    bool begin(int) { return true; }
    void initErrorPrint() {}
    bool exists(const char*) { return false; }
};
struct FsFile {
    bool busy = false;
    bool isBusy() { return busy; }
    size_t max_write = SIZE_MAX;
    uint32_t write_delay_us = 0;
    std::vector<uint8_t> output;
    bool open(const char*,int) { return true; }
    bool preAllocate(uint64_t) { return true; }
    bool truncate() { return true; }
    bool close() { return true; }
    size_t write(const uint8_t *b, size_t n) {
        fake_time += write_delay_us;
        n = std::min(n, max_write);
        output.insert(output.end(), b, b+n);
        return n;
    }
};
