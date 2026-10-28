#pragma once
#include "SPI.h"
#include <array>
#include <deque>
#include <vector>
#include <functional>
#define LSM6DSO32_OK 0
class LSM6DSO32Sensor {
public:
    std::deque<std::array<uint8_t,7>> fifo;
    std::vector<uint8_t> mode_writes;
    std::function<void(uint8_t)> on_read;
    uint32_t status_reads = 0;
    uint8_t status_flags = 0;
    uint8_t regs[128] = {};
    int fail_register = -1;
    int fifo_reads = 0, overrun_after_reads = -1, fail_after_reads = -1;
    explicit LSM6DSO32Sensor(SPIClass*,int,uint32_t) {}
    int begin() { return 0; }
    int IO_Read(uint8_t *b,uint8_t addr,uint16_t n) {
        if (on_read) on_read(addr);
        if (addr == 0x3a) ++status_reads;
        if (addr == fail_register) return -1;
        memcpy(b,regs+addr,n);
        if (addr == 0x3a) {
            b[0] = fifo.size() & 255;
            b[1] = ((fifo.size() >> 8) & 3) | status_flags;
        }
        if (addr == 0x78) {
            if (fifo_reads++ == fail_after_reads) return -1;
            if (fifo.empty()) return -1;
            memcpy(b,fifo.front().data(),n); fifo.pop_front();
            if (fifo_reads == overrun_after_reads) status_flags |= 0x48;
        }
        return 0;
    }
    int IO_Write(uint8_t*,uint8_t,uint16_t) { return 0; }
    int Write_Reg(uint8_t reg,uint8_t value) {
        if (reg == 0x0a) {
            mode_writes.push_back(value);
            if (value == 0) { fifo.clear(); status_flags = 0; }
        }
        return 0;
    }
    int Get_FIFO_Num_Samples(uint16_t *p) { *p = fifo.size(); return 0; }
    int Get_X_Axes(int32_t*) { return 0; }
    int Get_G_Axes(int32_t*) { return 0; }
    int ReadID(uint8_t*) { return 0; }
    int Get_X_ODR(float*) { return 0; }
    int Get_G_ODR(float*) { return 0; }
    int Get_X_FS(int32_t*) { return 0; }
    int Get_G_FS(int32_t*) { return 0; }
    int Get_FIFO_Full_Status(uint8_t*) { return 0; }
};
