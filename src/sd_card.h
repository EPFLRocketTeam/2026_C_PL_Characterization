#pragma once

#include <SdFat.h>
#include "bench_config.h"

#include "packets.h"
#include "debug.h"

constexpr uint64_t FILE_SIZE_GB =  20;
constexpr uint64_t FILE_SIZE = FILE_SIZE_GB * 1024ULL * 1024ULL * 1024ULL;
constexpr uint64_t MAX_SAFE_BYTES = FILE_SIZE - RING_BUFFER_SIZE - 4096ULL; // Reserve a queued ring and final metadata

enum END_STATES {
    JUMPER_END,
    PFM_END
};

bool setup_sd();
bool setup_file();
bool close_sd(enum END_STATES end);
constexpr uint32_t SD_BLOCK_BYTES = ACQ_SD_BLOCK_BYTES;
constexpr uint32_t SD_MAX_DEFER_US = 25000;
constexpr uint32_t SD_PRESSURE_BYTES = RING_BUFFER_SIZE * 3U / 4U;
// Normal: one aligned block. Force: one block or final partial tail.
bool sd_write_due();
bool sd_ready_for_write();
bool sd_at_pressure();
void write_buffer_to_sd(bool force = false, uint8_t pending_mask = 0);
bool reached_file_end();
