#pragma once

#include <SdFat.h>

#include "packets.h"
#include "debug.h"

constexpr uint64_t FILE_SIZE_GB =  3;
constexpr uint64_t FILE_SIZE = FILE_SIZE_GB * 1024ULL * 1024ULL * 1024ULL;
constexpr uint64_t MAX_SAFE_BYTES = FILE_SIZE - 1024ULL; // 1KB before EOF

enum END_STATES {
    JUMPER_END,
    PFM_END
};

bool setup_sd();
bool setup_file();
bool close_sd(enum END_STATES end);
void write_buffer_to_sd();
bool reached_file_end();