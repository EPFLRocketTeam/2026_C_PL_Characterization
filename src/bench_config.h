#pragma once
#include <stdint.h>

// Default next test: FIFO SDIO, one sector per main-loop pass.
// Set to 4096 (or -DACQ_SD_BLOCK_BYTES=4096) for the scheduling control.
#ifndef ACQ_SD_BLOCK_BYTES
#define ACQ_SD_BLOCK_BYTES 512
#endif
static_assert(ACQ_SD_BLOCK_BYTES == 512 || ACQ_SD_BLOCK_BYTES == 4096,
              "Supported test sizes: 512 or 4096");
constexpr uint32_t LSM_PROACTIVE_INTERVAL_US = 10000;
// The admission budget models FIFO growth; not a guaranteed SD stall bound.
constexpr uint32_t SD_ADMISSION_BUDGET_US = 5000;
constexpr uint32_t DIAGNOSTIC_PERIOD_MS = 5000;
#ifndef ACQ_BENCH_TRACES
#define ACQ_BENCH_TRACES 1
#endif
