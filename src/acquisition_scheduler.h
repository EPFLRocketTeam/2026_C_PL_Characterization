#pragma once

// Each call is bounded; no waiting for an IRQ, FIFO, or card to become ready.
void service_acquisition_pass();
void service_sd_if_ready();
