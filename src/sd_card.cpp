#include "sd_card.h"

SdFs sd;
FsFile file;

DMAMEM uint8_t ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t rb_head = 0;
volatile uint32_t rb_tail = 0;

uint64_t total_bytes_written = 0;

bool setup_sd() {
    if(!sd.begin(BUILTIN_SDCARD)){
        #ifdef DEBUG_
        sd.initErrorPrint();
        #endif
        return false;
    }

    return true;
}

bool setup_file() {
    uint8_t idx = 0;
    char file_name[16];
    do {
        snprintf(file_name, sizeof(file_name), "data_%04u.bin", idx++);
    } while(sd.exists(file_name));

    if(!file.open(file_name, O_WRITE | O_CREAT | O_TRUNC)){
        #ifdef DEBUG_
        Serial.println("File opening failed");
        #endif
        file.close();
        return false;
    }

    if (!file.preAllocate(FILE_SIZE)) {
        #ifdef DEBUG_
        Serial.println("File pre allocation failed");
        #endif
        file.close();
        return false;
    }

    return true;
}

bool close_sd(enum END_STATES end) {
    // --- INJECT TERMINATION PACKET ---
    CHUNK_HEADER end_header;
    end_header.sync_word = 0xAAAA;
    if (end == PFM_END) {
        end_header.sensor_type = PFM;
    } else {
        end_header.sensor_type = EOF_;
    }
    end_header.timestamp = micros();
    end_header.payload_len = 0;
    
    file.write((const uint8_t*)&end_header, sizeof(CHUNK_HEADER));

    // Safely close file
    if (end != PFM_END) file.truncate();
    file.close();
    return true;
}

void write_buffer_to_sd() {
    uint32_t current_head = rb_head;
    if (current_head != rb_tail) {
        uint32_t loop_bytes_written = 0;

        if (current_head > rb_tail) {
            uint32_t bytes_to_write = current_head - rb_tail;
            file.write(&ring_buffer[rb_tail], bytes_to_write);
            loop_bytes_written += bytes_to_write;
            rb_tail = current_head;
        } else {
            uint32_t bytes_to_end = RING_BUFFER_SIZE - rb_tail;
            file.write(&ring_buffer[rb_tail], bytes_to_end);
            loop_bytes_written += bytes_to_end;
            if (current_head > 0) {
                file.write(&ring_buffer[0], current_head);
                loop_bytes_written += current_head;
            }
            rb_tail = current_head;
        }

        // Track total throughput
        total_bytes_written += loop_bytes_written;
    }
}

bool reached_file_end() {
    return (total_bytes_written >= MAX_SAFE_BYTES);
}