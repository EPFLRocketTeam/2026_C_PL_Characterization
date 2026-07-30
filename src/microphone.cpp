#include "microphone.h"

AudioInputPDM pdm;
AudioRecordQueue queue;
AudioConnection patchCord(pdm, 0, queue, 0);

Mic_PACKET mic_packet;

bool setup_microphones() {
    AudioMemory(AUDIO_MEMORY_BLOCKS);

    // Setup static header info
    mic_packet.header.sync_word = 0xAAAA;
    mic_packet.header.sensor_type = ID_MIC;
    mic_packet.header.payload_len = sizeof(mic_packet.audio_samples);

    queue.begin();

    return true;
}

void read_microphone() {
    while (queue.available() > 0) {
        int16_t *buf = queue.readBuffer();

        mic_packet.header.timestamp = micros(); // Tag the block
        memcpy(mic_packet.audio_samples, buf, 256); // 128 samples * 2 bytes

        queue.freeBuffer();

        ring_buffer_push((uint8_t*)&mic_packet, sizeof(mic_packet));
    }
}