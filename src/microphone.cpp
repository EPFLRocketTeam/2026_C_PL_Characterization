#include "microphone.h"

AudioInputI2S i2s;
AudioRecordQueue queueLeft;  // Canal 0
AudioRecordQueue queueRight; // Canal 1

AudioConnection patchCord0(i2s, 0, queueLeft, 0);  // MK 1
AudioConnection patchCord1(i2s, 1, queueRight, 0); // MK 2

Mic_PACKET mic_packet;

bool setup_microphones() {
    AudioMemory(AUDIO_MEMORY_BLOCKS);

    // Setup static header info
    mic_packet.header.sync_word = 0xAA;
    mic_packet.header.sensor_type = ID_MIC;
    mic_packet.header.payload_len = sizeof(mic_packet.left_samples) + sizeof(mic_packet.right_samples);

    queueLeft.begin();
    queueRight.begin();

    return true;
}

void read_microphone() {
    while (queueLeft.available() > 0 && queueRight.available() > 0) {
        int16_t *bufL = queueLeft.readBuffer();
        int16_t *bufR = queueRight.readBuffer();

        mic_packet.header.timestamp = micros(); // Tag the block
        memcpy(mic_packet.left_samples, bufL, 256);  // 128 samples * 2 bytes
        memcpy(mic_packet.right_samples, bufR, 256); 

        queueLeft.freeBuffer();
        queueRight.freeBuffer();

        ring_buffer_push((uint8_t*)&mic_packet, sizeof(mic_packet));
    }
}