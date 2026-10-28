#pragma once
#include "Arduino.h"
struct AudioInputI2S {};
struct AudioRecordQueue {
    void begin() {}
    int available() { return 0; }
    int16_t *readBuffer() { static int16_t b[128]; return b; }
    void freeBuffer() {}
};
struct AudioConnection { template<class T> AudioConnection(T&,int,AudioRecordQueue&,int) {} };
inline void AudioMemory(int) {}
