#pragma once

#include <Audio.h>

#include "packets.h"

#define AUDIO_MEMORY_BLOCKS 128

bool setup_microphones();
void read_microphone();