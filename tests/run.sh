#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_binary="$(mktemp)"
trap 'rm -f "$test_binary"' EXIT
g++ -DACQ_SD_BLOCK_BYTES="${ACQ_SD_BLOCK_BYTES:-512}" -std=c++14 -Wall -Wextra -Wno-unused-parameter -fsanitize=address,undefined -fno-omit-frame-pointer -g -Itests/stubs -Isrc src/*.cpp tests/test_firmware.cpp -o "$test_binary"
ASAN_OPTIONS=detect_leaks=0 "$test_binary" "${1:-/dev/null}"
