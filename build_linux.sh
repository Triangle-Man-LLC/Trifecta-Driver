#!/usr/bin/env bash
set -euo pipefail

GENERATOR="Unix Makefiles"
command -v ninja >/dev/null && GENERATOR="Ninja"

rm -rf build_linux
mkdir -p build_linux

cmake -B build_linux -G "$GENERATOR" -DCMAKE_BUILD_TYPE=Release
cmake --build build_linux -- -j$(nproc)
