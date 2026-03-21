#!/usr/bin/env bash
set -euo pipefail

BUILD_KIND="${1:-static}"

rm -rf build_windows_mingw
mkdir -p build_windows_mingw

cmake -B build_windows_mingw \
    -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_mingw.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_KIND="$BUILD_KIND"

cmake --build build_windows_mingw -- -j"$(nproc)"
