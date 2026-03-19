#!/usr/bin/env bash
set -euo pipefail

# Default: build static library
BUILD_KIND="static"

# Parse optional argument: static | shared
if [[ $# -ge 1 ]]; then
    case "$1" in
        static|shared)
            BUILD_KIND="$1"
            ;;
        *)
            echo "Usage: $0 [static|shared]"
            exit 1
            ;;
    esac
fi

echo "=== Building Trifecta-Driver for Linux ==="
echo "Library type: $BUILD_KIND"

GENERATOR="Unix Makefiles"
command -v ninja >/dev/null && GENERATOR="Ninja"

rm -rf build_linux
mkdir -p build_linux

cmake -B build_linux \
    -G "$GENERATOR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_KIND="$BUILD_KIND"

cmake --build build_linux -- -j"$(nproc)"
