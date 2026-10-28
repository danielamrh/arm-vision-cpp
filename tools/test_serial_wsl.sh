#!/usr/bin/env bash
# Builds and runs the POSIX SerialPort tests in WSL, so the Linux code can be developed on the
# Windows PC without the Raspberry Pi. Only serial_port_posix.cpp and its tests are compiled
# (no std::format, no OpenCV), so an older WSL g++ (11+) is enough.
#
# Usage (from PowerShell in the repo root):   wsl bash tools/test_serial_wsl.sh [gtest filter]
# GoogleTest sources are taken from a Windows build tree (build/*/_deps/googletest-src),
# i.e. configure the project on Windows once first.
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
filter="${1:-SerialPortPosix.*}"

gtest_dir="$(ls -d "$repo"/build/*/_deps/googletest-src/googletest 2>/dev/null | head -1 || true)"
if [[ -z "$gtest_dir" ]]; then
    echo "GoogleTest sources not found. Configure the project on Windows once (cmake --preset ...)." >&2
    exit 1
fi

out=/tmp/armvision-serial-wsl
mkdir -p "$out"
if [[ ! -f "$out/gtest.o" ]]; then
    echo "Building GoogleTest once (takes a minute) ..."
    g++ -std=c++20 -O1 -I"$gtest_dir/include" -I"$gtest_dir" -c "$gtest_dir/src/gtest-all.cc" -o "$out/gtest.o"
    g++ -std=c++20 -O1 -I"$gtest_dir/include" -c "$gtest_dir/src/gtest_main.cc" -o "$out/gtest_main.o"
fi

g++ -std=c++20 -g -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion \
    -I"$repo/include" -I"$gtest_dir/include" \
    "$repo/src/serial_port_posix.cpp" "$repo/tests/test_serial_posix.cpp" \
    "$out/gtest.o" "$out/gtest_main.o" -pthread -o "$out/serial_tests"

"$out/serial_tests" --gtest_filter="$filter" --gtest_brief=1
