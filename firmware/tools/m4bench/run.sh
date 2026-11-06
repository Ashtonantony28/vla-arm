#!/bin/sh
# Build the core + sim for Cortex-M4F and count instructions per control step under QEMU.
# usage: tools/m4bench/run.sh [build-dir]     (needs arm-none-eabi-gcc and qemu-system-arm)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
FW=$(cd "$HERE/../.." && pwd)
OUT=${1:-$FW/build-m4bench}
mkdir -p "$OUT"
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -std=gnu11 \
  -Wall -Wextra -Werror -fno-math-errno -ffunction-sections -fdata-sections \
  -I"$FW/core/include" -I"$FW/sim" \
  "$HERE/bench.c" "$HERE/startup_mps2.c" "$FW"/core/src/*.c "$FW"/sim/*.c \
  -nostartfiles --specs=nano.specs --specs=nosys.specs -T"$HERE/mps2.ld" -Wl,--gc-sections \
  -o "$OUT/m4bench.elf"
timeout 300 qemu-system-arm -M mps2-an386 -nographic -monitor none -serial none \
  -semihosting-config enable=on,target=native -icount shift=6 -kernel "$OUT/m4bench.elf"
