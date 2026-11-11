#!/bin/sh
# Re-create the trimmed third-party sources used by the STM32G474 target build.
# Only the files the build needs are kept (plus each project's licence); versions are pinned.
#   firmware/third_party/fetch.sh            (needs git and network access to github.com)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

TINYUSB_TAG=0.21.0            # commit dae3f9a366bfcddbf9dcf1b48d7500286a849539, MIT
CMSIS_DEVICE_G4_TAG=v1.2.6    # commit 25664ddc3a7624ae9627ae8c4c672073dc5b2539, Apache-2.0
CMSIS_5_TAG=5.9.0             # commit 2b7495b8535bdcb306dac29b9ded4cfb679d7e5c, Apache-2.0

git clone -q --depth 1 --branch "$TINYUSB_TAG" https://github.com/hathach/tinyusb "$TMP/tinyusb"
git clone -q --depth 1 --branch "$CMSIS_DEVICE_G4_TAG" https://github.com/STMicroelectronics/cmsis_device_g4 "$TMP/cmsis_device_g4"
git clone -q --depth 1 --filter=blob:none --sparse --branch "$CMSIS_5_TAG" https://github.com/ARM-software/CMSIS_5 "$TMP/cmsis_5"
git -C "$TMP/cmsis_5" sparse-checkout set CMSIS/Core/Include

rm -rf "$HERE/tinyusb" "$HERE/cmsis_device_g4" "$HERE/cmsis_5"

# TinyUSB: core, device stack, CDC class, the STM32 FSDEV device driver
T="$HERE/tinyusb"
mkdir -p "$T/src/class/cdc" "$T/src/portable/st/stm32_fsdev"
cp "$TMP/tinyusb/LICENSE" "$T/"
cp "$TMP/tinyusb/src/tusb.c" "$TMP/tinyusb/src/tusb.h" "$TMP/tinyusb/src/tusb_option.h" "$T/src/"
cp -r "$TMP/tinyusb/src/common" "$TMP/tinyusb/src/device" "$TMP/tinyusb/src/osal" "$T/src/"
cp "$TMP/tinyusb/src/class/cdc/cdc.h" "$TMP/tinyusb/src/class/cdc/cdc_device.c" \
   "$TMP/tinyusb/src/class/cdc/cdc_device.h" "$T/src/class/cdc/"
for f in dcd_stm32_fsdev.c fsdev_common.c fsdev_common.h fsdev_stm32.h; do
  cp "$TMP/tinyusb/src/portable/st/stm32_fsdev/$f" "$T/src/portable/st/stm32_fsdev/"
done

# ST CMSIS device header for the STM32G474
D="$HERE/cmsis_device_g4"
mkdir -p "$D/Include"
cp "$TMP/cmsis_device_g4/LICENSE.md" "$D/"
for f in stm32g4xx.h stm32g474xx.h system_stm32g4xx.h; do cp "$TMP/cmsis_device_g4/Include/$f" "$D/Include/"; done

# Arm CMSIS-Core (Cortex-M4)
C="$HERE/cmsis_5"
mkdir -p "$C/CMSIS/Core/Include"
cp "$TMP/cmsis_5/LICENSE.txt" "$C/"
for f in core_cm4.h cmsis_compiler.h cmsis_gcc.h cmsis_version.h mpu_armv7.h; do
  cp "$TMP/cmsis_5/CMSIS/Core/Include/$f" "$C/CMSIS/Core/Include/"
done
echo "third_party refreshed: tinyusb $TINYUSB_TAG, cmsis_device_g4 $CMSIS_DEVICE_G4_TAG, CMSIS_5 $CMSIS_5_TAG"
