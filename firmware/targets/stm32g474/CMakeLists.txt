# Cross build of the armfw core + STM32G474 target layer.
#   cmake -S targets/stm32g474 -B build-arm -DCMAKE_TOOLCHAIN_FILE=../../cmake/arm-none-eabi.cmake
#   cmake --build build-arm
# Options: -DARMFW_HOST_LINK_USB=OFF  host link on LPUART1 (NUCLEO ST-LINK VCP) instead of USB CDC
cmake_minimum_required(VERSION 3.16)
project(armfw_stm32g474 C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE MinSizeRel)
endif()
set(FW_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(TP ${FW_ROOT}/third_party)
option(ARMFW_HOST_LINK_USB "Host link over USB CDC (TinyUSB); OFF = LPUART1" ON)

set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(WARN -Wall -Wextra -Werror -Wdouble-promotion -Wshadow -fno-math-errno)

# Portable core: strict ISO C11, same as the host build.
add_library(armfw_core_m4 STATIC
  ${FW_ROOT}/core/src/crc16.c ${FW_ROOT}/core/src/proto.c ${FW_ROOT}/core/src/config.c
  ${FW_ROOT}/core/src/traj.c ${FW_ROOT}/core/src/safety.c ${FW_ROOT}/core/src/servo_bus.c
  ${FW_ROOT}/core/src/control.c ${FW_ROOT}/core/src/analog.c ${FW_ROOT}/core/src/calstore.c
  ${FW_ROOT}/core/src/host_link.c)
target_include_directories(armfw_core_m4 PUBLIC ${FW_ROOT}/core/include)
target_compile_options(armfw_core_m4 PRIVATE ${MCU_FLAGS} ${WARN} -pedantic -Wconversion -Wno-sign-conversion
                       -ffunction-sections -fdata-sections -fstack-usage)
set_target_properties(armfw_core_m4 PROPERTIES C_EXTENSIONS OFF)

# ST CMSIS device header + Arm CMSIS-Core (vendored, see third_party/fetch.sh)
set(CMSIS_INC ${TP}/cmsis_device_g4/Include ${TP}/cmsis_5/CMSIS/Core/Include)

# TinyUSB (device stack, CDC class, STM32 FSDEV driver). Third-party code: same warnings, no
# -Werror, so a compiler upgrade cannot break the build on a vendored file.
set(TUSB ${TP}/tinyusb/src)
add_library(tinyusb_m4 STATIC
  ${TUSB}/tusb.c ${TUSB}/common/tusb_fifo.c ${TUSB}/device/usbd.c
  ${TUSB}/class/cdc/cdc_device.c ${TUSB}/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
  ${TUSB}/portable/st/stm32_fsdev/fsdev_common.c)
target_include_directories(tinyusb_m4 PUBLIC ${TUSB} ${CMAKE_CURRENT_SOURCE_DIR} ${CMSIS_INC})
target_compile_definitions(tinyusb_m4 PUBLIC STM32G474xx)
target_compile_options(tinyusb_m4 PRIVATE ${MCU_FLAGS} -Wall -Wextra -ffunction-sections -fdata-sections)

# Target modules that use ST's register definitions (ADC, flash, USB glue/descriptors).
set(TARGET_CMSIS_SRC adc_stm32g4.c flash_stm32g4.c)
if(ARMFW_HOST_LINK_USB)
  list(APPEND TARGET_CMSIS_SRC usb_cdc_glue.c usb_descriptors.c)
endif()
add_library(target_cmsis_m4 OBJECT ${TARGET_CMSIS_SRC})
target_include_directories(target_cmsis_m4 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMSIS_INC})
target_compile_definitions(target_cmsis_m4 PRIVATE STM32G474xx)
target_compile_options(target_cmsis_m4 PRIVATE ${MCU_FLAGS} ${WARN} -ffunction-sections -fdata-sections -fstack-usage)
set_target_properties(target_cmsis_m4 PROPERTIES C_EXTENSIONS ON)
target_link_libraries(target_cmsis_m4 PUBLIC armfw_core_m4)
if(ARMFW_HOST_LINK_USB)
  target_link_libraries(target_cmsis_m4 PUBLIC tinyusb_m4)
endif()

# Target layer on the minimal register header (stm32g4_min.h, checked by tools/check_regs.py).
# GNU C (inline asm, range designators in the vector table).
add_executable(armfw_stm32g474.elf
  main.c hal_stm32g474.c startup_stm32g474.c system_stm32g474.c)
set_target_properties(armfw_stm32g474.elf PROPERTIES C_EXTENSIONS ON)
target_include_directories(armfw_stm32g474.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(armfw_stm32g474.elf PRIVATE ${MCU_FLAGS} ${WARN} -ffunction-sections -fdata-sections
                       -fstack-usage)
if(ARMFW_HOST_LINK_USB)
  target_compile_definitions(armfw_stm32g474.elf PRIVATE ARMFW_HOST_LINK_USB)
endif()
target_link_libraries(armfw_stm32g474.elf PRIVATE target_cmsis_m4 armfw_core_m4)
target_link_options(armfw_stm32g474.elf PRIVATE ${MCU_FLAGS} -nostartfiles --specs=nano.specs --specs=nosys.specs
                    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32g474re.ld -Wl,--gc-sections
                    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/armfw_stm32g474.map -Wl,--print-memory-usage)

add_custom_command(TARGET armfw_stm32g474.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:armfw_stm32g474.elf> ${CMAKE_CURRENT_BINARY_DIR}/armfw_stm32g474.bin
  COMMAND ${CMAKE_SIZE} -A -d $<TARGET_FILE:armfw_stm32g474.elf>
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:armfw_stm32g474.elf>)
