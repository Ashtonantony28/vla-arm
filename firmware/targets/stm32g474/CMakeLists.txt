# Cross build of the armfw core + STM32G474 target layer.
#   cmake -S targets/stm32g474 -B build-arm -DCMAKE_TOOLCHAIN_FILE=../../cmake/arm-none-eabi.cmake
#   cmake --build build-arm
cmake_minimum_required(VERSION 3.16)
project(armfw_stm32g474 C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE MinSizeRel)
endif()
set(FW_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(WARN -Wall -Wextra -Werror -Wdouble-promotion -Wshadow -fno-math-errno)

# Portable core: strict ISO C11, same as the host build.
add_library(armfw_core_m4 STATIC
  ${FW_ROOT}/core/src/crc16.c ${FW_ROOT}/core/src/proto.c ${FW_ROOT}/core/src/config.c
  ${FW_ROOT}/core/src/traj.c ${FW_ROOT}/core/src/safety.c ${FW_ROOT}/core/src/servo_bus.c
  ${FW_ROOT}/core/src/control.c)
target_include_directories(armfw_core_m4 PUBLIC ${FW_ROOT}/core/include)
target_compile_options(armfw_core_m4 PRIVATE ${MCU_FLAGS} ${WARN} -pedantic -Wconversion -Wno-sign-conversion
                       -ffunction-sections -fdata-sections -fstack-usage)
set_target_properties(armfw_core_m4 PROPERTIES C_EXTENSIONS OFF)

# Target layer: GNU C (inline asm, range designators in the vector table).
add_executable(armfw_stm32g474.elf
  main.c hal_stm32g474.c startup_stm32g474.c system_stm32g474.c usb_cdc_glue.c)
set_target_properties(armfw_stm32g474.elf PROPERTIES C_EXTENSIONS ON)
target_include_directories(armfw_stm32g474.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(armfw_stm32g474.elf PRIVATE ${MCU_FLAGS} ${WARN} -ffunction-sections -fdata-sections
                       -fstack-usage)
target_link_libraries(armfw_stm32g474.elf PRIVATE armfw_core_m4)
target_link_options(armfw_stm32g474.elf PRIVATE ${MCU_FLAGS} -nostartfiles --specs=nano.specs --specs=nosys.specs
                    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32g474re.ld -Wl,--gc-sections
                    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/armfw_stm32g474.map -Wl,--print-memory-usage)

add_custom_command(TARGET armfw_stm32g474.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:armfw_stm32g474.elf> ${CMAKE_CURRENT_BINARY_DIR}/armfw_stm32g474.bin
  COMMAND ${CMAKE_SIZE} -A -d $<TARGET_FILE:armfw_stm32g474.elf>
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:armfw_stm32g474.elf>)
