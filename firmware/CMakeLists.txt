cmake_minimum_required(VERSION 3.16)
project(armfw C)

# Host build of the portable firmware core, the simulation harness, libarmfw.so and tests.
# The STM32 cross build lives in targets/stm32g474/ (separate CMake project, see README).

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(ARMFW_SANITIZE "Build everything with -fsanitize=address,undefined" OFF)

set(ARMFW_STRICT -Wall -Wextra -Werror -pedantic -Wshadow -Wdouble-promotion -Wconversion
    -Wno-sign-conversion -fno-math-errno)
if(ARMFW_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined)
endif()

# ---------------- portable core (no HAL, no malloc, no libm) ----------------
set(ARMFW_CORE_SRC
  core/src/crc16.c
  core/src/proto.c
  core/src/config.c
  core/src/traj.c
  core/src/safety.c
  core/src/servo_bus.c
  core/src/control.c
  core/src/analog.c
  core/src/calstore.c
  core/src/host_link.c)
add_library(armfw_core STATIC ${ARMFW_CORE_SRC})
target_include_directories(armfw_core PUBLIC core/include)
target_compile_options(armfw_core PRIVATE ${ARMFW_STRICT})

# ---------------- host simulation (fake servos + fake HAL) ----------------
add_library(armfw_sim STATIC sim/fake_servo.c sim/fake_flash.c sim/sim.c)
target_include_directories(armfw_sim PUBLIC sim)
target_link_libraries(armfw_sim PUBLIC armfw_core)
target_compile_options(armfw_sim PRIVATE ${ARMFW_STRICT})

# ---------------- libarmfw.so: stable C API for the host driver ----------------
add_library(armfw SHARED lib/armfw_api.c ${ARMFW_CORE_SRC} sim/fake_servo.c sim/fake_flash.c sim/sim.c)
target_include_directories(armfw PUBLIC lib core/include sim)
target_compile_definitions(armfw PRIVATE ARMFW_BUILDING_LIB)
target_compile_options(armfw PRIVATE ${ARMFW_STRICT} -fvisibility=hidden)
set_target_properties(armfw PROPERTIES OUTPUT_NAME armfw VERSION 1.0.0 SOVERSION 1)

# ---------------- fw_sim CLI ----------------
add_executable(fw_sim tools/fw_sim.c)
target_link_libraries(fw_sim PRIVATE armfw_sim)
target_compile_definitions(fw_sim PRIVATE _POSIX_C_SOURCE=200809L)
target_compile_options(fw_sim PRIVATE ${ARMFW_STRICT})

# ---------------- tests ----------------
enable_testing()
file(GLOB ARMFW_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_*.c)
foreach(src ${ARMFW_TEST_SOURCES})
  get_filename_component(exe ${src} NAME_WE)
  add_executable(${exe} ${src})
  target_link_libraries(${exe} PRIVATE armfw_sim m)
  target_include_directories(${exe} PRIVATE tests lib)
  target_compile_definitions(${exe} PRIVATE _POSIX_C_SOURCE=200809L)
  target_compile_options(${exe} PRIVATE -Wall -Wextra -Werror)
  if(exe STREQUAL "test_host_link")
    find_package(Threads REQUIRED)
    target_link_libraries(${exe} PRIVATE Threads::Threads)
  endif()
  if(exe STREQUAL "test_lib_api")
    target_link_libraries(${exe} PRIVATE armfw)
  endif()
  # Register each TT_TEST(name) as its own ctest case (re-scanned whenever the file changes).
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${src})
  file(STRINGS ${src} lines REGEX "^TT_TEST\\(")
  foreach(line ${lines})
    string(REGEX REPLACE "^TT_TEST\\(([A-Za-z0-9_]+)\\).*" "\\1" tname "${line}")
    string(REPLACE "test_" "" mod ${exe})
    add_test(NAME ${mod}.${tname} COMMAND ${exe} ${tname})
  endforeach()
endforeach()

# fw_sim smoke test: pipe a PING + SET_TARGETS stream through the CLI and check the replies.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
  if(ARMFW_SANITIZE)
    # an ASan-instrumented .so cannot be dlopen'ed by a non-ASan python; test the CLI only
    set(ARMFW_PIPE_LIB --no-lib)
  else()
    set(ARMFW_PIPE_LIB $<TARGET_FILE:armfw>)
  endif()
  add_test(NAME cli.fw_sim_pipe
           COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/fw_sim_pipe_test.py
                   $<TARGET_FILE:fw_sim> ${ARMFW_PIPE_LIB})
endif()
