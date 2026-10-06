# Cortex-M4F benchmark firmware (guide Step 8, embedded numbers), built for an
# STM32F405 and run under QEMU (netduinoplus2). Configure with:
#   cmake -S embedded -B build-fw -DCMAKE_TOOLCHAIN_FILE=embedded/arm-none-eabi.cmake
cmake_minimum_required(VERSION 3.16)
project(kalman_c_firmware C)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(TINYEKF_DIR ${ROOT}/tests/baselines/tinyekf)

# The same compiler and flags for every image of a target, as the guide requires.
# m4: STM32F405 (Cortex-M4F, hardware single precision), QEMU netduinoplus2.
# m3: STM32F205 (Cortex-M3, no FPU: floats are emulated in software), QEMU netduino2.
set(FW_CPU_m4 -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(FW_CPU_m3 -mcpu=cortex-m3 -mfloat-abi=soft)
set(FW_COMMON_CFLAGS -O2 -g -std=c99 -ffunction-sections -fdata-sections -Wall -Wextra)
set(FW_COMMON_LDFLAGS -nostartfiles -T${CMAKE_CURRENT_SOURCE_DIR}/stm32f405.ld
    -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs)

add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/fw_data.h
    COMMAND Python3::Interpreter ${ROOT}/scripts/gen_fw_data.py
            ${ROOT}/tests/vectors/S2.txt ${CMAKE_BINARY_DIR}/fw_data.h 1000
    DEPENDS ${ROOT}/scripts/gen_fw_data.py ${ROOT}/tests/vectors/S2.txt)
add_custom_target(fw_data DEPENDS ${CMAKE_BINARY_DIR}/fw_data.h)

set(KALMAN_SOURCES
    ${ROOT}/src/kf_linalg.c ${ROOT}/src/kf_linear.c ${ROOT}/src/kf_ekf.c ${ROOT}/src/kf_ukf.c
    ${ROOT}/src/kf_sqrt.c ${ROOT}/src/kf_fixed.c)

# <prefix><name>_1000 runs 1000 steps of S2; <prefix><name>_0 is identical but
# runs none. The M4F images keep the fw_ prefix; the M3 images use fw_m3_.
function(add_firmware name)
    cmake_parse_arguments(FW "" "TARGET" "SOURCES;DEFINES;INCLUDES;QUIET_SOURCES" ${ARGN})
    if(NOT FW_TARGET)
        set(FW_TARGET m4)
    endif()
    if(FW_TARGET STREQUAL "m4")
        set(prefix fw_)
    else()
        set(prefix fw_${FW_TARGET}_)
    endif()
    foreach(steps 1000 0)
        set(t ${prefix}${name}_${steps})
        add_executable(${t} startup.c semihost.c fw_main.c ${FW_SOURCES} ${FW_QUIET_SOURCES})
        set_target_properties(${t} PROPERTIES SUFFIX .elf)
        add_dependencies(${t} fw_data)
        target_include_directories(${t} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR} ${ROOT}/include ${ROOT}/tests
            ${FW_INCLUDES})
        target_compile_definitions(${t} PRIVATE
            FW_STEPS=${steps} KF_MAX_STATE=4 KF_MAX_MEAS=2 ${FW_DEFINES})
        target_compile_options(${t} PRIVATE ${FW_CPU_${FW_TARGET}} ${FW_COMMON_CFLAGS})
        target_link_options(${t} PRIVATE ${FW_CPU_${FW_TARGET}} ${FW_COMMON_LDFLAGS})
        target_link_libraries(${t} PRIVATE m)
    endforeach()
    if(FW_QUIET_SOURCES)
        # Third-party code: not ours to fix warnings in.
        set_source_files_properties(${FW_QUIET_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
    endif()
endfunction()

add_firmware(empty SOURCES fw_empty.c)
add_firmware(kalman_c SOURCES fw_kalman_c.c ${KALMAN_SOURCES} INCLUDES ${ROOT}/src)
add_firmware(kalman_c_specialized SOURCES fw_kalman_c.c ${KALMAN_SOURCES}
    INCLUDES ${ROOT}/src DEFINES "KF_SPECIALIZE=KF_SIZE(4,2)")
add_firmware(kalman_c_sr SOURCES fw_kalman_c_sr.c ${KALMAN_SOURCES} INCLUDES ${ROOT}/src)
add_firmware(naive SOURCES fw_naive.c ${ROOT}/tests/baselines/naive_kf.c)
add_firmware(tinyekf QUIET_SOURCES fw_tinyekf.c INCLUDES ${TINYEKF_DIR}/src)
add_firmware(kalman_c_fixed SOURCES fw_kalman_c_fixed.c ${KALMAN_SOURCES} INCLUDES ${ROOT}/src)

# Cortex-M3 without an FPU: float filters run on software floating point.
add_firmware(empty TARGET m3 SOURCES fw_empty.c)
add_firmware(kalman_c TARGET m3 SOURCES fw_kalman_c.c ${KALMAN_SOURCES} INCLUDES ${ROOT}/src)
add_firmware(kalman_c_fixed TARGET m3 SOURCES fw_kalman_c_fixed.c ${KALMAN_SOURCES}
    INCLUDES ${ROOT}/src)
add_firmware(tinyekf TARGET m3 QUIET_SOURCES fw_tinyekf.c INCLUDES ${TINYEKF_DIR}/src)
