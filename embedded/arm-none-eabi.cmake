# Toolchain file for the bare-metal Cortex-M firmware.
# Set ARM_TOOLCHAIN_DIR (cache or environment) to the Arm GNU Toolchain root,
# or put arm-none-eabi-gcc on PATH.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_TOOLCHAIN_DIR "$ENV{ARM_TOOLCHAIN_DIR}" CACHE PATH "Arm GNU Toolchain root")
if(ARM_TOOLCHAIN_DIR)
    set(_prefix "${ARM_TOOLCHAIN_DIR}/bin/")
endif()
set(CMAKE_C_COMPILER "${_prefix}arm-none-eabi-gcc")
set(CMAKE_SIZE "${_prefix}arm-none-eabi-size" CACHE FILEPATH "arm-none-eabi-size")
