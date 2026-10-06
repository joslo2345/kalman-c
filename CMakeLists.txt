cmake_minimum_required(VERSION 3.16)

# The version's single source is KALMAN_C_VERSION in kf_config.h.
file(STRINGS include/kalman/kf_config.h _kf_version_line REGEX "^#define KALMAN_C_VERSION ")
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" KF_VERSION "${_kf_version_line}")
project(kalman_c VERSION ${KF_VERSION} LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(KF_USE_DOUBLE "Use double precision" OFF)
option(KF_BUILD_TESTS "Build tests" ON)
option(KF_BUILD_BENCH "Build the comparison benchmarks" OFF)
option(KF_BUILD_EXAMPLES "Build the worked examples" ON)
set(KF_MAX_STATE "" CACHE STRING "Override the maximum state size (default 12)")
set(KF_MAX_MEAS "" CACHE STRING "Override the maximum measurement size (default 6)")
set(KF_SPECIALIZE "" CACHE STRING
    "Compile-time KF/EKF specializations, e.g. KF_SIZE(4,2)KF_SIZE(15,6) (see kf_config.h)")

# Scenario S5 has 15 states and 6 measurements.
if(KF_BUILD_BENCH AND KF_MAX_STATE STREQUAL "")
    set(KF_MAX_STATE 15)
endif()

add_library(kalman STATIC
    src/kf_linalg.c src/kf_linear.c src/kf_ekf.c src/kf_ukf.c src/kf_sqrt.c)
target_include_directories(kalman PUBLIC include)
target_compile_options(kalman PRIVATE -Wall -Wextra -Wpedantic)

# sqrt/isfinite live in a separate libm on most non-Apple toolchains.
find_library(KF_LIBM m)
if(KF_LIBM)
    target_link_libraries(kalman PUBLIC ${KF_LIBM})
endif()

if(KF_USE_DOUBLE)
    target_compile_definitions(kalman PUBLIC KF_USE_DOUBLE)
endif()
if(NOT KF_MAX_STATE STREQUAL "")
    target_compile_definitions(kalman PUBLIC KF_MAX_STATE=${KF_MAX_STATE})
endif()
if(NOT KF_MAX_MEAS STREQUAL "")
    target_compile_definitions(kalman PUBLIC KF_MAX_MEAS=${KF_MAX_MEAS})
endif()
# Passed through target_compile_definitions, which escapes the parentheses for
# the shell. Putting it in CMAKE_C_FLAGS silently drops it with Makefiles.
if(NOT KF_SPECIALIZE STREQUAL "")
    target_compile_definitions(kalman PRIVATE "KF_SPECIALIZE=${KF_SPECIALIZE}")
endif()

if(KF_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(KF_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(KF_BUILD_BENCH)
    add_subdirectory(bench)
endif()
