cmake_minimum_required(VERSION 3.16)
project(kalman_c VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(KF_USE_DOUBLE "Use double precision" OFF)
option(KF_BUILD_TESTS "Build tests" ON)
option(KF_BUILD_BENCH "Build the comparison benchmarks" OFF)

add_library(kalman STATIC
    src/kf_linalg.c src/kf_linear.c src/kf_ekf.c src/kf_ukf.c)
target_include_directories(kalman PUBLIC include)
target_compile_options(kalman PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(kalman PUBLIC KALMAN_C_VERSION="${PROJECT_VERSION}")

if(KF_USE_DOUBLE)
    target_compile_definitions(kalman PUBLIC KF_USE_DOUBLE)
endif()

if(KF_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(KF_BUILD_BENCH)
    message(FATAL_ERROR "KF_BUILD_BENCH: bench/ is not implemented yet (see Step 8 of the guide)")
endif()
