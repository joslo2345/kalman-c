# Desktop comparison benchmarks (guide Step 8). Reuses the baselines from tests/.
set(KF_TESTS_DIR ${PROJECT_SOURCE_DIR}/tests)
set(TINYEKF_DIR ${KF_TESTS_DIR}/baselines/tinyekf)
if(NOT EXISTS ${TINYEKF_DIR}/src/tinyekf.h)
    message(FATAL_ERROR "TinyEKF baseline missing. Run: git submodule update --init")
endif()

execute_process(
    COMMAND git -C ${TINYEKF_DIR} rev-parse --short HEAD
    OUTPUT_VARIABLE TINYEKF_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE)

add_executable(bench
    bench_main.c
    scenario.c
    runners/metrics.c
    runners/run_ours.c
    runners/run_naive.c
    runners/run_tinyekf.c
    runners/tinyekf_2x1.c
    runners/tinyekf_4x2.c
    runners/tinyekf_15x6.c
    ${KF_TESTS_DIR}/baselines/naive_kf.c)
target_include_directories(bench PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR} ${KF_TESTS_DIR} ${TINYEKF_DIR}/src)
target_link_libraries(bench PRIVATE kalman)
if(KF_SPECIALIZE STREQUAL "")
    set(KF_BENCH_NAME "kalman-c")
else()
    set(KF_BENCH_NAME "kalman-c-specialized")
endif()
target_compile_definitions(bench PRIVATE
    KF_BENCH_NAME="${KF_BENCH_NAME}"
    TINYEKF_COMMIT="${TINYEKF_COMMIT}"
    KF_VECTORS_DIR="${KF_TESTS_DIR}/vectors")
target_compile_options(bench PRIVATE -Wall -Wextra)

# Third-party header: non-C99 empty initializers and unused statics.
set_source_files_properties(
    runners/tinyekf_2x1.c runners/tinyekf_4x2.c runners/tinyekf_15x6.c
    PROPERTIES COMPILE_OPTIONS "-w")
