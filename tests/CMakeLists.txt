include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.0)
FetchContent_MakeAvailable(unity)
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_DOUBLE)

set(TINYEKF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/baselines/tinyekf)
if(NOT EXISTS ${TINYEKF_DIR}/src/tinyekf.h)
    message(FATAL_ERROR "TinyEKF baseline missing. Run: git submodule update --init")
endif()

# Scenarios and baselines shared by the comparison tests (and later the benchmarks).
add_library(kf_test_support STATIC
    scenarios/rng.c
    scenarios/constant_velocity.c
    scenarios/ill_conditioned.c
    baselines/naive_kf.c
    baselines/baseline_tinyekf.c)
target_include_directories(kf_test_support
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${TINYEKF_DIR}/src)
target_link_libraries(kf_test_support PUBLIC kalman)
target_compile_options(kf_test_support PRIVATE -Wall -Wextra)
# Third-party header: uses non-C99 empty initializers and has unused statics.
set_source_files_properties(baselines/baseline_tinyekf.c PROPERTIES COMPILE_OPTIONS "-w")

function(kf_add_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE kf_test_support unity)
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

kf_add_test(test_linalg unit/test_linalg.c)
kf_add_test(test_linear unit/test_linear.c)
kf_add_test(test_ekf unit/test_ekf.c)
kf_add_test(test_ukf unit/test_ukf.c)
kf_add_test(test_errors unit/test_errors.c)

kf_add_test(test_equivalence comparison/test_equivalence.c)
kf_add_test(test_stability comparison/test_stability.c)
kf_add_test(test_consistency comparison/test_consistency.c)
set_tests_properties(test_stability PROPERTIES TIMEOUT 600)

# Zero heap allocations, checked two ways: the library's symbol table on every
# toolchain, and wrapped allocators at run time where the linker supports --wrap.
add_test(NAME no_malloc_symbols
    COMMAND ${CMAKE_COMMAND} -DNM=${CMAKE_NM} -DLIB=$<TARGET_FILE:kalman>
            -P ${CMAKE_CURRENT_SOURCE_DIR}/check_no_malloc_symbols.cmake)
if(NOT APPLE)
    kf_add_test(no_malloc comparison/test_no_malloc.c)
    target_link_options(no_malloc PRIVATE
        -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc)
else()
    message(STATUS "Skipping no_malloc run-time test: linker does not support --wrap")
endif()
