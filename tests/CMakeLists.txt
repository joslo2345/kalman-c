include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.0)
FetchContent_MakeAvailable(unity)

function(kf_add_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE kalman unity)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

kf_add_test(test_errors unit/test_errors.c)

# --wrap is a GNU ld / lld feature; Apple's linker doesn't support it.
if(NOT APPLE)
    kf_add_test(no_malloc comparison/test_no_malloc.c)
    target_link_options(no_malloc PRIVATE
        -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc)
else()
    message(STATUS "Skipping no_malloc test: linker does not support --wrap")
endif()
