# Worked examples. Each simulates its own data and exits non-zero if the filter
# does not clearly beat the raw sensor, so they also run as tests.
foreach(example constant_velocity imu_attitude)
    add_executable(example_${example} ${example}.c)
    target_link_libraries(example_${example} PRIVATE kalman)
    target_compile_options(example_${example} PRIVATE -Wall -Wextra)
    if(KF_BUILD_TESTS)
        add_test(NAME example_${example} COMMAND example_${example})
    endif()
endforeach()

# The README's quick start, extracted at configure time so it cannot drift
# from the API.
file(READ ${PROJECT_SOURCE_DIR}/README.md KF_README)
string(REGEX MATCH "## Quick start\n\n```c\n([^`]*)```" _ "${KF_README}")
if(NOT CMAKE_MATCH_1)
    message(FATAL_ERROR "README.md: could not find the C quick start block")
endif()
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/readme_quickstart.c "${CMAKE_MATCH_1}")
add_executable(example_readme_quickstart ${CMAKE_CURRENT_BINARY_DIR}/readme_quickstart.c)
target_link_libraries(example_readme_quickstart PRIVATE kalman)
target_compile_options(example_readme_quickstart PRIVATE -Wall -Wextra -Wpedantic)
if(KF_BUILD_TESTS)
    add_test(NAME example_readme_quickstart COMMAND example_readme_quickstart)
endif()
