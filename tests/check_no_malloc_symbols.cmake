# Fails if the static library references any heap allocation function.
# Usage: cmake -DNM=<nm> -DLIB=<libkalman.a> -P check_no_malloc_symbols.cmake
execute_process(COMMAND ${NM} -u ${LIB} OUTPUT_VARIABLE undefined RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "nm failed on ${LIB}")
endif()
# One symbol per line: "U malloc" (GNU), "_malloc" (Apple), possibly indented.
string(REGEX MATCHALL "\n[ \tU]*_?(malloc|calloc|realloc|free|aligned_alloc)\n" hits "\n${undefined}\n")
if(hits)
    message(FATAL_ERROR "libkalman references heap functions:\n${hits}")
endif()
message(STATUS "No heap allocation symbols referenced by ${LIB}")
