execute_process(
    COMMAND git describe --always --dirty
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE PERIPLE_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

add_executable(periple_bench src/main.cpp)
target_link_libraries(periple_bench PRIVATE periple::periple)
target_compile_definitions(periple_bench PRIVATE
    PERIPLE_INSTANCES_DIR="${CMAKE_SOURCE_DIR}/benchmarks/instances"
    PERIPLE_OPTIMA_JSON="${CMAKE_SOURCE_DIR}/benchmarks/optima.json"
    PERIPLE_GIT_COMMIT="${PERIPLE_GIT_COMMIT}"
)