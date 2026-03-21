add_executable(test_distance_matrix test_distance_matrix.cpp)
target_link_libraries(test_distance_matrix PRIVATE periple::periple)
add_test(NAME distance_matrix COMMAND test_distance_matrix)

add_executable(test_solver test_solver.cpp)
target_link_libraries(test_solver PRIVATE periple::periple)
add_test(NAME solver COMMAND test_solver)

add_executable(test_jonker_volgenant test_jonker_volgenant.cpp)
target_link_libraries(test_jonker_volgenant PRIVATE periple::periple)
add_test(NAME jonker_volgenant COMMAND test_jonker_volgenant)

add_executable(test_tsplib_parser test_tsplib_parser.cpp)
target_link_libraries(test_tsplib_parser PRIVATE periple::periple)
target_compile_definitions(test_tsplib_parser PRIVATE
    PERIPLE_INSTANCES_DIR="${CMAKE_SOURCE_DIR}/benchmarks/instances")
add_test(NAME tsplib_parser COMMAND test_tsplib_parser)

add_subdirectory(algorithms)
