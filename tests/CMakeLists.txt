# Tests are assert-based: keep assert active in every build type.
foreach(config RELEASE RELWITHDEBINFO MINSIZEREL)
    string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_${config} "${CMAKE_CXX_FLAGS_${config}}")
endforeach()

add_executable(test_distance_matrix test_distance_matrix.cpp)
target_link_libraries(test_distance_matrix PRIVATE periple::periple)
add_test(NAME distance_matrix COMMAND test_distance_matrix)

add_executable(test_solver test_solver.cpp)
target_link_libraries(test_solver PRIVATE periple::periple)
target_compile_definitions(test_solver PRIVATE PERIPLE_TESTING)
target_include_directories(test_solver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME solver COMMAND test_solver)

add_executable(test_jonker_volgenant test_jonker_volgenant.cpp)
target_link_libraries(test_jonker_volgenant PRIVATE periple::periple)
add_test(NAME jonker_volgenant COMMAND test_jonker_volgenant)

add_executable(test_tsplib_parser test_tsplib_parser.cpp)
target_link_libraries(test_tsplib_parser PRIVATE periple::periple)
target_compile_definitions(test_tsplib_parser PRIVATE
    PERIPLE_INSTANCES_DIR="${CMAKE_SOURCE_DIR}/benchmarks/instances")
add_test(NAME tsplib_parser COMMAND test_tsplib_parser)

add_executable(test_callbacks test_callbacks.cpp)
target_link_libraries(test_callbacks PRIVATE periple::periple)
target_include_directories(test_callbacks PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME callbacks COMMAND test_callbacks)

add_executable(test_composed test_composed.cpp)
target_link_libraries(test_composed PRIVATE periple::periple)
add_test(NAME composed COMMAND test_composed)

add_executable(test_replay test_replay.cpp)
target_link_libraries(test_replay PRIVATE periple::periple)
target_compile_definitions(test_replay PRIVATE PERIPLE_TESTING)
target_include_directories(test_replay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME replay COMMAND test_replay)

add_subdirectory(algorithms)
