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

# Each program must abort on an assert whose message matches.
function(periple_death_test name message)
    add_executable(death_${name} death/${name}.cpp)
    target_link_libraries(death_${name} PRIVATE periple::periple)
    add_test(NAME death_${name}
             COMMAND ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:death_${name}>
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/death/expect_abort.cmake)
    set_tests_properties(death_${name} PROPERTIES PASS_REGULAR_EXPRESSION "${message}")
endfunction()

periple_death_test(replay_reads_tour "ctx.tour\\(\\): no partial tour during a replay or a DP")
periple_death_test(dp_reads_tour "ctx.tour\\(\\): no partial tour during a replay or a DP")

# Each file must fail to compile with the given static_assert message.
function(periple_compile_fail name message)
    add_executable(compile_fail_${name} EXCLUDE_FROM_ALL compile_fail/${name}.cpp)
    target_link_libraries(compile_fail_${name} PRIVATE periple::periple)
    add_test(NAME compile_fail_${name}
             COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target compile_fail_${name})
    # Each test builds in the shared tree: never two at once under ctest -j.
    set_tests_properties(compile_fail_${name} PROPERTIES
        PASS_REGULAR_EXPRESSION "${message}"
        RESOURCE_LOCK periple_build_tree)
endfunction()

periple_compile_fail(append_only_variant_held_karp
    "held_karp: the variant, or one of its components, has no DPMove callback")
periple_compile_fail(composed_partial_held_karp
    "held_karp: the variant, or one of its components, has no DPMove callback")
periple_compile_fail(dp_only_variant
    "variant has callbacks but none for AppendMove")
periple_compile_fail(append_only_dimension_held_karp
    "held_karp: a dimension has no DPMove init or commit")
periple_compile_fail(unstaged_dimension_two_opt
    "evaluate_replay: a dimension commits AppendMove state without staging")
