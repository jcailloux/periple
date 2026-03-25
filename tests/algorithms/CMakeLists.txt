add_executable(test_algorithms test_algorithms.cpp)
target_link_libraries(test_algorithms PRIVATE periple::periple)
target_compile_definitions(test_algorithms PRIVATE PERIPLE_TESTING)
target_include_directories(test_algorithms PRIVATE ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME algorithms COMMAND test_algorithms)

add_executable(test_variants test_variants.cpp)
target_link_libraries(test_variants PRIVATE periple::periple)
add_test(NAME variants COMMAND test_variants)
