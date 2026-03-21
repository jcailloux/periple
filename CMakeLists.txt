cmake_minimum_required(VERSION 3.21)
project(periple
    VERSION 0.1.0
    LANGUAGES CXX
    DESCRIPTION "C++20 header-only TSP solver library"
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(periple INTERFACE)
add_library(periple::periple ALIAS periple)

target_include_directories(periple INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(periple INTERFACE cxx_std_20)

# --- Install ---

install(TARGETS periple EXPORT periple-targets)
install(EXPORT periple-targets
    FILE periple-targets.cmake
    NAMESPACE periple::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/periple
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/periple-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/periple-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/periple
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/periple-config-version.cmake
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/periple-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/periple-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/periple
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# --- Tests ---
option(PERIPLE_BUILD_TESTS "Build tests" ${PROJECT_IS_TOP_LEVEL})
if(PERIPLE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# --- Benchmarks ---
option(PERIPLE_BUILD_BENCHMARKS "Build benchmarks" ${PROJECT_IS_TOP_LEVEL})
if(PERIPLE_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()
